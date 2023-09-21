/*
 * foc.c
 *
 *  Created on: 16 janv. 2021
 *      Author: Patrick, Kai
 */

/// DOC FOC(open loop) : https://docs.simplefoc.com/foc_theory
/// DOC SVM https://www.embedded.com/painless-mcu-implementation-of-space-vector-modulation-for-electric-motor-systems/

#include "foc.h"
#include "foc_utils.h"
#include "cordic.h"
#include "serial.h"
#include "position_sensor.h"
#include "math_tool.h"
#include "control_table.h"
#include "binary_tool.h"
#include "pid.h"

#include <string.h>
#include <math.h>
#include <stdlib.h>

// hard-coded settings
#define ALPHA_CURRENT_SENSE_OFFSET 0.001f // low pass filter for calibrating the phase current ADC offset (automatically)

// FOC peripherals
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim6;
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc2;
extern OPAMP_HandleTypeDef hopamp1;
extern OPAMP_HandleTypeDef hopamp2;
extern OPAMP_HandleTypeDef hopamp3;

// serial communication (UART2) for TRACEs
// TODO : use STM32 CUBE MONITOR
extern HAL_Serial_Handler serial;

// high priority high interupt
// TIM1 => Update Event Trigger => CAN (x2) ==> DMA (x2) ==> FOC IT
void API_FOC_It(ADC_HandleTypeDef *hadc) __attribute__((section(".ccmram")));

// high priority high frequency process called by IT
void API_FOC_Torque_Update() __attribute__((section(".ccmram")));

// ADC IT for motor current sense, and votlage/temperature monitoring
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) __attribute__((section(".ccmram")));

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)
{
	API_FOC_It(hadc);
}

// FOC state variable
#define FOC_STATE_IDLE 0		   // brake
#define FOC_STATE_TORQUE_CONTROL 1 // normal operation
#define FOC_STATE_FLUX_CONTROL 10  // calibration
static uint32_t foc_state = FOC_STATE_IDLE;
static uint16_t foc_timestamp_us = 0;
static uint32_t foc_timestamp_ms = 0;

// FOC setpoints variables
static float setpoint_torque_current_mA = 0.0f;
static float setpoint_flux_current_mA = 0.0f;
static float setpoint_electrical_angle_rad = 0.0f;
static float setpoint_flux_voltage_V = 0.0f;

// FOC variables
static float present_Ids_mA = 0.0;
static float present_Iqs_mA = 0.0f;
static pid_context_t flux_pi;
static pid_context_t torque_pi;
float theta_rad = 0.0f;				// public // DEBUG
float absolute_position_rad = 0.0f; // public // DEBUG

// FOC current sense
static float motor_current_mA[3] = {0.0f, 0.0f, 0.0f};
static float motor_current_input_adc_offset[3] = {2464.0f, 2482.0f, 2485.0f};
static const float motor_current_input_adc_KmA = -29.41f; // V/mA // note : the (-) sign here
// process phase current
// Note : when current flows inward phase, shunt voltage is negative
// Note : when current flows outward phase, shunt voltage is positive
// Note : The current sign is positive when flowing in to a phase
// Note : The current sign is negative when flowing out from a phase

// FOC analog measure
static int32_t current_samples = 0;
volatile uint16_t ADC1_DMA[5] = {0, 0, 0, 0, 0}; // Dummy conversion (ST workaround for -x),
volatile uint16_t ADC2_DMA[3] = {0, 0, 0};		 // Dummy conversion (ST workaround for -x)
static float motor_current_input_adc[3] = {0.0f, 0.0f, 0.0f};
float potentiometer_input_adc = 0.0f; // public
static float vbus_input_adc = 0.0f;
static float temperature_input_adc = 0.0f;
static float present_voltage_V = 0.0f;
static float present_temperature_C = 0.0f;

// FOC performance monitoring
static float average_processing_time_us = 0.0f;
static uint32_t foc_counter = 0;

void API_FOC_Torque_Enable()
{
	foc_state = FOC_STATE_TORQUE_CONTROL;

	setpoint_torque_current_mA = 0.0f;
	setpoint_flux_current_mA = 0.0f;
	setpoint_electrical_angle_rad = 0.0f;
	setpoint_flux_voltage_V = 0.0f;

	present_Ids_mA = 0.0f;
	present_Iqs_mA = 0.0f;

	pid_reset(&flux_pi);
	pid_reset(&torque_pi);
}

void API_FOC_Torque_Disable()
{
	foc_state = FOC_STATE_IDLE;
	HAL_Delay(1);

	// enforce brake
	LL_FOC_brake();
}

void API_FOC_Set_Torque_Flux_Currents_mA(float Iq_mA, float Id_mA)
{
	setpoint_torque_current_mA = Iq_mA;
	setpoint_flux_current_mA = Id_mA;
}

// user API function
// this function reset state of FOC
// This function starts peripherals
void API_FOC_Init()
{
	// Motor PWM init and BRAKE
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, 0);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
	HAL_TIMEx_PWMN_Start(&htim1, TIM_CHANNEL_3);
	// OPAMP and ADC init
	HAL_OPAMP_Start(&hopamp1);
	HAL_OPAMP_Start(&hopamp2);
	HAL_OPAMP_Start(&hopamp3);
	HAL_ADC_Start_DMA(&hadc1, (uint32_t *)ADC1_DMA, 5);
	HAL_ADC_Start_DMA(&hadc2, (uint32_t *)ADC2_DMA, 3);
	// CORDIC init
	API_CORDIC_Processor_Init();
	// disable FOC
	API_FOC_Torque_Disable();
}

void LL_FOC_Update_Temperature() __attribute__((section(".ccmram")));

// low level function
// this function update present_temperature_C
// this function update REG_HARDWARE_ERROR_STATUS register (set/reset HW_ERROR_BIT_OVERHEATING bit)
// this function use REG_TEMPERATURE_LIMIT register
// the temperature ADC samples are collected with phase current samples
void LL_FOC_Update_Temperature()
{
	// convert ADC sample into temperature (STM32G431-ESC1 specific)
	static const float R60 = 4700.0f;											// ohm
	static const float eps = 0.1f;												// epsilon (avoid divide by zero)
	const float R_NTC = R60 * (4096.0f / (temperature_input_adc + eps) - 1.0f); // 10kohm NTC at 25°C
	static const float Beta = 3455.0f;											// for a 10k NTC
	static const float Kelvin = 273.15f;										// °C
	static const float T0 = 273.15f + 25.0f;
	static const float R0 = 10000.0f; // 10kohm at 25° for 10k NTC
	const float present_temperature_K = Beta * T0 / (Beta - T0 * logf(R0 / R_NTC));
	present_temperature_C = present_temperature_K - Kelvin;

	// apply thermal protection and update hardware error register
	const float max_temperature_C = regs[REG_TEMPERATURE_LIMIT];
	if (present_temperature_C > max_temperature_C)
	{
		// set overheating error
		regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_OVERHEATING;
		// HAL_Serial_Print(&serial,"h");
	}
	else if (((regs[REG_HARDWARE_ERROR_STATUS] & (1UL << HW_ERROR_BIT_OVERHEATING)) != 0)) // hard-coded hysteresis 12°C
	{
		if (present_temperature_C < max_temperature_C - 12.0f)
			// clear overheating error
			regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_OVERHEATING);
		// wait for cooling
	}
	else
	{
		// clear overheating error
		regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_OVERHEATING);
	}
}

void LL_FOC_Update_Voltage() __attribute__((section(".ccmram")));

// low level function
// this function update present_voltage_V
// this function update REG_HARDWARE_ERROR_STATUS register (set/reset HW_ERROR_BIT_VOLTAGE bit)
// this function use REG_LOW_VOLTAGE_LIMIT and REG_HIGH_VOLTAGE_LIMIT registers
// the voltage ADC samples are collected with phase current samples
void LL_FOC_Update_Voltage()
{
	// process input voltage (STM32G431-ESC1 specific)
	{
		static const float R68 = 169.0f; // kohm
		static const float R76 = 18.0f;	 // kohm
		static const float alpha_voltage = 0.05f;
		present_voltage_V = (vbus_input_adc / 4096.0f * 3.3f * (R68 + R76) / R76) * alpha_voltage + (1.0f - alpha_voltage) * present_voltage_V;
	}

	// apply voltage protection and update
	const float min_voltage_V = regs[REG_LOW_VOLTAGE_LIMIT];
	const float max_voltage_V = regs[REG_HIGH_VOLTAGE_LIMIT];
	if ((present_voltage_V > max_voltage_V) || (present_voltage_V < min_voltage_V))
	{
		// set voltage error
		regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_VOLTAGE;
		// HAL_Serial_Print(&serial,"v");
	}
	else
	{
		// clear voltage error
		regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_VOLTAGE);
	}
}

// user API function
// this function synchronize physical and electrical angles, set motor normal/reverse rotation, and check pole pairs
// this function uses REG_MOTOR_POLE_PAIRS register
int API_FOC_Calibrate()
{
	/* Rewrite for the MIT LUT logic */
	// Clear old lookup table
	memset(regs_lut, 0, sizeof(regs_lut));

	// Init
	// define general variables
	const int encoder_bits = regs[REG_ENCODER_BITS];
	const int lut_bits = REG_MAX_LUT_BITS;
	const int shift_bits = encoder_bits - lut_bits;
	const int npp = regs[REG_MOTOR_POLE_PAIRS];
	const int n = REG_MAX_LUT * npp; // number of positions to be sampled per mechanical rotation.  Multiple of NPP for filtering reasons (see later)
	const int n2 = 5;				 // increments between saved samples (for smoothing motion)
	const int n_lut = REG_MAX_LUT;
	const float cpr = pow(2, encoder_bits);
	const float calibration_voltage = 1.5f; // Put volts on the D-Axis
	float delta = M_2PI * npp / (n * n2);	// change in angle between samples
	// define arrays
	int16_t *error = (int16_t *)malloc(n * sizeof(int16_t));
	int16_t *error_filt = (int16_t *)malloc(n_lut * sizeof(int16_t));
	float theta_ref = 0;
	float theta_actual = 0;
	// save memory
	int raw_f_0 = 0;
	int raw_b_n1 = 0;
	float theta_start = 0;
	float theta_end = 0;
	float precison_multiplier = 10000; // max encoder deviation without messing up the LUT is "data type for error max value" / precison_multiplier rad

	// Start calibration
	HAL_Serial_Print(&serial, "Starting calibration procedure\n\r");

	// Reset controller output
	setpoint_electrical_angle_rad = 0.0f;
	setpoint_flux_voltage_V = 0.0f;
	foc_state = FOC_STATE_IDLE;
	HAL_Delay(200);
	// reset settings
	regs[REG_INV_PHASE_MOTOR] = 0;
	regs[REG_MOTOR_SYNCHRO_L] = 0;
	regs[REG_MOTOR_SYNCHRO_H] = 0;
	regs[REG_MOTOR_SYNCHRO_H] = 0;

	// Apply voltage with voltage angle set to zero, wait for rotor position to settle
	setpoint_electrical_angle_rad = 0.0f;
	setpoint_flux_voltage_V = calibration_voltage; // hard-coded V setpoint
	foc_state = FOC_STATE_FLUX_CONTROL;
	HAL_Delay(200);

	// Find the direction of the encoder
	positionSensor_update();
	theta_start = positionSensor_getRadiansMultiturn();
	int n_dir = 500;
	for (int i = 0; i <= n_dir; i++)
	{
		setpoint_electrical_angle_rad = M_2PI * i / n_dir;
		positionSensor_update();
		HAL_Delay(1);
	}
	HAL_Delay(200);
	positionSensor_update();
	theta_end = positionSensor_getRadiansMultiturn();
	for (int i = n_dir; i >= 0; i--)
	{
		setpoint_electrical_angle_rad = M_2PI * i / n_dir;
		positionSensor_update();
		HAL_Delay(1);
	}
	const float delta_angle = theta_end - theta_start;
	regs[REG_INV_PHASE_MOTOR] = delta_angle > 0.0f ? 0 : 1;
	const float reverse = regs[REG_INV_PHASE_MOTOR] == 0 ? 1.0f : -1.0f;
	HAL_Delay(200);
	positionSensor_update();

	// Rotate forward
	HAL_Serial_Print(&serial, "Rotating forward\n\r");
	raw_f_0 = positionSensor_getAngleRaw();
	theta_start = positionSensor_getRadiansMultiturn();
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n2; j++)
		{
			theta_ref += delta;
			setpoint_electrical_angle_rad = theta_ref;
			positionSensor_update();
			HAL_Delay(1);
		}
		positionSensor_update();
		theta_actual = positionSensor_getRadiansMultiturn() - theta_start;
		float error_f = theta_ref / npp - theta_actual;
		error[i] = (int16_t)round(error_f * precison_multiplier);
	}

	// Rotate backwards
	HAL_Serial_Print(&serial, "Rotating backward\n\r");
	for (int i = 0; i < n; i++)
	{
		for (int j = 0; j < n2; j++)
		{
			theta_ref -= delta;
			setpoint_electrical_angle_rad = theta_ref;
			positionSensor_update();
			HAL_Delay(1);
		}
		positionSensor_update();
		theta_actual = positionSensor_getRadiansMultiturn() - theta_start;
		float error_b = theta_ref / npp - theta_actual;
		error[n - i - 1] = (int16_t)round((error_b * precison_multiplier + error[n - i - 1]) / 2);
	}
	raw_b_n1 = positionSensor_getAngleRaw();

	// Release motor
	setpoint_electrical_angle_rad = 0.0f;
	setpoint_flux_voltage_V = 0.0f;
	foc_state = FOC_STATE_IDLE;

	// Compute direction and electrical offset
	float offset = 0;
	for (int i = 0; i < n; i++)
	{
		offset += (error[i] / precison_multiplier - theta_start) / n; // calclate average position sensor offset
	}
	const float phase_synchro_offset_rad = normalize_angle(offset * npp * reverse);
	regs[REG_MOTOR_SYNCHRO_L] = LOW_BYTE((int)RADIANS_TO_DEGREES(phase_synchro_offset_rad));
	regs[REG_MOTOR_SYNCHRO_H] = HIGH_BYTE((int)RADIANS_TO_DEGREES(phase_synchro_offset_rad));

	// Perform filtering to linearize position sensor eccentricity
	float mean = 0;
	for (int i = 0; i < n; i++)
	{
		float error_filt_i = 0;
		for (int j = 0; j < n_lut; j++)
		{
			int ind = -n_lut / 2 + j + i; // indexes from -n_lut/2 to + n_lut/2
			if (ind < 0)
			{
				ind += n;
			}
			else if (ind > n - 1) // moving average wraps around
			{
				ind -= n;
			}
			error_filt_i += error[ind] / (float)n_lut;
		}
		mean += error_filt_i / precison_multiplier / n;
		if (i % npp == 0)
		{
			error_filt[i / npp] = (int16_t)round(error_filt_i);
		}
	}
	int raw_offset = (raw_f_0 + raw_b_n1) / 2;

	// Build lookup table
	for (int i = 0; i < n_lut; i++)
	{
		int ind = (raw_offset >> shift_bits) + i;
		if (ind > (n_lut - 1))
		{
			ind -= n_lut;
		}
		regs_lut[ind] = (int)((error_filt[i] / precison_multiplier - mean) * cpr / M_2PI);
	}

	// Print calibration information
	HAL_Serial_Print(&serial, "Direction: %d\n", (int)(reverse));
	HAL_Serial_Print(&serial, "Synchro: %d\n", (int)(RADIANS_TO_DEGREES(phase_synchro_offset_rad)));
	HAL_Serial_Print(&serial, "Lookup Table:\n");
	for (int i = 0; i < n_lut; i++)
	{
		HAL_Serial_Print(&serial, "%d. %d\n", i, regs_lut[i]);
		HAL_Delay(10);
	}

	// Store calibration into EEPROM
	store_eeprom_regs();

	// Free memory
	free(error);
	free(error_filt);
	return 0; // calibration success
}

void API_FOC_Service_Update()
{
	// check temperature and voltage
	LL_FOC_Update_Temperature();
	LL_FOC_Update_Voltage();
}

// user API function
// this function process an closed-loop FOC from flux and torque current setpoints
// this function allow on-the-go synchronization angle adjustment
// the open loop mode means that the present Id and Iq are forced to 0
//    this may require adjustment of the Kp and Ki of both flux and torque PI regulator
void API_FOC_Torque_Update()
{
	// timestamp
	foc_timestamp_ms = HAL_GetTick();
	foc_timestamp_us = __HAL_TIM_GET_COUNTER(&htim6);

	float Vds = 0.0f;
	float Vqs = 0.0f;
	float cosine_theta = 0.0f;
	float sine_theta = 1.0f;

	// synch with registers
	const float phase_offset_rad = DEGREES_TO_RADIANS((int16_t)(MAKE_SHORT(regs[REG_MOTOR_SYNCHRO_L], regs[REG_MOTOR_SYNCHRO_H])));
	const float phase_synchro_offset_rad = DEGREES_TO_RADIANS((float)(MAKE_SHORT(regs[REG_GOAL_SYNCHRO_OFFSET_L], regs[REG_GOAL_SYNCHRO_OFFSET_H]))); // manual synchro triming
	const float reg_pole_pairs = regs[REG_MOTOR_POLE_PAIRS];
	const float reverse = regs[REG_INV_PHASE_MOTOR] == 0 ? 1.0f : -1.0f;
	const float flux_Kp = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_FLUX_CURRENT_KP_L], regs[REG_PID_FLUX_CURRENT_KP_H]))) / 100000.0f;
	const float flux_Ki = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_FLUX_CURRENT_KI_L], regs[REG_PID_FLUX_CURRENT_KI_H]))) / 100000000.0f;
	const float torque_Kp = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_TORQUE_CURRENT_KP_L], regs[REG_PID_TORQUE_CURRENT_KP_H]))) / 100000.0f;
	const float torque_Ki = (float)((int16_t)(MAKE_SHORT(regs[REG_PID_TORQUE_CURRENT_KI_L], regs[REG_PID_TORQUE_CURRENT_KI_H]))) / 100000000.0f;

	// check control mode
	switch (foc_state)
	{
	case FOC_STATE_IDLE:
	{
		// [Theta]
		theta_rad = normalize_angle(positionSensor_getRadiansEstimation(foc_timestamp_us) * reg_pole_pairs * reverse + phase_offset_rad + phase_synchro_offset_rad);

		// [Cosine]
		API_CORDIC_Processor_Update(theta_rad, &cosine_theta, &sine_theta);

		// [Clarke Transformation]
		const float present_Ialpha = (2.0f * motor_current_mA[0] - motor_current_mA[1] - motor_current_mA[2]) / 3.0f;
		const float present_Ibeta = INV_SQRT3 * (motor_current_mA[1] - motor_current_mA[2]);

		// [Park Transformation]
		present_Ids_mA = present_Ialpha * cosine_theta + present_Ibeta * sine_theta;
		present_Iqs_mA = -present_Ialpha * sine_theta + present_Ibeta * cosine_theta;

		// do brake
		LL_FOC_brake();
	}
	break;
	case FOC_STATE_TORQUE_CONTROL:
	{
		// computation ~7µs (-02)

		// [Theta]
		absolute_position_rad = positionSensor_getRadiansEstimation(foc_timestamp_us);
		theta_rad = normalize_angle(absolute_position_rad * reg_pole_pairs * reverse + phase_offset_rad + phase_synchro_offset_rad);

		// [Cosine]
		API_CORDIC_Processor_Update(theta_rad, &cosine_theta, &sine_theta);

		// [Clarke Transformation]
		const float present_Ialpha = (2.0f * motor_current_mA[0] - motor_current_mA[1] - motor_current_mA[2]) / 3.0f;
		const float present_Ibeta = INV_SQRT3 * (motor_current_mA[1] - motor_current_mA[2]);

		// [Park Transformation]
		present_Ids_mA = (present_Ialpha * cosine_theta + present_Ibeta * sine_theta);
		present_Iqs_mA = (-present_Ialpha * sine_theta + present_Ibeta * cosine_theta);

		// [PI]
		Vds = pi_process_antiwindup_clamp(
			&flux_pi,
			setpoint_flux_current_mA - present_Ids_mA,
			flux_Kp,
			flux_Ki,
			present_voltage_V // output_limit
		);
		Vqs = pi_process_antiwindup_clamp(
			&torque_pi,
			setpoint_torque_current_mA - present_Iqs_mA,
			torque_Kp,
			torque_Ki,
			present_voltage_V // output_limit
		);

		// voltage norm saturation Umax = Udc/sqrt(3)
		const float Vmax = present_voltage_V * INV_SQRT3;
		const float Vnorm = sqrtf(Vds * Vds + Vqs * Vqs);
		if (Vnorm > Vmax)
		{
			const float k = fabsf(Vmax / Vnorm);
			Vqs *= k;
			Vds *= k;
		}

		// do inverse clarke and park transformation and update 3-phase PWM generation
		LL_FOC_set_phase_voltage(Vds, Vqs, cosine_theta, sine_theta, present_voltage_V);
	}
	break;
	case FOC_STATE_FLUX_CONTROL:
	{
		// cannot estimate phase current
		present_Ids_mA = 0.0f;
		present_Iqs_mA = 0.0f;

		// compute theta
		const float theta_rad = normalize_angle(setpoint_electrical_angle_rad * reverse);

		// compute cosine and sine
		API_CORDIC_Processor_Update(theta_rad, &cosine_theta, &sine_theta);

		// compute (Vd,Vq)
		Vds = setpoint_flux_voltage_V; // torque setpoint open loop
		Vqs = 0.0f;					   // no torque

		// do inverse clarke and park transformation and update 3-phase PWM generation
		LL_FOC_set_phase_voltage(Vds, Vqs, cosine_theta, sine_theta, present_voltage_V);
	}
	break;
	}

	// performance monitoring
	uint16_t const t_end = __HAL_TIM_GET_COUNTER(&htim6);
	uint16_t const t_tp = t_end - foc_timestamp_us;
	static const float alpha_performance_monitoring = 0.001f;
	average_processing_time_us = (1.0f - alpha_performance_monitoring) * average_processing_time_us + alpha_performance_monitoring * (float)t_tp;
	++foc_counter;
}

uint32_t API_FOC_Get_Timestamp_ms()
{
	return foc_timestamp_ms;
}

uint16_t API_FOC_Get_Timestamp_us()
{
	return foc_timestamp_us;
}

float API_FOC_Get_Present_Torque_Current()
{
	return present_Iqs_mA;
}

float API_FOC_Get_Present_Flux_Current()
{
	return present_Ids_mA;
}

float API_FOC_Get_Present_Voltage()
{
	return present_voltage_V;
}

float API_FOC_Get_Present_Temp()
{
	return present_temperature_C;
}

float API_FOC_Get_Processing_Time()
{
	return average_processing_time_us;
}

float API_FOC_Get_Processing_Frequency()
{
	return (float)foc_counter / (float)HAL_GetTick() * 1000.0f;
}

void API_FOC_It(ADC_HandleTypeDef *hadc)
{
	if (hadc == &hadc1)
	{
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
		{
			// phase current
			motor_current_input_adc[0] = ADC1_DMA[1];
			motor_current_mA[0] = ((float)motor_current_input_adc[0] - motor_current_input_adc_offset[0]) * motor_current_input_adc_KmA;
			++current_samples;
			// aux
			potentiometer_input_adc = ADC1_DMA[2];
			vbus_input_adc = ADC1_DMA[3];
			temperature_input_adc = ADC1_DMA[4];
		}
		else
		{
			motor_current_input_adc_offset[0] = ALPHA_CURRENT_SENSE_OFFSET * (float)(ADC1_DMA[1]) + (1.0f - ALPHA_CURRENT_SENSE_OFFSET) * motor_current_input_adc_offset[0];
		}
	}
	if (hadc == &hadc2)
	{
		if (__HAL_TIM_IS_TIM_COUNTING_DOWN(&htim1))
		{
			// phase current
			motor_current_input_adc[1] = ADC2_DMA[1];
			motor_current_input_adc[2] = ADC2_DMA[2];
			motor_current_mA[1] = ((float)motor_current_input_adc[1] - motor_current_input_adc_offset[1]) * motor_current_input_adc_KmA;
			motor_current_mA[2] = ((float)motor_current_input_adc[2] - motor_current_input_adc_offset[2]) * motor_current_input_adc_KmA;
			current_samples += 2;
		}
		else
		{
			motor_current_input_adc_offset[1] = ALPHA_CURRENT_SENSE_OFFSET * (float)(ADC2_DMA[1]) + (1.0f - ALPHA_CURRENT_SENSE_OFFSET) * motor_current_input_adc_offset[1];
			motor_current_input_adc_offset[2] = ALPHA_CURRENT_SENSE_OFFSET * (float)(ADC2_DMA[2]) + (1.0f - ALPHA_CURRENT_SENSE_OFFSET) * motor_current_input_adc_offset[2];
		}
	}
	// once the 3 phase current are acquired, call for FOC
	if (current_samples >= 3)
	{
		current_samples = 0;
		API_FOC_Torque_Update();
	}
}
