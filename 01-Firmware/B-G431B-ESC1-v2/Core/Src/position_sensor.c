/*
 * position_sensor.c
 *
 *  Created on: 08.03.2021
 *      Author: kai
 */
#include <stdio.h>
#include <stdlib.h>
#include "position_sensor.h"
#include "as5600.h"
#include "as5048a.h"
#include "math_tool.h"
#include "control_table.h"

#define ALPHA_VELOCITY_TIME_CONST_US 10000.0f // low pass filter for velocity measurement

extern I2C_HandleTypeDef hi2c1;
extern TIM_HandleTypeDef htim4;

// µs TIMER
extern TIM_HandleTypeDef htim6;

typedef struct
{

	e_sensor_type sensor_type;

	// AS5600 handle
	AS5600_TypeDef *as5600Handle;

	// common sensor values
	uint16_t lastUpdate;
	uint16_t last_angle_data;
	int angle;
	int last_angle;
	float velocity_deg;
	float velocity_rad;

	float angle_rad;
	float angle_deg;

	float angle_prev_rad;
	float angle_prev_deg;

	int natural_direction;
	int full_rotation_offset;

} positionSensor_t;

static positionSensor_t *sensor;

positionSensor_t *positionSensor_New(void)
{
	positionSensor_t *sensor = (positionSensor_t *)calloc(1, sizeof(positionSensor_t));
	return sensor;
}

int positionSensor_init(e_sensor_type sensor_type)
{

	int status = 0;

	sensor = positionSensor_New();

	switch (sensor_type)
	{
	case AS5600_I2C:
	{
		const int encoder_bits = regs[REG_ENCODER_BITS];
		const int lut_bits = REG_MAX_LUT_BITS;
		const int shift_bits = encoder_bits - lut_bits;
		uint16_t angle_data;

		sensor->full_rotation_offset = 0;

		sensor->as5600Handle = AS5600_New();
		sensor->as5600Handle->i2cHandle = &hi2c1;
		AS5600_Init(sensor->as5600Handle);

		AS5600_GetAngle(sensor->as5600Handle, &angle_data);
		sensor->last_angle_data = angle_data;

		// Lookup table angle correction
		int off_1 = regs_lut[angle_data >> shift_bits];
		int off_2 = regs_lut[((angle_data >> shift_bits) + 1) % 128];
		int off_interp = off_1 + ((off_2 - off_1) * (angle_data - ((angle_data >> shift_bits) << shift_bits)) >> shift_bits); // Interpolate between lookup table entries
		int angle = angle_data + off_interp;

		sensor->angle = angle;
		sensor->last_angle = angle;

		// velocity calculation init
		sensor->angle_rad = 0;
		sensor->lastUpdate = __HAL_TIM_GET_COUNTER(&htim6);

		sensor->sensor_type = sensor_type;

		status = 1;
		break;
	}
	case AS5048A_PWM:
		API_AS5048A_Position_Sensor_Init(&htim4);
		sensor->sensor_type = sensor_type;
		status = 1;
		break;
	default:
		status = 0;
		break;
	}
	return status;
}

float positionSensor_getRadiansEstimation(uint16_t time_us)
{
	switch (sensor->sensor_type)
	{
	case AS5600_I2C:
	{
		uint16_t delta_t_us;
		if (time_us <= sensor->lastUpdate)
		{
			delta_t_us = 0xffff - sensor->lastUpdate + time_us;
		}
		else
		{
			delta_t_us = (time_us - sensor->lastUpdate);
		}
		return sensor->angle_rad + sensor->velocity_rad * (float)(delta_t_us + 1200.0f) / 1000000.0f; // latency compensation
	}
	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_Radians_Estimation(time_us);
	default:
		return 0;
	}
}

void positionSensor_update(void)
{

	switch (sensor->sensor_type)
	{
	// get new data from as5600 sensor
	case AS5600_I2C:
	{
		const int encoder_bits = regs[REG_ENCODER_BITS];
		const int lut_bits = REG_MAX_LUT_BITS;
		const int shift_bits = encoder_bits - lut_bits;
		const float cpr = pow(2, encoder_bits);
		float delta_time_us;
		float d_angle;
		uint16_t angle_data;

		// raw data from the sensor
		HAL_StatusTypeDef status = AS5600_GetAngle(sensor->as5600Handle, &angle_data);
		if (status != HAL_OK)
		{
			// set encoder error
			regs[REG_HARDWARE_ERROR_STATUS] |= 1UL << HW_ERROR_BIT_POSITION_SENSOR_STATUS_ERROR;
			return;
		}
		else
		{
			// clear encoder error
			regs[REG_HARDWARE_ERROR_STATUS] &= ~(1UL << HW_ERROR_BIT_POSITION_SENSOR_STATUS_ERROR);
		}

		// Lookup table angle correction
		int off_1 = regs_lut[angle_data >> shift_bits];
		int off_2 = regs_lut[((angle_data >> shift_bits) + 1) % 128];
		int off_interp = off_1 + ((off_2 - off_1) * (angle_data - ((angle_data >> shift_bits) << shift_bits)) >> shift_bits); // Interpolate between lookup table entries
		int angle = angle_data + off_interp;

		// tracking the number of rotations
		// in order to expand angle range form [0,2PI]
		// to basically infinity
		d_angle = (float)(angle - sensor->last_angle);
		// if overflow happened track it as full rotation
		int full_rotation = 0;
		if (abs(d_angle) > (0.8 * cpr))
		{
			full_rotation = d_angle > 0 ? -1 : 1;
		}
		sensor->full_rotation_offset += full_rotation;
		// save the current angle value for the next steps
		// in order to know if overflow happened
		sensor->last_angle = angle;
		sensor->last_angle_data = angle_data;

		// calculate sample time
		uint16_t now_us = __HAL_TIM_GET_COUNTER(&htim6);
		if (now_us <= sensor->lastUpdate)
		{
			delta_time_us = 0xffff - sensor->lastUpdate + now_us;
		}
		else
		{
			delta_time_us = (now_us - sensor->lastUpdate);
		}

		// angle and velocity calculation
		float angle_rad = ((float)angle / cpr) * M_2PI;
		float angle_deg = RADIANS_TO_DEGREES(angle_rad);
		float alpha_velocity_sense = (float)delta_time_us / (ALPHA_VELOCITY_TIME_CONST_US + (float)delta_time_us);
		float velocity_rad = alpha_velocity_sense * ((angle_rad + full_rotation * M_2PI - sensor->angle_prev_rad) / delta_time_us * 1000000.0f) + (1.0f - alpha_velocity_sense) * sensor->velocity_rad;
		float velocity_deg = alpha_velocity_sense * ((angle_deg + full_rotation * 360.0f - sensor->angle_prev_deg) / delta_time_us * 1000000.0f) + (1.0f - alpha_velocity_sense) * sensor->velocity_deg;

		// update global variables
		sensor->lastUpdate = now_us;
		sensor->angle_rad = angle_rad;
		sensor->angle_deg = angle_deg;
		sensor->velocity_rad = velocity_rad;
		// sensor->velocity_deg = velocity_deg;
		sensor->velocity_deg = (angle_deg + full_rotation * 360.0f - sensor->angle_prev_deg) / delta_time_us * 1000000.0f;

		// last angle
		sensor->angle_prev_rad = sensor->angle_rad;
		sensor->angle_prev_deg = sensor->angle_deg;

		// encoder noise detection
		int sign_raw = (velocity_deg > 0) - (velocity_deg < 0);
		int sign_filtered = (sensor->velocity_deg > 0) - (sensor->velocity_deg < 0);
		// if the sign of the velocity is different, then the encoder is noisy
		if (sign_raw != sign_filtered)
		{
			regs[REG_DEBUG]++;
		}

		break;
	}
	default:
		break;
	}
}

float positionSensor_getRadians(void)
{
	switch (sensor->sensor_type)
	{
	case AS5600_I2C:
		return sensor->angle_rad;

	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_Radians();

	default:
		return 0;
	}
}

float positionSensor_getRadiansMultiturn(void)
{
	switch (sensor->sensor_type)
	{
	case AS5600_I2C:
		return sensor->angle_rad + sensor->full_rotation_offset * M_2PI;

	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_Multiturn_Radians();

	default:
		return 0;
	}
}

float positionSensor_getDegree(void)
{
	switch (sensor->sensor_type)
	{
	case AS5600_I2C:
		return sensor->angle_deg;

	case AS5048A_PWM:
		return RADIANS_TO_DEGREES(API_AS5048A_Position_Sensor_Get_Radians());

	default:
		return 0;
	}
}

float positionSensor_getDegreeMultiturn(void)
{
	switch (sensor->sensor_type)
	{
	case AS5600_I2C:
		return sensor->angle_deg + sensor->full_rotation_offset * 360.0f;

	case AS5048A_PWM:
		return RADIANS_TO_DEGREES(API_AS5048A_Position_Sensor_Get_Multiturn_Radians());

	default:
		return 0;
	}
}

float positionSensor_getVelocityDegree(void)
{
	switch (sensor->sensor_type)
	{
	case AS5600_I2C:
		return sensor->velocity_deg;

	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_DPS();

	default:
		return 0;
	}
}

e_sensor_type positionSensor_getType(void)
{
	return sensor->sensor_type;
}

uint16_t positionSensor_getDeltaTimestamp()
{
	switch (sensor->sensor_type)
	{
	case AS5600_I2C:
		return 0;

	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_DeltaTimestamp();

	default:
		return 0;
	}
}

int16_t positionSensor_getDeltaTimeEstimation()
{
	switch (sensor->sensor_type)
	{
	case AS5600_I2C:
		return 0;

	case AS5048A_PWM:
		return API_AS5048A_Position_Sensor_Get_DeltaTimeEstimation();

	default:
		return 0;
	}
}

int positionSensor_getAngleRaw()
{
	switch (sensor->sensor_type)
	{
	case AS5600_I2C:
		return sensor->last_angle_data;

	case AS5048A_PWM:
		return 0;

	default:
		return 0;
	}
}