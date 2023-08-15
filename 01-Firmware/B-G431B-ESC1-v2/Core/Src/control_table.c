/*
 * control_table.c
 *
 *  Created on: 27 sept. 2020
 *      Author: patrick
 */

#include "control_table.h"
#include "binary_tool.h"
#include "eeprom.h"

#include "stm32g4xx_hal.h"
#include <string.h>
#include "math_tool.h"

uint8_t regs[REG_MAX];
uint16_t regs_lut[REG_MAX_LUT];

void factory_reset_eeprom_regs()
{
	regs[REG_MODEL_NUMBER_L] = LOW_BYTE(REG_MODEL_NUMBER_VALUE);
	regs[REG_MODEL_NUMBER_H] = HIGH_BYTE(REG_MODEL_NUMBER_VALUE);
	regs[REG_VERSION] = REG_VERSION_VALUE;
	regs[REG_ID] = REG_ID_VALUE;
	regs[REG_BAUD_RATE] = REG_BAUD_RATE_VALUE;
	regs[REG_RETURN_DELAY] = REG_RETURN_DELAY_VALUE;

	regs[REG_MIN_POSITION_DEG_L] = LOW_BYTE(REG_MIN_POSITION_DEG_VALUE);
	regs[REG_MIN_POSITION_DEG_H] = HIGH_BYTE(REG_MIN_POSITION_DEG_VALUE);
	regs[REG_MAX_POSITION_DEG_L] = LOW_BYTE(REG_MAX_POSITION_DEG_VALUE);
	regs[REG_MAX_POSITION_DEG_H] = HIGH_BYTE(REG_MAX_POSITION_DEG_VALUE);
	regs[REG_MAX_VELOCITY_DPS_L] = LOW_BYTE(REG_MAX_VELOCITY_DPS_VALUE);
	regs[REG_MAX_VELOCITY_DPS_H] = HIGH_BYTE(REG_MAX_VELOCITY_DPS_VALUE);
	regs[REG_MAX_ACCELERATION_DPSS_L] = LOW_BYTE(REG_MAX_ACCELERATION_DPSS_VALUE);
	regs[REG_MAX_ACCELERATION_DPSS_H] = HIGH_BYTE(REG_MAX_ACCELERATION_DPSS_VALUE);
	regs[REG_MAX_CURRENT_MA_L] = LOW_BYTE(REG_MAX_CURRENT_MA_VALUE);
	regs[REG_MAX_CURRENT_MA_H] = HIGH_BYTE(REG_MAX_CURRENT_MA_VALUE);
	regs[REG_TEMPERATURE_LIMIT] = REG_TEMPERATURE_LIMIT_VALUE;
	regs[REG_LOW_VOLTAGE_LIMIT] = REG_LOW_VOLTAGE_LIMIT_VALUE;
	regs[REG_HIGH_VOLTAGE_LIMIT] = REG_HIGH_VOLTAGE_LIMIT_VALUE;

	regs[REG_MOVING_THRESHOLD_DPS] = REG_MOVING_THRESHOLD_DPS_VALUE;
	regs[REG_STATUS_RETURN_LVL] = REG_STATUS_RETURN_LVL_VALUE;
	regs[REG_ALARM_LED] = REG_ALARM_LED_VALUE;
	regs[REG_ALARM_SHUTDOWN] = REG_ALARM_SHUTDOWN_VALUE;

	regs[REG_ENCODER_BITS] = REG_ENCODER_BITS_VALUE;
	regs[REG_MOTOR_POLE_PAIRS] = REG_MOTOR_POLE_PAIRS_VALUE;
	regs[REG_MOTOR_SYNCHRO_L] = LOW_BYTE(REG_MOTOR_SYNCHRO_VALUE);
	regs[REG_MOTOR_SYNCHRO_H] = HIGH_BYTE(REG_MOTOR_SYNCHRO_VALUE);
	regs[REG_INV_PHASE_MOTOR] = REG_INV_PHASE_VALUE;
	regs[REG_FIELD_WEAKENING_K] = REG_FIELD_WEAKENING_K_VALUE;

	regs[REG_PID_POSITION_KP_L] = LOW_BYTE(REG_PID_POSITION_KP_VALUE);
	regs[REG_PID_POSITION_KP_H] = HIGH_BYTE(REG_PID_POSITION_KP_VALUE);
	regs[REG_PID_POSITION_KI_L] = LOW_BYTE(REG_PID_POSITION_KI_VALUE);
	regs[REG_PID_POSITION_KI_H] = HIGH_BYTE(REG_PID_POSITION_KI_VALUE);
	regs[REG_PID_POSITION_KD_L] = LOW_BYTE(REG_PID_POSITION_KD_VALUE);
	regs[REG_PID_POSITION_KD_H] = HIGH_BYTE(REG_PID_POSITION_KD_VALUE);

	regs[REG_PID_VELOCITY_KP_L] = LOW_BYTE(REG_PID_VELOCITY_KP_VALUE);
	regs[REG_PID_VELOCITY_KP_H] = HIGH_BYTE(REG_PID_VELOCITY_KP_VALUE);
	regs[REG_PID_VELOCITY_KI_L] = LOW_BYTE(REG_PID_VELOCITY_KI_VALUE);
	regs[REG_PID_VELOCITY_KI_H] = HIGH_BYTE(REG_PID_VELOCITY_KI_VALUE);
	regs[REG_PID_VELOCITY_KD_L] = LOW_BYTE(REG_PID_VELOCITY_KD_VALUE);
	regs[REG_PID_VELOCITY_KD_H] = HIGH_BYTE(REG_PID_VELOCITY_KD_VALUE);
	regs[REG_PID_VELOCITY_KFF_L] = LOW_BYTE(REG_PID_VELOCITY_KFF_VALUE);
	regs[REG_PID_VELOCITY_KFF_H] = HIGH_BYTE(REG_PID_VELOCITY_KFF_VALUE);
	regs[REG_PID_ACCELERATION_KFF_L] = LOW_BYTE(REG_PID_ACCELERATION_KFF_VALUE);
	regs[REG_PID_ACCELERATION_KFF_H] = HIGH_BYTE(REG_PID_ACCELERATION_KFF_VALUE);

	regs[REG_PID_FLUX_CURRENT_KP_L] = LOW_BYTE(REG_PID_FLUX_CURRENT_KP_VALUE);
	regs[REG_PID_FLUX_CURRENT_KP_H] = HIGH_BYTE(REG_PID_FLUX_CURRENT_KP_VALUE);
	regs[REG_PID_FLUX_CURRENT_KI_L] = LOW_BYTE(REG_PID_FLUX_CURRENT_KI_VALUE);
	regs[REG_PID_FLUX_CURRENT_KI_H] = HIGH_BYTE(REG_PID_FLUX_CURRENT_KI_VALUE);
	regs[REG_PID_FLUX_CURRENT_KFF_L] = LOW_BYTE(REG_PID_FLUX_CURRENT_KFF_VALUE);
	regs[REG_PID_FLUX_CURRENT_KFF_H] = HIGH_BYTE(REG_PID_FLUX_CURRENT_KFF_VALUE);

	regs[REG_PID_TORQUE_CURRENT_KP_L] = LOW_BYTE(REG_PID_TORQUE_CURRENT_KP_VALUE);
	regs[REG_PID_TORQUE_CURRENT_KP_H] = HIGH_BYTE(REG_PID_TORQUE_CURRENT_KP_VALUE);
	regs[REG_PID_TORQUE_CURRENT_KI_L] = LOW_BYTE(REG_PID_TORQUE_CURRENT_KI_VALUE);
	regs[REG_PID_TORQUE_CURRENT_KI_H] = HIGH_BYTE(REG_PID_TORQUE_CURRENT_KI_VALUE);
	regs[REG_PID_TORQUE_CURRENT_KFF_L] = LOW_BYTE(REG_PID_TORQUE_CURRENT_KFF_VALUE);
	regs[REG_PID_TORQUE_CURRENT_KFF_H] = HIGH_BYTE(REG_PID_TORQUE_CURRENT_KFF_VALUE);

	regs[REG_CAL_PHASE1_CURRENT_SENSE_MA_L] = LOW_BYTE(REG_CAL_PHASE1_CURRENT_SENSE_MA_VALUE);
	regs[REG_CAL_PHASE1_CURRENT_SENSE_MA_H] = HIGH_BYTE(REG_CAL_PHASE1_CURRENT_SENSE_MA_VALUE);
	regs[REG_CAL_PHASE1_CURRENT_SENSE_OFFSET_L] = LOW_BYTE(REG_CAL_PHASE1_CURRENT_SENSE_OFFSET_VALUE);
	regs[REG_CAL_PHASE1_CURRENT_SENSE_OFFSET_H] = HIGH_BYTE(REG_CAL_PHASE1_CURRENT_SENSE_OFFSET_VALUE);

	regs[REG_CAL_PHASE2_CURRENT_SENSE_MA_L] = LOW_BYTE(REG_CAL_PHASE2_CURRENT_SENSE_MA_VALUE);
	regs[REG_CAL_PHASE2_CURRENT_SENSE_MA_H] = HIGH_BYTE(REG_CAL_PHASE2_CURRENT_SENSE_MA_VALUE);
	regs[REG_CAL_PHASE2_CURRENT_SENSE_OFFSET_L] = LOW_BYTE(REG_CAL_PHASE2_CURRENT_SENSE_OFFSET_VALUE);
	regs[REG_CAL_PHASE2_CURRENT_SENSE_OFFSET_H] = HIGH_BYTE(REG_CAL_PHASE2_CURRENT_SENSE_OFFSET_VALUE);

	regs[REG_CAL_PHASE3_CURRENT_SENSE_MA_L] = LOW_BYTE(REG_CAL_PHASE3_CURRENT_SENSE_MA_VALUE);
	regs[REG_CAL_PHASE3_CURRENT_SENSE_MA_H] = HIGH_BYTE(REG_CAL_PHASE3_CURRENT_SENSE_MA_VALUE);
	regs[REG_CAL_PHASE3_CURRENT_SENSE_OFFSET_L] = LOW_BYTE(REG_CAL_PHASE3_CURRENT_SENSE_OFFSET_VALUE);
	regs[REG_CAL_PHASE3_CURRENT_SENSE_OFFSET_H] = HIGH_BYTE(REG_CAL_PHASE3_CURRENT_SENSE_OFFSET_VALUE);

	regs[REG_CAL_VOLTAGE_SENSOR_L] = LOW_BYTE(REG_CAL_VOLTAGE_SENSOR_VALUE);
	regs[REG_CAL_VOLTAGE_SENSOR_H] = HIGH_BYTE(REG_CAL_VOLTAGE_SENSOR_VALUE);

	regs[REG_EWMA_ENCODER] = REG_EWMA_ENCODER_VALUE;

	// Restore LUT to default value of a linear array from 0 to 2*M_PI*regs[REG_MOTOR_POLE_PAIRS],
	for (int i = 0; i < REG_MAX_LUT; i++)
	{
		float electrical_angle = M_2PI * regs[REG_MOTOR_POLE_PAIRS] * i / REG_MAX_LUT;
		regs_lut[i] = (uint16_t)(electrical_angle / (M_2PI * regs[REG_MOTOR_POLE_PAIRS]) * UINT16_MAX);
	}

	eeprom_store(regs, regs_lut, REG_TORQUE_ENABLE, REG_MAX_LUT); // REG_TORQUE_ENABLE must be 64 bits aligned
}

void load_eeprom_regs()
{
	eeprom_restore(regs, regs_lut, REG_TORQUE_ENABLE, REG_MAX_LUT); // REG_TORQUE_ENABLE must be 64 bits aligned
}

void store_eeprom_regs()
{
	eeprom_store(regs, regs_lut, REG_TORQUE_ENABLE, REG_MAX_LUT); // REG_TORQUE_ENABLE must be 64 bits aligned
}

void reset_ram_regs()
{
	memset(&regs[REG_TORQUE_ENABLE], 0, REG_MAX - REG_TORQUE_ENABLE);
	memset(regs_lut, 0, REG_MAX_LUT * sizeof(uint16_t));
}
