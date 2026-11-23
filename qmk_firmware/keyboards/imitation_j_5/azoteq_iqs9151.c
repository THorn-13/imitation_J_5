/*
Copyright 2026 sekigon (@sekigon-gonnoc)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.
*/

#ifdef AUTO_KDK_TRACKPAD_IQS9151_ENABLED

#include "azoteq_iqs9151.h"

#include <string.h>

#include "debug.h"
#include "gpio.h"
#include "i2c_master.h"
#include "IQS9151_init.h"
#include "timer.h"
#include "trackpad_gesture_parser.h"
#include "wait.h"

#ifndef AZOTEQ_IQS9151_TX_CHANNEL_COUNT
#    error "AZOTEQ_IQS9151_TX_CHANNEL_COUNT must be defined in config.h"
#endif

#ifndef AZOTEQ_IQS9151_RX_CHANNEL_COUNT
#    error "AZOTEQ_IQS9151_RX_CHANNEL_COUNT must be defined in config.h"
#endif

#ifndef AZOTEQ_IQS9151_DEBUG_LOG
#    define AZOTEQ_IQS9151_DEBUG_LOG 0
#endif

#ifndef AZOTEQ_IQS9151_DEBUG_INTERVAL_MS
#    define AZOTEQ_IQS9151_DEBUG_INTERVAL_MS 200
#endif

#ifndef AZOTEQ_IQS9151_DISABLE_HID_REPORT
#    define AZOTEQ_IQS9151_DISABLE_HID_REPORT 0
#endif

#ifndef AZOTEQ_IQS9151_ATI_TIMEOUT_MS
#    define AZOTEQ_IQS9151_ATI_TIMEOUT_MS 3000
#endif

#ifndef AZOTEQ_IQS9151_ATI_IGNORE_RDY
#    define AZOTEQ_IQS9151_ATI_IGNORE_RDY 0
#endif

#ifndef AZOTEQ_IQS9151_ATI_ALLOW_TIMEOUT
#    define AZOTEQ_IQS9151_ATI_ALLOW_TIMEOUT 0
#endif

#ifndef AZOTEQ_IQS9151_ATI_RDY_STUCK_MS
#    define AZOTEQ_IQS9151_ATI_RDY_STUCK_MS 1000
#endif

#ifndef AZOTEQ_IQS9151_INIT_STARTUP_DELAY_MS
#    define AZOTEQ_IQS9151_INIT_STARTUP_DELAY_MS 1000
#endif

#ifndef AZOTEQ_IQS9151_ATI_AUTO_TUNE_START_DIV
#    define AZOTEQ_IQS9151_ATI_AUTO_TUNE_START_DIV 9
#endif

#ifndef AZOTEQ_IQS9151_ATI_AUTO_TUNE_MIN_DIV
#    define AZOTEQ_IQS9151_ATI_AUTO_TUNE_MIN_DIV 6
#endif

#ifndef AZOTEQ_IQS9151_ATI_AUTO_TUNE_MIN_COUNT
#    define AZOTEQ_IQS9151_ATI_AUTO_TUNE_MIN_COUNT 600
#endif

#define IQS9151_ATI_RESULT_MASK 0x03FFu

#if (AZOTEQ_IQS9151_TX_CHANNEL_COUNT < 1) || (AZOTEQ_IQS9151_TX_CHANNEL_COUNT > 10)
#    error "AZOTEQ_IQS9151_TX_CHANNEL_COUNT must be between 1 and 10"
#endif

#if (AZOTEQ_IQS9151_RX_CHANNEL_COUNT < 1) || (AZOTEQ_IQS9151_RX_CHANNEL_COUNT > 11)
#    error "AZOTEQ_IQS9151_RX_CHANNEL_COUNT must be between 1 and 11"
#endif

const pointing_device_driver_t azoteq_iqs9151_pointing_device_driver = {
	.init       = azoteq_iqs9151_init,
	.get_report = azoteq_iqs9151_get_report,
	.set_cpi    = azoteq_iqs9151_set_cpi,
	.get_cpi    = azoteq_iqs9151_get_cpi,
};

static i2c_status_t               azoteq_iqs9151_init_status = I2C_STATUS_ERROR;
static bool                       azoteq_iqs9151_use_ready_pin;
static trackpad_gesture_parser_t  azoteq_iqs9151_parser;
static pin_t                      azoteq_iqs9151_ready_pin = NO_PIN;
static uint32_t                   azoteq_iqs9151_dbg_last_frame_ms;
static uint32_t                   azoteq_iqs9151_dbg_last_error_ms;
static bool                       azoteq_iqs9151_prev_show_reset;

static bool azoteq_iqs9151_dbg_due(uint32_t *last_ms, uint16_t interval_ms) {
	uint32_t now_ms = timer_read32();
	if (*last_ms == 0 || timer_elapsed32(*last_ms) >= interval_ms) {
		*last_ms = now_ms;
		return true;
	}
	return false;
}

static const uint8_t iqs9151_alp_compensation[] = {
	ALP_COMPENSATION_RX0_0, ALP_COMPENSATION_RX0_1, ALP_COMPENSATION_RX1_0, ALP_COMPENSATION_RX1_1,
	ALP_COMPENSATION_RX2_0, ALP_COMPENSATION_RX2_1, ALP_COMPENSATION_RX3_0, ALP_COMPENSATION_RX3_1,
	ALP_COMPENSATION_RX4_0, ALP_COMPENSATION_RX4_1, ALP_COMPENSATION_RX5_0, ALP_COMPENSATION_RX5_1,
	ALP_COMPENSATION_RX6_0, ALP_COMPENSATION_RX6_1, ALP_COMPENSATION_RX7_0, ALP_COMPENSATION_RX7_1,
	ALP_COMPENSATION_RX8_0, ALP_COMPENSATION_RX8_1, ALP_COMPENSATION_RX9_0, ALP_COMPENSATION_RX9_1,
	ALP_COMPENSATION_RX10_0, ALP_COMPENSATION_RX10_1, ALP_COMPENSATION_RX11_0, ALP_COMPENSATION_RX11_1,
	ALP_COMPENSATION_RX12_0, ALP_COMPENSATION_RX12_1,
};

static const uint8_t iqs9151_main_config[] = {
	MINOR_VERSION, MAJOR_VERSION,
	TP_ATI_MULTDIV_L, TP_ATI_MULTDIV_H,
	ALP_ATI_COARSE_RX0_L, ALP_ATI_COARSE_RX0_H, ALP_ATI_COARSE_RX1_L, ALP_ATI_COARSE_RX1_H,
	ALP_ATI_COARSE_RX2_L, ALP_ATI_COARSE_RX2_H, ALP_ATI_COARSE_RX3_L, ALP_ATI_COARSE_RX3_H,
	ALP_ATI_COARSE_RX4_L, ALP_ATI_COARSE_RX4_H, ALP_ATI_COARSE_RX5_L, ALP_ATI_COARSE_RX5_H,
	ALP_ATI_COARSE_RX6_L, ALP_ATI_COARSE_RX6_H, ALP_ATI_COARSE_RX7_L, ALP_ATI_COARSE_RX7_H,
	ALP_ATI_COARSE_RX8_L, ALP_ATI_COARSE_RX8_H, ALP_ATI_COARSE_RX9_L, ALP_ATI_COARSE_RX9_H,
	ALP_ATI_COARSE_RX10_L, ALP_ATI_COARSE_RX10_H, ALP_ATI_COARSE_RX11_L, ALP_ATI_COARSE_RX11_H,
	ALP_ATI_COARSE_RX12_L, ALP_ATI_COARSE_RX12_H,
	TP_ATI_TARGET_0, TP_ATI_TARGET_1,
	ALP_ATI_TARGET_0, ALP_ATI_TARGET_1,
	ALP_BASE_TARGET_0, ALP_BASE_TARGET_1,
	TP_NEG_DELTA_REATI_0, TP_NEG_DELTA_REATI_1, TP_POS_DELTA_REATI_0, TP_POS_DELTA_REATI_1,
	TP_REF_DRIFT_LIMIT, ALP_LTA_DRIFT_LIMIT,
	ACTIVE_MODE_SAMPLING_PERIOD_0, ACTIVE_MODE_SAMPLING_PERIOD_1,
	IDLE_TOUCH_MODE_SAMPLING_PERIOD_0, IDLE_TOUCH_MODE_SAMPLING_PERIOD_1,
	IDLE_MODE_SAMPLING_PERIOD_0, IDLE_MODE_SAMPLING_PERIOD_1,
	LP1_MODE_SAMPLING_PERIOD_0, LP1_MODE_SAMPLING_PERIOD_1,
	LP2_MODE_SAMPLING_PERIOD_0, LP2_MODE_SAMPLING_PERIOD_1,
	STATIONARY_TOUCH_TIMEOUT_0, STATIONARY_TOUCH_TIMEOUT_1,
	IDLE_TOUCH_MODE_TIMEOUT_0, IDLE_TOUCH_MODE_TIMEOUT_1,
	IDLE_MODE_TIMEOUT_0, IDLE_MODE_TIMEOUT_1,
	LP1_MODE_TIMEOUT_0, LP1_MODE_TIMEOUT_1,
	ACTIVE_MODE_TIMEOUT_0, ACTIVE_MODE_TIMEOUT_1,
	REATI_RETRY_TIME, REF_UPDATE_TIME,
	I2C_TIMEOUT_0, I2C_TIMEOUT_1,
	SNAP_TIMEOUT, OPEN_TIMING,
	SYSTEM_CONTROL_0, SYSTEM_CONTROL_1,
	CONFIG_SETTINGS_0, CONFIG_SETTINGS_1,
	OTHER_SETTINGS_0, OTHER_SETTINGS_1,
	ALP_SETUP_0, ALP_SETUP_1, ALP_SETUP_2, ALP_SETUP_3,
	ALP_TX_ENABLE_0, ALP_TX_ENABLE_1, ALP_TX_ENABLE_2, ALP_TX_ENABLE_3, ALP_TX_ENABLE_4, ALP_TX_ENABLE_5,
	TRACKPAD_TOUCH_SET_THRESHOLD, TRACKPAD_TOUCH_CLEAR_THRESHOLD,
	ALP_THRESHOLD, ALP_AUTOPROX_THRESHOLD,
	ALP_SET_DEBOUNCE, ALP_CLEAR_DEBOUNCE,
	SNAP_SET_THRESHOLD, SNAP_CLEAR_THRESHOLD,
	ALP_COUNT_BETA_LP1, ALP_LTA_BETA_LP1,
	ALP_COUNT_BETA_LP2, ALP_LTA_BETA_LP2,
	TP_FRAC, TP_PERIOD1, TP_PERIOD2,
	ALP_FRAC, ALP_PERIOD1, ALP_PERIOD2,
	TRACKPAD_HARDWARE_SETTINGS_0, TRACKPAD_HARDWARE_SETTINGS_1,
	ALP_HARDWARE_SETTINGS_0, ALP_HARDWARE_SETTINGS_1,
	TRACKPAD_SETTINGS_0_0, TRACKPAD_SETTINGS_0_1,
	TRACKPAD_SETTINGS_1_0, TRACKPAD_SETTINGS_1_1,
	X_RESOLUTION_0, X_RESOLUTION_1,
	Y_RESOLUTION_0, Y_RESOLUTION_1,
	XY_DYNAMIC_FILTER_BOTTOM_SPEED_0, XY_DYNAMIC_FILTER_BOTTOM_SPEED_1,
	XY_DYNAMIC_FILTER_TOP_SPEED_0, XY_DYNAMIC_FILTER_TOP_SPEED_1,
	XY_DYNAMIC_FILTER_BOTTOM_BETA,
	XY_DYNAMIC_FILTER_STATIC_FILTER_BETA,
	STATIONARY_TOUCH_MOV_THRESHOLD,
	FINGER_SPLIT_FACTOR,
	X_TRIM_VALUE,
	Y_TRIM_VALUE,
	JITTER_FILTER_DELTA,
	FINGER_CONFIDENCE_THRESHOLD,
};

static const uint8_t iqs9151_channel_disable[] = {
	TPCHANNELDISABLE_0, TPCHANNELDISABLE_1, TPCHANNELDISABLE_2, TPCHANNELDISABLE_3,
	TPCHANNELDISABLE_4, TPCHANNELDISABLE_5, TPCHANNELDISABLE_6, TPCHANNELDISABLE_7,
	TPCHANNELDISABLE_8, TPCHANNELDISABLE_9, TPCHANNELDISABLE_10, TPCHANNELDISABLE_11,
	TPCHANNELDISABLE_12, TPCHANNELDISABLE_13, TPCHANNELDISABLE_14, TPCHANNELDISABLE_15,
	TPCHANNELDISABLE_16, TPCHANNELDISABLE_17, TPCHANNELDISABLE_18, TPCHANNELDISABLE_19,
	TPCHANNELDISABLE_20, TPCHANNELDISABLE_21, TPCHANNELDISABLE_22, TPCHANNELDISABLE_23,
	TPCHANNELDISABLE_24, TPCHANNELDISABLE_25, TPCHANNELDISABLE_26, TPCHANNELDISABLE_27,
	TPCHANNELDISABLE_28, TPCHANNELDISABLE_29, TPCHANNELDISABLE_30, TPCHANNELDISABLE_31,
	TPCHANNELDISABLE_32, TPCHANNELDISABLE_33, TPCHANNELDISABLE_34, TPCHANNELDISABLE_35,
	TPCHANNELDISABLE_36, TPCHANNELDISABLE_37, TPCHANNELDISABLE_38, TPCHANNELDISABLE_39,
	TPCHANNELDISABLE_40, TPCHANNELDISABLE_41, TPCHANNELDISABLE_42, TPCHANNELDISABLE_43,
	TPCHANNELDISABLE_44, TPCHANNELDISABLE_45, TPCHANNELDISABLE_46, TPCHANNELDISABLE_47,
	TPCHANNELDISABLE_48, TPCHANNELDISABLE_49, TPCHANNELDISABLE_50, TPCHANNELDISABLE_51,
	TPCHANNELDISABLE_52, TPCHANNELDISABLE_53, TPCHANNELDISABLE_54, TPCHANNELDISABLE_55,
	TPCHANNELDISABLE_56, TPCHANNELDISABLE_57, TPCHANNELDISABLE_58, TPCHANNELDISABLE_59,
	TPCHANNELDISABLE_60, TPCHANNELDISABLE_61, TPCHANNELDISABLE_62, TPCHANNELDISABLE_63,
	TPCHANNELDISABLE_64, TPCHANNELDISABLE_65, TPCHANNELDISABLE_66, TPCHANNELDISABLE_67,
	TPCHANNELDISABLE_68, TPCHANNELDISABLE_69, TPCHANNELDISABLE_70, TPCHANNELDISABLE_71,
	TPCHANNELDISABLE_72, TPCHANNELDISABLE_73, TPCHANNELDISABLE_74, TPCHANNELDISABLE_75,
	TPCHANNELDISABLE_76, TPCHANNELDISABLE_77, TPCHANNELDISABLE_78, TPCHANNELDISABLE_79,
	TPCHANNELDISABLE_80, TPCHANNELDISABLE_81, TPCHANNELDISABLE_82, TPCHANNELDISABLE_83,
	TPCHANNELDISABLE_84, TPCHANNELDISABLE_85, TPCHANNELDISABLE_86, TPCHANNELDISABLE_87,
};

static const uint8_t iqs9151_snap_enable[] = {
	SNAPCHANNELENABLE_0, SNAPCHANNELENABLE_1, SNAPCHANNELENABLE_2, SNAPCHANNELENABLE_3,
	SNAPCHANNELENABLE_4, SNAPCHANNELENABLE_5, SNAPCHANNELENABLE_6, SNAPCHANNELENABLE_7,
	SNAPCHANNELENABLE_8, SNAPCHANNELENABLE_9, SNAPCHANNELENABLE_10, SNAPCHANNELENABLE_11,
	SNAPCHANNELENABLE_12, SNAPCHANNELENABLE_13, SNAPCHANNELENABLE_14, SNAPCHANNELENABLE_15,
	SNAPCHANNELENABLE_16, SNAPCHANNELENABLE_17, SNAPCHANNELENABLE_18, SNAPCHANNELENABLE_19,
	SNAPCHANNELENABLE_20, SNAPCHANNELENABLE_21, SNAPCHANNELENABLE_22, SNAPCHANNELENABLE_23,
	SNAPCHANNELENABLE_24, SNAPCHANNELENABLE_25, SNAPCHANNELENABLE_26, SNAPCHANNELENABLE_27,
	SNAPCHANNELENABLE_28, SNAPCHANNELENABLE_29, SNAPCHANNELENABLE_30, SNAPCHANNELENABLE_31,
	SNAPCHANNELENABLE_32, SNAPCHANNELENABLE_33, SNAPCHANNELENABLE_34, SNAPCHANNELENABLE_35,
	SNAPCHANNELENABLE_36, SNAPCHANNELENABLE_37, SNAPCHANNELENABLE_38, SNAPCHANNELENABLE_39,
	SNAPCHANNELENABLE_40, SNAPCHANNELENABLE_41, SNAPCHANNELENABLE_42, SNAPCHANNELENABLE_43,
	SNAPCHANNELENABLE_44, SNAPCHANNELENABLE_45, SNAPCHANNELENABLE_46, SNAPCHANNELENABLE_47,
	SNAPCHANNELENABLE_48, SNAPCHANNELENABLE_49, SNAPCHANNELENABLE_50, SNAPCHANNELENABLE_51,
	SNAPCHANNELENABLE_52, SNAPCHANNELENABLE_53, SNAPCHANNELENABLE_54, SNAPCHANNELENABLE_55,
	SNAPCHANNELENABLE_56, SNAPCHANNELENABLE_57, SNAPCHANNELENABLE_58, SNAPCHANNELENABLE_59,
	SNAPCHANNELENABLE_60, SNAPCHANNELENABLE_61, SNAPCHANNELENABLE_62, SNAPCHANNELENABLE_63,
	SNAPCHANNELENABLE_64, SNAPCHANNELENABLE_65, SNAPCHANNELENABLE_66, SNAPCHANNELENABLE_67,
	SNAPCHANNELENABLE_68, SNAPCHANNELENABLE_69, SNAPCHANNELENABLE_70, SNAPCHANNELENABLE_71,
	SNAPCHANNELENABLE_72, SNAPCHANNELENABLE_73, SNAPCHANNELENABLE_74, SNAPCHANNELENABLE_75,
	SNAPCHANNELENABLE_76, SNAPCHANNELENABLE_77, SNAPCHANNELENABLE_78, SNAPCHANNELENABLE_79,
	SNAPCHANNELENABLE_80, SNAPCHANNELENABLE_81, SNAPCHANNELENABLE_82, SNAPCHANNELENABLE_83,
	SNAPCHANNELENABLE_84, SNAPCHANNELENABLE_85, SNAPCHANNELENABLE_86, SNAPCHANNELENABLE_87,
};

static uint16_t iqs9151_le_u16(const uint8_t *buf) {
	return (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
}

static void iqs9151_put_le_u16(uint16_t v, uint8_t *buf) {
	buf[0] = (uint8_t)(v & 0xFF);
	buf[1] = (uint8_t)(v >> 8);
}

static uint16_t iqs9151_swap_u16(uint16_t v) {
	return (uint16_t)((v << 8) | (v >> 8));
}

static i2c_status_t iqs9151_i2c_write(uint16_t reg, const uint8_t *buf, size_t len) {
	if (len > IQS9151_I2C_CHUNK_SIZE) {
		return I2C_STATUS_ERROR;
	}

	/* i2c_write_register16 uses big-endian register order; IQS9151 expects little-endian. */
	return i2c_write_register16(AZOTEQ_IQS9151_ADDRESS, iqs9151_swap_u16(reg), buf, (uint16_t)len, AZOTEQ_IQS9151_TIMEOUT_MS);
}

static i2c_status_t iqs9151_i2c_read(uint16_t reg, uint8_t *buf, size_t len) {
	/* i2c_read_register16 uses big-endian register order; IQS9151 expects little-endian. */
	return i2c_read_register16(AZOTEQ_IQS9151_ADDRESS, iqs9151_swap_u16(reg), buf, (uint16_t)len, AZOTEQ_IQS9151_TIMEOUT_MS);
}

static i2c_status_t iqs9151_read_u16(uint16_t reg, uint16_t *value) {
	uint8_t tmp[2];
	i2c_status_t status = iqs9151_i2c_read(reg, tmp, sizeof(tmp));
	if (status != I2C_STATUS_SUCCESS) {
		return status;
	}
	*value = iqs9151_le_u16(tmp);
	return I2C_STATUS_SUCCESS;
}

static i2c_status_t iqs9151_write_u16(uint16_t reg, uint16_t value) {
	uint8_t tmp[2];
	iqs9151_put_le_u16(value, tmp);
	return iqs9151_i2c_write(reg, tmp, sizeof(tmp));
}

static uint8_t iqs9151_tp_ati_multdiv_h(uint8_t div) {
	return (uint8_t)((1u << 7) | (uint8_t)(div << 1) | 1u);
}

static i2c_status_t iqs9151_update_bits_u16(uint16_t reg, uint16_t mask, uint16_t value) {
	uint16_t current;
	i2c_status_t status = iqs9151_read_u16(reg, &current);
	if (status != I2C_STATUS_SUCCESS) {
		return status;
	}
	current = (uint16_t)((current & ~mask) | (value & mask));
	return iqs9151_write_u16(reg, current);
}

static bool azoteq_iqs9151_is_ready(void) {
	if (!azoteq_iqs9151_use_ready_pin) {
		return true;
	}

	return !readPin(azoteq_iqs9151_ready_pin);
}

static void azoteq_iqs9151_wait_for_ready(uint16_t timeout_ms) {
	uint16_t elapsed = 0;

	if (!azoteq_iqs9151_use_ready_pin) {
		return;
	}

	while (!azoteq_iqs9151_is_ready() && elapsed < timeout_ms) {
		wait_ms(1);
		elapsed++;
	}

	if (elapsed >= timeout_ms && AZOTEQ_IQS9151_DEBUG_LOG && azoteq_iqs9151_dbg_due(&azoteq_iqs9151_dbg_last_error_ms, AZOTEQ_IQS9151_DEBUG_INTERVAL_MS)) {
		dprintf("IQS9151 DBG: RDY timeout, pin=%d\n", azoteq_iqs9151_use_ready_pin ? (int)readPin(azoteq_iqs9151_ready_pin) : -1);
	}
}

static i2c_status_t iqs9151_write_chunks(uint16_t start_reg, const uint8_t *buf, size_t len) {
	size_t offset = 0;

	while (offset < len) {
		size_t chunk_len = len - offset;
		i2c_status_t status;
		bool         written = false;
		if (chunk_len > IQS9151_I2C_CHUNK_SIZE) {
			chunk_len = IQS9151_I2C_CHUNK_SIZE;
		}

		for (uint8_t attempt = 0; attempt < 3; attempt++) {
			azoteq_iqs9151_wait_for_ready(200);
			if (azoteq_iqs9151_use_ready_pin && !azoteq_iqs9151_is_ready()) {
				status = I2C_STATUS_TIMEOUT;
			} else {
				status = iqs9151_i2c_write((uint16_t)(start_reg + offset), &buf[offset], chunk_len);
			}

			if (status == I2C_STATUS_SUCCESS) {
				written = true;
				break;
			}

			if (AZOTEQ_IQS9151_DEBUG_LOG) {
				dprintf("IQS9151 INIT: cfg write retry reg=0x%04X off=%u len=%u attempt=%u status=%d\n",
						(uint16_t)(start_reg + offset),
						(unsigned)offset,
						(unsigned)chunk_len,
						(unsigned)(attempt + 1),
						status);
			}
			wait_ms(10);
		}

		if (!written) {
			return status;
		}

		offset += chunk_len;
	}

	return I2C_STATUS_SUCCESS;
}

static void iqs9151_parse_frame(const uint8_t *raw, azoteq_iqs9151_frame_t *frame) {
	frame->rel_x = (int16_t)iqs9151_le_u16(&raw[0]);
	frame->rel_y = (int16_t)iqs9151_le_u16(&raw[2]);
	frame->info_flags = iqs9151_le_u16(&raw[12]);
	frame->trackpad_flags = iqs9151_le_u16(&raw[14]);
	frame->finger_count = (uint8_t)(frame->trackpad_flags & IQS9151_TP_FINGER_COUNT_MASK);
	frame->finger1_x = iqs9151_le_u16(&raw[16]);
	frame->finger1_y = iqs9151_le_u16(&raw[18]);
	frame->finger2_x = iqs9151_le_u16(&raw[24]);
	frame->finger2_y = iqs9151_le_u16(&raw[26]);
	frame->finger3_x = iqs9151_le_u16(&raw[32]);
	frame->finger3_y = iqs9151_le_u16(&raw[34]);
	frame->timestamp_ms = timer_read32();
}

static i2c_status_t iqs9151_read_frame(azoteq_iqs9151_frame_t *frame) {
	uint8_t raw[IQS9151_FRAME_READ_SIZE];
	i2c_status_t status;

	azoteq_iqs9151_wait_for_ready(50);
	status = iqs9151_i2c_read(IQS9151_ADDR_RELATIVE_X, raw, sizeof(raw));
	if (status != I2C_STATUS_SUCCESS) {
		return status;
	}

	iqs9151_parse_frame(raw, frame);
	return I2C_STATUS_SUCCESS;
}

static i2c_status_t iqs9151_check_product_number(void) {
	uint16_t product = 0;
	uint8_t  raw[2]  = {0, 0};

	for (uint8_t attempt = 0; attempt < 5; attempt++) {
		i2c_status_t status = iqs9151_i2c_read(IQS9151_ADDR_PRODUCT_NUMBER, raw, sizeof(raw));
		if (status != I2C_STATUS_SUCCESS) {
			dprintf("IQS9151 INIT: product read failed attempt=%u status=%d\n", (unsigned)(attempt + 1), status);
			wait_ms(20);
			continue;
		}

		product = iqs9151_le_u16(raw);
		if (product == AZOTEQ_IQS9151_PRODUCT_NUM) {
			dprintf("IQS9151 INIT: product OK=0x%04X raw=[0x%02X 0x%02X]\n", product, raw[0], raw[1]);
			return I2C_STATUS_SUCCESS;
		}

		dprintf("IQS9151 INIT: unexpected product attempt=%u value=0x%04X raw=[0x%02X 0x%02X] expected=0x%04X\n",
				(unsigned)(attempt + 1), product, raw[0], raw[1], AZOTEQ_IQS9151_PRODUCT_NUM);
		wait_ms(20);
	}

	return I2C_STATUS_ERROR;
}

static i2c_status_t iqs9151_wait_for_show_reset(uint16_t timeout_ms) {
	uint32_t start = timer_read32();

	while (timer_elapsed32(start) < timeout_ms) {
		uint16_t info_flags = 0;
		i2c_status_t status;

		azoteq_iqs9151_wait_for_ready(100);
		status = iqs9151_read_u16(IQS9151_ADDR_INFO_FLAGS, &info_flags);
		if (status != I2C_STATUS_SUCCESS) {
			return status;
		}
		if ((info_flags & IQS9151_INFO_SHOW_RESET) != 0) {
			return I2C_STATUS_SUCCESS;
		}
		wait_ms(10);
	}

	return I2C_STATUS_ERROR;
}

static i2c_status_t iqs9151_sw_reset(void) {
	i2c_status_t status = iqs9151_update_bits_u16(IQS9151_ADDR_SYSTEM_CONTROL, IQS9151_SYS_CTRL_SW_RESET, IQS9151_SYS_CTRL_SW_RESET);
	if (status != I2C_STATUS_SUCCESS) {
		return status;
	}
	return iqs9151_wait_for_show_reset(3000);
}

static i2c_status_t iqs9151_ack_reset(void) {
	azoteq_iqs9151_wait_for_ready(200);
	return iqs9151_update_bits_u16(IQS9151_ADDR_SYSTEM_CONTROL, IQS9151_SYS_CTRL_ACK_RESET, IQS9151_SYS_CTRL_ACK_RESET);
}

static i2c_status_t iqs9151_run_ati(void) {
	uint8_t ctrl[2] = {
		SYSTEM_CONTROL_0,
		(uint8_t)(SYSTEM_CONTROL_1 | IQS9151_SYS_CTRL_ALP_RE_ATI | IQS9151_SYS_CTRL_TP_RE_ATI),
	};

	for (uint8_t attempt = 0; attempt < 3; attempt++) {
		azoteq_iqs9151_wait_for_ready(200);
		i2c_status_t status = iqs9151_i2c_write(IQS9151_ADDR_SYSTEM_CONTROL, ctrl, sizeof(ctrl));
		if (status == I2C_STATUS_SUCCESS) {
			return I2C_STATUS_SUCCESS;
		}

		if (AZOTEQ_IQS9151_DEBUG_LOG) {
			dprintf("IQS9151 INIT: ATI request retry attempt=%u status=%d\n", (unsigned)(attempt + 1), status);
		}
		wait_ms(10);
	}

	return I2C_STATUS_ERROR;
}

static i2c_status_t iqs9151_set_tp_ati_div(uint8_t div) {
	uint8_t ati_mult[2] = {
		TP_ATI_MULTDIV_L,
		iqs9151_tp_ati_multdiv_h(div),
	};

	for (uint8_t attempt = 0; attempt < 3; attempt++) {
		azoteq_iqs9151_wait_for_ready(200);
		i2c_status_t status = iqs9151_i2c_write(IQS9151_ADDR_ATI_MULTIPLIERS, ati_mult, sizeof(ati_mult));
		if (status == I2C_STATUS_SUCCESS) {
			return I2C_STATUS_SUCCESS;
		}

		if (AZOTEQ_IQS9151_DEBUG_LOG) {
			dprintf("IQS9151 INIT: ATI div write retry div=%u attempt=%u status=%d\n", (unsigned)div, (unsigned)(attempt + 1), status);
		}
		wait_ms(10);
	}

	return I2C_STATUS_ERROR;
}

static i2c_status_t iqs9151_wait_for_ati(uint16_t timeout_ms) {
	uint32_t start = timer_read32();
	bool     saw_read_success = false;
	bool     force_poll = false;
	uint32_t rdy_busy_start_ms = 0;

	while (timer_elapsed32(start) < timeout_ms) {
		uint16_t ctrl = 0;
		i2c_status_t status;

		/* During ATI the device may hold RDY busy for a while; avoid forcing I2C reads until it is ready. */
		if (!AZOTEQ_IQS9151_ATI_IGNORE_RDY && !force_poll && azoteq_iqs9151_use_ready_pin && !azoteq_iqs9151_is_ready()) {
			if (rdy_busy_start_ms == 0) {
				rdy_busy_start_ms = timer_read32();
			}

			if (timer_elapsed32(rdy_busy_start_ms) >= AZOTEQ_IQS9151_ATI_RDY_STUCK_MS) {
				force_poll = true;
				if (AZOTEQ_IQS9151_DEBUG_LOG) {
					dprintf("IQS9151 INIT: ATI RDY busy too long, force polling i2c\n");
				}
			} else {
			if (AZOTEQ_IQS9151_DEBUG_LOG && azoteq_iqs9151_dbg_due(&azoteq_iqs9151_dbg_last_error_ms, AZOTEQ_IQS9151_DEBUG_INTERVAL_MS)) {
				dprintf("IQS9151 INIT: ATI busy, waiting RDY pin=%d\n", (int)readPin(azoteq_iqs9151_ready_pin));
			}
			wait_ms(10);
			continue;
			}
		} else {
			rdy_busy_start_ms = 0;
		}

		status = iqs9151_read_u16(IQS9151_ADDR_SYSTEM_CONTROL, &ctrl);
		if (status != I2C_STATUS_SUCCESS) {
			if (AZOTEQ_IQS9151_DEBUG_LOG && azoteq_iqs9151_dbg_due(&azoteq_iqs9151_dbg_last_error_ms, AZOTEQ_IQS9151_DEBUG_INTERVAL_MS)) {
				dprintf("IQS9151 INIT: ATI read system ctrl failed status=%d\n", status);
			}
			wait_ms(10);
			continue;
		}

		saw_read_success = true;
		if ((ctrl & (IQS9151_SYS_CTRL_ALP_RE_ATI | IQS9151_SYS_CTRL_TP_RE_ATI)) == 0) {
			return I2C_STATUS_SUCCESS;
		}
		wait_ms(10);
	}

#if AZOTEQ_IQS9151_ATI_ALLOW_TIMEOUT
	if (AZOTEQ_IQS9151_DEBUG_LOG) {
		dprintf("IQS9151 INIT: ATI timeout ignored (debug), read_success=%d\n", (int)saw_read_success);
	}
	return I2C_STATUS_SUCCESS;
#endif

	if (AZOTEQ_IQS9151_DEBUG_LOG) {
		dprintf("IQS9151 INIT: ATI timeout, read_success=%d\n", (int)saw_read_success);
	}

	(void)saw_read_success; /* silence unused warning if debug log is disabled */

	return I2C_STATUS_TIMEOUT;
}

static i2c_status_t iqs9151_wait_for_post_ati_ready(uint16_t timeout_ms) {
	uint32_t start = timer_read32();
	bool     force_poll = false;
	uint32_t rdy_busy_start_ms = 0;

	while (timer_elapsed32(start) < timeout_ms) {
		uint16_t info_flags = 0;
		i2c_status_t status;

		if (!force_poll && azoteq_iqs9151_use_ready_pin && !azoteq_iqs9151_is_ready()) {
			if (rdy_busy_start_ms == 0) {
				rdy_busy_start_ms = timer_read32();
			}

			if (timer_elapsed32(rdy_busy_start_ms) >= AZOTEQ_IQS9151_ATI_RDY_STUCK_MS) {
				force_poll = true;
				if (AZOTEQ_IQS9151_DEBUG_LOG) {
					dprintf("IQS9151 INIT: post-ATI RDY busy too long, force polling i2c\n");
				}
			} else {
				wait_ms(10);
				continue;
			}
		} else {
			rdy_busy_start_ms = 0;
		}

		status = iqs9151_read_u16(IQS9151_ADDR_INFO_FLAGS, &info_flags);
		if (status == I2C_STATUS_SUCCESS) {
			return I2C_STATUS_SUCCESS;
		}

		if (AZOTEQ_IQS9151_DEBUG_LOG && azoteq_iqs9151_dbg_due(&azoteq_iqs9151_dbg_last_error_ms, AZOTEQ_IQS9151_DEBUG_INTERVAL_MS)) {
			dprintf("IQS9151 INIT: post-ATI ready check failed status=%d\n", status);
		}
		wait_ms(10);
	}

	return I2C_STATUS_TIMEOUT;
}

static i2c_status_t iqs9151_read_ati_min_count(uint16_t *min_count) {
	size_t   node_count = (size_t)AZOTEQ_IQS9151_RX_CHANNEL_COUNT * (size_t)AZOTEQ_IQS9151_TX_CHANNEL_COUNT;
	uint16_t min_value  = UINT16_MAX;
	i2c_status_t status;

	if (min_count == NULL || node_count == 0) {
		return I2C_STATUS_ERROR;
	}

	status = iqs9151_wait_for_post_ati_ready(AZOTEQ_IQS9151_ATI_TIMEOUT_MS);
	if (status != I2C_STATUS_SUCCESS) {
		return status;
	}

	for (size_t index = 0; index < node_count; index++) {
		uint8_t      raw[2];
		bool         read_ok = false;

		for (uint8_t attempt = 0; attempt < 5; attempt++) {
			status = iqs9151_i2c_read((uint16_t)(IQS9151_ADDR_ATI_RESULT_BASE + (index * sizeof(raw))), raw, sizeof(raw));
			if (status == I2C_STATUS_SUCCESS) {
				read_ok = true;
				break;
			}

			if (AZOTEQ_IQS9151_DEBUG_LOG && azoteq_iqs9151_dbg_due(&azoteq_iqs9151_dbg_last_error_ms, AZOTEQ_IQS9151_DEBUG_INTERVAL_MS)) {
				dprintf("IQS9151 INIT: ATI result read retry idx=%u reg=0x%04X attempt=%u status=%d\n",
						(unsigned)index,
						(unsigned)(IQS9151_ADDR_ATI_RESULT_BASE + (index * sizeof(raw))),
						(unsigned)(attempt + 1),
						status);
			}
			wait_ms(10);
		}

		if (!read_ok) {
			return status;
		}

		uint16_t value = (uint16_t)(iqs9151_le_u16(raw) & IQS9151_ATI_RESULT_MASK);
		if (value < min_value) {
			min_value = value;
		}
	}

	*min_count = min_value;
	return I2C_STATUS_SUCCESS;
}

static i2c_status_t iqs9151_auto_tune_tp_sensitivity(void) {
	uint8_t div = AZOTEQ_IQS9151_ATI_AUTO_TUNE_START_DIV;

	while (true) {
		uint16_t     min_count = 0;
		i2c_status_t status = iqs9151_set_tp_ati_div(div);
		if (status != I2C_STATUS_SUCCESS) {
			return status;
		}

		status = iqs9151_run_ati();
		if (status != I2C_STATUS_SUCCESS) {
			return status;
		}

		status = iqs9151_wait_for_ati(AZOTEQ_IQS9151_ATI_TIMEOUT_MS);
		if (status != I2C_STATUS_SUCCESS) {
			return status;
		}

		wait_ms(20);

		status = iqs9151_read_ati_min_count(&min_count);
		if (status != I2C_STATUS_SUCCESS) {
			return status;
		}

		if (AZOTEQ_IQS9151_DEBUG_LOG) {
			dprintf("IQS9151 INIT: ATI tune div=%u min=%u threshold=%u\n",
					(unsigned)div,
					(unsigned)min_count,
					(unsigned)AZOTEQ_IQS9151_ATI_AUTO_TUNE_MIN_COUNT);
		}

		if (min_count >= AZOTEQ_IQS9151_ATI_AUTO_TUNE_MIN_COUNT || div <= AZOTEQ_IQS9151_ATI_AUTO_TUNE_MIN_DIV) {
			return I2C_STATUS_SUCCESS;
		}

		div--;
	}
}

static i2c_status_t iqs9151_set_event_mode(void) {
	uint8_t config_settings[2] = {
		CONFIG_SETTINGS_0,
		(uint8_t)(CONFIG_SETTINGS_1 | (uint8_t)(IQS9151_CFG_EVENT_MODE >> 8)),
	};

	for (uint8_t attempt = 0; attempt < 3; attempt++) {
		azoteq_iqs9151_wait_for_ready(300);
		i2c_status_t status = iqs9151_i2c_write(IQS9151_ADDR_CONFIG_SETTINGS, config_settings, sizeof(config_settings));
		if (status == I2C_STATUS_SUCCESS) {
			return I2C_STATUS_SUCCESS;
		}

		if (AZOTEQ_IQS9151_DEBUG_LOG) {
			dprintf("IQS9151 INIT: event mode retry attempt=%u status=%d\n", (unsigned)(attempt + 1), status);
		}
		wait_ms(10);
	}

	return I2C_STATUS_ERROR;
}

static int iqs9151_build_rxtx_map(uint8_t tx_channel, uint8_t rx_channel, uint8_t *map, size_t map_len) {
	uint8_t idx = 0;
	int8_t rx_start = 0x0A;
	int8_t tx_start;

	if (map == NULL || map_len != IQS9151_RXTX_MAP_SIZE || tx_channel == 0 || rx_channel == 0) {
		return -1;
	}

	if ((uint8_t)(tx_channel + rx_channel) > (IQS9151_RXTX_MAP_SIZE - 1)) {
		return -1;
	}

	memset(map, RX_TX_OPEN, map_len);

	for (uint8_t i = 0; i < rx_channel; i++) {
		map[idx++] = (uint8_t)(rx_start - (int8_t)i);
	}

	tx_start = 0x2B - (10 - (int8_t)tx_channel);
	for (uint8_t i = 0; i < tx_channel; i++) {
		map[idx++] = (uint8_t)(tx_start - (int8_t)i);
	}

	map[IQS9151_RXTX_MAP_SIZE - 1] = RX_TX_OPEN;
	return 0;
}

static i2c_status_t iqs9151_apply_config(void) {
	uint8_t rxtx_map[IQS9151_RXTX_MAP_SIZE];
	i2c_status_t status;

	if (iqs9151_build_rxtx_map(AZOTEQ_IQS9151_TX_CHANNEL_COUNT, AZOTEQ_IQS9151_RX_CHANNEL_COUNT, rxtx_map, sizeof(rxtx_map)) != 0) {
		return I2C_STATUS_ERROR;
	}

	azoteq_iqs9151_wait_for_ready(500);

	status = iqs9151_write_chunks(IQS9151_ADDR_ALP_COMPENSATION, iqs9151_alp_compensation, sizeof(iqs9151_alp_compensation));
	if (status != I2C_STATUS_SUCCESS) {
		dprintf("IQS9151 INIT: apply config fail ALP_COMPENSATION status=%d\n", status);
		return status;
	}
	status = iqs9151_write_chunks(IQS9151_ADDR_SETTINGS_MINOR, iqs9151_main_config, sizeof(iqs9151_main_config));
	if (status != I2C_STATUS_SUCCESS) {
		dprintf("IQS9151 INIT: apply config fail MAIN_CONFIG status=%d\n", status);
		return status;
	}
	status = iqs9151_write_chunks(IQS9151_ADDR_RX_TX_MAPPING, rxtx_map, sizeof(rxtx_map));
	if (status != I2C_STATUS_SUCCESS) {
		dprintf("IQS9151 INIT: apply config fail RX_TX_MAPPING status=%d\n", status);
		return status;
	}
	status = iqs9151_write_chunks(IQS9151_ADDR_CHANNEL_DISABLE, iqs9151_channel_disable, sizeof(iqs9151_channel_disable));
	if (status != I2C_STATUS_SUCCESS) {
		dprintf("IQS9151 INIT: apply config fail CHANNEL_DISABLE status=%d\n", status);
		return status;
	}
	status = iqs9151_write_chunks(IQS9151_ADDR_SNAP_ENABLE, iqs9151_snap_enable, sizeof(iqs9151_snap_enable));
	if (status != I2C_STATUS_SUCCESS) {
		dprintf("IQS9151 INIT: apply config fail SNAP_ENABLE status=%d\n", status);
		return status;
	}

	return I2C_STATUS_SUCCESS;
}

void azoteq_iqs9151_set_cpi(uint16_t cpi) {
	(void)cpi;
}

uint16_t azoteq_iqs9151_get_cpi(void) {
	return 0;
}

void azoteq_iqs9151_init(void) {
	i2c_status_t status;

	i2c_init();

#if AZOTEQ_IQS9151_DEBUG_LOG
	debug_enable = true;
	wait_ms(5000);
#endif

#if AZOTEQ_IQS9151_INIT_STARTUP_DELAY_MS > 0
	wait_ms(AZOTEQ_IQS9151_INIT_STARTUP_DELAY_MS);
#endif
	dprintf("IQS9151 INIT: start\n");

	if (AZOTEQ_IQS9151_RDY_PIN != NO_PIN) {
		azoteq_iqs9151_ready_pin = AZOTEQ_IQS9151_RDY_PIN;
		setPinInputHigh(azoteq_iqs9151_ready_pin);
		azoteq_iqs9151_use_ready_pin = true;
		dprintf("IQS9151 INIT: RDY pin=%d\n", (int)azoteq_iqs9151_ready_pin);
	} else {
		azoteq_iqs9151_use_ready_pin = false;
		azoteq_iqs9151_ready_pin = NO_PIN;
		dprintf("IQS9151 INIT: RDY pin disabled\n");
	}

	trackpad_gesture_parser_init(&azoteq_iqs9151_parser);
	azoteq_iqs9151_prev_show_reset = false;

	if (AZOTEQ_IQS9151_DEBUG_LOG) {
		i2c_status_t ping_cfg = i2c_ping_address(AZOTEQ_IQS9151_ADDRESS, AZOTEQ_IQS9151_TIMEOUT_MS);
		dprintf("IQS9151 INIT: I2C ping addr=0x%02X status=%d\n", AZOTEQ_IQS9151_ADDRESS, ping_cfg);
		(void)ping_cfg; /* silence unused warning if debug log is disabled */

		if (AZOTEQ_IQS9151_ADDRESS >= 2) {
			uint8_t      alt_addr = (uint8_t)(AZOTEQ_IQS9151_ADDRESS >> 1);
			i2c_status_t ping_alt = i2c_ping_address(alt_addr, AZOTEQ_IQS9151_TIMEOUT_MS);
			dprintf("IQS9151 INIT: I2C ping alt_addr=0x%02X status=%d\n", alt_addr, ping_alt);
			(void)ping_alt; /* silence unused warning if debug log is disabled */
		}
	}

	azoteq_iqs9151_wait_for_ready(100);
	status = iqs9151_check_product_number();
	if (status != I2C_STATUS_SUCCESS) {
		dprintf("IQS9151 INIT: fail at product check status=%d\n", status);
		azoteq_iqs9151_init_status = I2C_STATUS_ERROR;
		return;
	}

	status = iqs9151_sw_reset();
	if (status != I2C_STATUS_SUCCESS) {
		dprintf("IQS9151 INIT: fail at sw reset status=%d\n", status);
		azoteq_iqs9151_init_status = I2C_STATUS_ERROR;
		return;
	}
	dprintf("IQS9151 INIT: sw reset OK\n");

	status = iqs9151_ack_reset();
	if (status != I2C_STATUS_SUCCESS) {
		dprintf("IQS9151 INIT: fail at ack reset status=%d\n", status);
		azoteq_iqs9151_init_status = I2C_STATUS_ERROR;
		return;
	}
	dprintf("IQS9151 INIT: ack reset OK\n");
	wait_ms(100);
	azoteq_iqs9151_wait_for_ready(500);

	status = iqs9151_apply_config();
	if (status != I2C_STATUS_SUCCESS) {
		dprintf("IQS9151 INIT: fail at apply config status=%d\n", status);
		azoteq_iqs9151_init_status = I2C_STATUS_ERROR;
		return;
	}
	dprintf("IQS9151 INIT: apply config OK\n");

	status = iqs9151_auto_tune_tp_sensitivity();
	if (status != I2C_STATUS_SUCCESS) {
		dprintf("IQS9151 INIT: fail at ATI tune status=%d\n", status);
		azoteq_iqs9151_init_status = I2C_STATUS_ERROR;
		return;
	}
	dprintf("IQS9151 INIT: ATI tune OK\n");
	wait_ms(50);
	azoteq_iqs9151_wait_for_ready(500);

	status = iqs9151_set_event_mode();
	if (status != I2C_STATUS_SUCCESS) {
		dprintf("IQS9151 INIT: fail at event mode status=%d\n", status);
		azoteq_iqs9151_init_status = I2C_STATUS_ERROR;
		return;
	}

	azoteq_iqs9151_init_status = I2C_STATUS_SUCCESS;
	dprintf("IQS9151 INIT: complete\n");
}

report_mouse_t azoteq_iqs9151_get_report(report_mouse_t mouse_report) {
	azoteq_iqs9151_frame_t   frame;
	trackpad_gesture_frame_t gesture_frame;
	report_mouse_t           report = mouse_report;
	i2c_status_t             frame_status;
	uint8_t                  touch_count;

	if (azoteq_iqs9151_init_status != I2C_STATUS_SUCCESS) {
		if (AZOTEQ_IQS9151_DEBUG_LOG && azoteq_iqs9151_dbg_due(&azoteq_iqs9151_dbg_last_error_ms, AZOTEQ_IQS9151_DEBUG_INTERVAL_MS)) {
			dprintf("IQS9151 DBG: init_status=%d\n", azoteq_iqs9151_init_status);
		}
		return report;
	}

	if (!azoteq_iqs9151_is_ready()) {
		return report;
	}

	frame_status = iqs9151_read_frame(&frame);
	if (frame_status != I2C_STATUS_SUCCESS) {
		if (AZOTEQ_IQS9151_DEBUG_LOG && azoteq_iqs9151_dbg_due(&azoteq_iqs9151_dbg_last_error_ms, AZOTEQ_IQS9151_DEBUG_INTERVAL_MS)) {
			dprintf("IQS9151 DBG: frame read failed, status=%d\n", frame_status);
		}
		return report;
	}

	bool show_reset = (frame.info_flags & IQS9151_INFO_SHOW_RESET) != 0;
	if (show_reset && !azoteq_iqs9151_prev_show_reset) {
		trackpad_gesture_parser_reset(&azoteq_iqs9151_parser);
		if (AZOTEQ_IQS9151_DEBUG_LOG) {
			dprintf("IQS9151 DBG: parser reset on SHOW_RESET edge\n");
		}
	}
	azoteq_iqs9151_prev_show_reset = show_reset;

	gesture_frame.rel_x = frame.rel_x;
	gesture_frame.rel_y = frame.rel_y;
	touch_count = frame.finger_count;
	if (touch_count > TRACKPAD_GESTURE_MAX_TOUCHES) {
		touch_count = TRACKPAD_GESTURE_MAX_TOUCHES;
	}
	gesture_frame.touch_count = touch_count;
	gesture_frame.trackpad_flags = frame.trackpad_flags;
	memset(gesture_frame.points, 0, sizeof(gesture_frame.points));

	gesture_frame.points[0].valid = (frame.finger1_x != UINT16_MAX) && (frame.finger1_y != UINT16_MAX);
	gesture_frame.points[0].x = frame.finger1_x;
	gesture_frame.points[0].y = frame.finger1_y;

	gesture_frame.points[1].valid = (frame.finger2_x != UINT16_MAX) && (frame.finger2_y != UINT16_MAX);
	gesture_frame.points[1].x = frame.finger2_x;
	gesture_frame.points[1].y = frame.finger2_y;

	gesture_frame.points[2].valid = (frame.finger3_x != UINT16_MAX) && (frame.finger3_y != UINT16_MAX);
	gesture_frame.points[2].x = frame.finger3_x;
	gesture_frame.points[2].y = frame.finger3_y;

	gesture_frame.timestamp_ms = frame.timestamp_ms;

	if (AZOTEQ_IQS9151_DEBUG_LOG && azoteq_iqs9151_dbg_due(&azoteq_iqs9151_dbg_last_frame_ms, AZOTEQ_IQS9151_DEBUG_INTERVAL_MS)) {
		dprintf("IQS9151 DBG: fcnt=%u rel=(%d,%d) info=0x%04X tp=0x%04X f1=(%u,%u) f2=(%u,%u) f3=(%u,%u)\n",
				gesture_frame.touch_count,
				frame.rel_x,
				frame.rel_y,
				frame.info_flags,
				frame.trackpad_flags,
				frame.finger1_x,
				frame.finger1_y,
				frame.finger2_x,
				frame.finger2_y,
				frame.finger3_x,
				frame.finger3_y);
	}

	report = trackpad_gesture_parser_process(&azoteq_iqs9151_parser, &gesture_frame);

	if (AZOTEQ_IQS9151_DEBUG_LOG && azoteq_iqs9151_dbg_due(&azoteq_iqs9151_dbg_last_frame_ms, AZOTEQ_IQS9151_DEBUG_INTERVAL_MS)) {
		dprintf("IQS9151 DBG: report x=%d y=%d v=%d h=%d buttons=0x%02X\n", report.x, report.y, report.v, report.h, report.buttons);
	}

#if AZOTEQ_IQS9151_DISABLE_HID_REPORT
	report = mouse_report;
	report.x = 0;
	report.y = 0;
	report.v = 0;
	report.h = 0;
	report.buttons = 0;
#endif

	return report;
}

void pointing_device_driver_init(void) {
	azoteq_iqs9151_init();
}

report_mouse_t pointing_device_driver_get_report(report_mouse_t mouse_report) {
	return azoteq_iqs9151_get_report(mouse_report);
}

void pointing_device_driver_set_cpi(uint16_t cpi) {
	azoteq_iqs9151_set_cpi(cpi);
}

uint16_t pointing_device_driver_get_cpi(void) {
	return azoteq_iqs9151_get_cpi();
}

#endif // AUTO_KDK_TRACKPAD_IQS9151_ENABLED