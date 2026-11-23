/*
Copyright 2026 sekigon (@sekigon-gonnoc)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.
*/

#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "pointing_device.h"

#ifndef AZOTEQ_IQS9151_ADDRESS
#    define AZOTEQ_IQS9151_ADDRESS (0x56 << 1)
#endif

#ifndef AZOTEQ_IQS9151_TIMEOUT_MS
#    define AZOTEQ_IQS9151_TIMEOUT_MS 100
#endif

#ifndef AZOTEQ_IQS9151_RDY_PIN
#    define AZOTEQ_IQS9151_RDY_PIN NO_PIN
#endif

#define AZOTEQ_IQS9151_PRODUCT_NUM 0x09BC

#define IQS9151_ADDR_PRODUCT_NUMBER 0x1000
#define IQS9151_ADDR_RELATIVE_X 0x1014
#define IQS9151_ADDR_INFO_FLAGS 0x1020
#define IQS9151_ADDR_TRACKPAD_FLAGS 0x1022
#define IQS9151_ADDR_FINGER1_X 0x1024
#define IQS9151_ADDR_FINGER1_Y 0x1026
#define IQS9151_ADDR_FINGER2_X 0x102C
#define IQS9151_ADDR_FINGER2_Y 0x102E
#define IQS9151_ADDR_FINGER3_X 0x1034
#define IQS9151_ADDR_FINGER3_Y 0x1036

#define IQS9151_ADDR_ALP_COMPENSATION 0x115C
#define IQS9151_ADDR_SETTINGS_MINOR 0x1178
#define IQS9151_ADDR_ATI_MULTIPLIERS 0x117A
#define IQS9151_ADDR_TRACKPAD_ATI_TARGET 0x1196
#define IQS9151_ADDR_SYSTEM_CONTROL 0x11BC
#define IQS9151_ADDR_CONFIG_SETTINGS 0x11BE
#define IQS9151_ADDR_TRACKPAD_RX_CHANNEL 0x11E3
#define IQS9151_ADDR_TRACKPAD_TX_CHANNEL 0x11E4
#define IQS9151_ADDR_X_RESOLUTION 0x11E6
#define IQS9151_ADDR_Y_RESOLUTION 0x11E8
#define IQS9151_ADDR_XY_DYNAMIC_FILTER_BOTTOM_SPEED 0x11EA
#define IQS9151_ADDR_XY_DYNAMIC_FILTER_TOP_SPEED 0x11EC
#define IQS9151_ADDR_XY_DYNAMIC_FILTER_BOTTOM_BETA 0x11EE
#define IQS9151_ADDR_RX_TX_MAPPING 0x1218
#define IQS9151_ADDR_CHANNEL_DISABLE 0x1246
#define IQS9151_ADDR_SNAP_ENABLE 0x129E

#define IQS9151_ADDR_ATI_RESULT_BASE 0xD000

#define IQS9151_FRAME_READ_SIZE 36
#define IQS9151_I2C_CHUNK_SIZE 30
#define IQS9151_RXTX_MAP_SIZE 46

#define IQS9151_INFO_SHOW_RESET (1u << 7)

#define IQS9151_TP_MOVEMENT_DETECTED (1u << 4)
#define IQS9151_TP_FINGER_COUNT_MASK 0x0Fu

#define IQS9151_SYS_CTRL_SW_RESET (1u << 9)
#define IQS9151_SYS_CTRL_ACK_RESET (1u << 7)
#define IQS9151_SYS_CTRL_ALP_RE_ATI (1u << 6)
#define IQS9151_SYS_CTRL_TP_RE_ATI (1u << 5)

#define IQS9151_CFG_EVENT_MODE (1u << 8)

typedef struct {
    int16_t rel_x;
    int16_t rel_y;
    uint16_t info_flags;
    uint16_t trackpad_flags;
    uint8_t finger_count;
    uint16_t finger1_x;
    uint16_t finger1_y;
    uint16_t finger2_x;
    uint16_t finger2_y;
    uint16_t finger3_x;
    uint16_t finger3_y;
    uint32_t timestamp_ms;
} azoteq_iqs9151_frame_t;

void azoteq_iqs9151_init(void);
report_mouse_t azoteq_iqs9151_get_report(report_mouse_t mouse_report);
void azoteq_iqs9151_set_cpi(uint16_t cpi);
uint16_t azoteq_iqs9151_get_cpi(void);

extern const pointing_device_driver_t azoteq_iqs9151_pointing_device_driver;
