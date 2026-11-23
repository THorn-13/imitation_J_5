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

#include "report.h"

#define TRACKPAD_GESTURE_MAX_TOUCHES 5

typedef struct {
    bool valid;
    uint16_t x;
    uint16_t y;
} trackpad_touch_point_t;

typedef struct {
    int16_t rel_x;
    int16_t rel_y;
    uint8_t touch_count;
    uint16_t trackpad_flags;
    trackpad_touch_point_t points[TRACKPAD_GESTURE_MAX_TOUCHES];
    uint32_t timestamp_ms;
} trackpad_gesture_frame_t;

typedef struct {
    bool one_active;
    bool one_hold;
    uint32_t one_start_ms;
    uint16_t one_start_x;
    uint16_t one_start_y;
    uint16_t one_prev_x;
    uint16_t one_prev_y;

    bool two_active;
    bool two_hold;
    uint32_t two_start_ms;
    int16_t two_center_start_x;
    int16_t two_center_start_y;
    int16_t two_center_prev_x;
    int16_t two_center_prev_y;
    int16_t two_distance_start;
    int16_t two_distance_prev;
    uint8_t two_mode;
    int16_t two_pinch_remainder;
    bool two_release_pending;
    uint32_t two_release_pending_ms;

    bool three_active;
    bool three_hold;
    bool three_swipe_sent;
    bool three_release_pending;
    uint32_t three_release_pending_ms;
    uint32_t three_start_ms;
    int16_t three_start_x;
    int16_t three_start_y;
    int16_t three_prev_x;
    int16_t three_prev_y;
    int16_t three_dx;
    int16_t three_dy;

    uint32_t tap_suppress_start_ms;
    uint16_t tap_suppress_duration_ms;

    uint32_t debug_frame_seq;
    uint32_t debug_last_timestamp_ms;
    uint16_t debug_same_timestamp_count;
} trackpad_gesture_parser_t;

void trackpad_gesture_parser_init(trackpad_gesture_parser_t *parser);
void trackpad_gesture_parser_reset(trackpad_gesture_parser_t *parser);
report_mouse_t trackpad_gesture_parser_process(trackpad_gesture_parser_t *parser, const trackpad_gesture_frame_t *frame);
