/*
Copyright 2026 sekigon (@sekigon-gonnoc)

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 2 of the License, or
(at your option) any later version.
*/

#ifdef AUTO_KDK_TRACKPAD_IQS9151_ENABLED

#include "trackpad_gesture_parser.h"

#include <string.h>

#include "debug.h"
#include "pointing_device.h"
#include "timer.h"

#ifndef IQS_TRACKPAD_TAP_MAX_MS
#    define IQS_TRACKPAD_TAP_MAX_MS 180
#endif

#ifndef IQS_TRACKPAD_1F_HOLD_MS
#    define IQS_TRACKPAD_1F_HOLD_MS 250
#endif

#ifndef IQS_TRACKPAD_2F_HOLD_MS
#    define IQS_TRACKPAD_2F_HOLD_MS 280
#endif

#ifndef IQS_TRACKPAD_3F_HOLD_MS
#    define IQS_TRACKPAD_3F_HOLD_MS 320
#endif

#ifndef IQS_TRACKPAD_TAP_MOVE
#    define IQS_TRACKPAD_TAP_MOVE 24
#endif

#ifndef IQS_TRACKPAD_2F_SCROLL_START_MOVE
#    define IQS_TRACKPAD_2F_SCROLL_START_MOVE 18
#endif

#ifndef IQS_TRACKPAD_2F_PINCH_START_DISTANCE
#    define IQS_TRACKPAD_2F_PINCH_START_DISTANCE 28
#endif

#ifndef IQS_TRACKPAD_2F_SCROLL_START_MOVE_SUM
#    define IQS_TRACKPAD_2F_SCROLL_START_MOVE_SUM 14
#endif

#ifndef IQS_TRACKPAD_2F_PINCH_DOMINANCE_PCT
#    define IQS_TRACKPAD_2F_PINCH_DOMINANCE_PCT 250
#endif

#ifndef IQS_TRACKPAD_2F_PINCH_STEP_MIN
#    define IQS_TRACKPAD_2F_PINCH_STEP_MIN 8
#endif

#ifndef IQS_TRACKPAD_2F_PINCH_WHEEL_DIV
#    define IQS_TRACKPAD_2F_PINCH_WHEEL_DIV 64
#endif

#ifndef IQS_TRACKPAD_3F_SWIPE_THRESHOLD
#    define IQS_TRACKPAD_3F_SWIPE_THRESHOLD 110
#endif

#ifndef IQS_TRACKPAD_SCROLL_DIV
#    define IQS_TRACKPAD_SCROLL_DIV 1
#endif

#ifndef IQS_TRACKPAD_2F_RELEASE_PENDING_MAX_MS
#    define IQS_TRACKPAD_2F_RELEASE_PENDING_MAX_MS 120
#endif

#ifndef IQS_TRACKPAD_3F_RELEASE_PENDING_MAX_MS
#    define IQS_TRACKPAD_3F_RELEASE_PENDING_MAX_MS 150
#endif

#ifndef IQS_TRACKPAD_TAP_SUPPRESS_AFTER_SCROLL_MS
#    define IQS_TRACKPAD_TAP_SUPPRESS_AFTER_SCROLL_MS 220
#endif

#ifndef IQS_TRACKPAD_TAP_SUPPRESS_AFTER_SWIPE_MS
#    define IQS_TRACKPAD_TAP_SUPPRESS_AFTER_SWIPE_MS 260
#endif

#ifndef IQS_TRACKPAD_GESTURE_DEBUG
#    define IQS_TRACKPAD_GESTURE_DEBUG 0
#endif

#ifndef IQS_TRACKPAD_GESTURE_FRAME_DEBUG
#    define IQS_TRACKPAD_GESTURE_FRAME_DEBUG 0
#endif

#if IQS_TRACKPAD_GESTURE_DEBUG
#    define IQS_GDBG(...) dprintf(__VA_ARGS__)
#else
#    define IQS_GDBG(...)
#endif

enum {
	IQS_2F_MODE_NONE = 0,
	IQS_2F_MODE_SCROLL,
	IQS_2F_MODE_PINCH,
};

static uint8_t iqs_count_valid_points(const trackpad_gesture_frame_t *frame) {
	uint8_t count = 0;
	for (uint8_t i = 0; i < TRACKPAD_GESTURE_MAX_TOUCHES; i++) {
		if (frame->points[i].valid) {
			count++;
		}
	}
	return count;
}

#if IQS_TRACKPAD_GESTURE_FRAME_DEBUG
static const char *iqs_2f_mode_name(uint8_t mode) {
	switch (mode) {
		case IQS_2F_MODE_SCROLL:
			return "SCROLL";
		case IQS_2F_MODE_PINCH:
			return "PINCH";
		default:
			return "NONE";
	}
}
#endif

static bool iqs_get_nth_valid_point(const trackpad_gesture_frame_t *frame, uint8_t nth, uint16_t *x, uint16_t *y) {
	uint8_t found = 0;
	for (uint8_t i = 0; i < TRACKPAD_GESTURE_MAX_TOUCHES; i++) {
		if (!frame->points[i].valid) {
			continue;
		}
		if (found == nth) {
			*x = frame->points[i].x;
			*y = frame->points[i].y;
			return true;
		}
		found++;
	}
	return false;
}

static bool iqs_get_primary_point(const trackpad_gesture_frame_t *frame, uint16_t *x, uint16_t *y) {
	return iqs_get_nth_valid_point(frame, 0, x, y);
}

static bool iqs_get_centroid(const trackpad_gesture_frame_t *frame, uint8_t limit, int16_t *x, int16_t *y, uint8_t *valid_count) {
	int32_t sum_x = 0;
	int32_t sum_y = 0;
	uint8_t count = 0;

	if (limit > frame->touch_count) {
		limit = frame->touch_count;
	}

	for (uint8_t i = 0; i < TRACKPAD_GESTURE_MAX_TOUCHES && count < limit; i++) {
		if (!frame->points[i].valid) {
			continue;
		}
		sum_x += frame->points[i].x;
		sum_y += frame->points[i].y;
		count++;
	}

	if (valid_count != NULL) {
		*valid_count = count;
	}

	if (count == 0) {
		return false;
	}

	*x = (int16_t)(sum_x / count);
	*y = (int16_t)(sum_y / count);
	return true;
}

static int16_t iqs_abs16(int16_t value) {
	return value < 0 ? (int16_t)-value : value;
}

static int16_t iqs_clamp16(int32_t value) {
	if (value > INT8_MAX) {
		return INT8_MAX;
	}
	if (value < INT8_MIN) {
		return INT8_MIN;
	}
	return (int16_t)value;
}

static int16_t iqs_two_finger_distance(uint16_t x1, uint16_t y1, uint16_t x2, uint16_t y2) {
	return (int16_t)(iqs_abs16((int16_t)(x1 - x2)) + iqs_abs16((int16_t)(y1 - y2)));
}

static bool iqs_is_tap(uint32_t now_ms, uint32_t start_ms, int16_t dx, int16_t dy) {
	const uint32_t dt = timer_elapsed32(start_ms == 0 ? now_ms : start_ms);
	return dt <= IQS_TRACKPAD_TAP_MAX_MS && iqs_abs16(dx) <= IQS_TRACKPAD_TAP_MOVE && iqs_abs16(dy) <= IQS_TRACKPAD_TAP_MOVE;
}

static void iqs_suppress_tap(trackpad_gesture_parser_t *parser, uint16_t duration_ms) {
	parser->tap_suppress_start_ms = timer_read32();
	parser->tap_suppress_duration_ms = duration_ms;
	IQS_GDBG("GEST TAP suppress start duration=%u\n", (unsigned)duration_ms);
}

static bool iqs_is_tap_suppressed(trackpad_gesture_parser_t *parser) {
	if (parser->tap_suppress_duration_ms == 0) {
		return false;
	}

	if (timer_elapsed32(parser->tap_suppress_start_ms) < parser->tap_suppress_duration_ms) {
		return true;
	}

	parser->tap_suppress_duration_ms = 0;
	return false;
}

void trackpad_gesture_parser_init(trackpad_gesture_parser_t *parser) {
	trackpad_gesture_parser_reset(parser);
}

void trackpad_gesture_parser_reset(trackpad_gesture_parser_t *parser) {
	const uint32_t debug_frame_seq = parser->debug_frame_seq;
	const uint32_t debug_last_timestamp_ms = parser->debug_last_timestamp_ms;
	const uint16_t debug_same_timestamp_count = parser->debug_same_timestamp_count;

	memset(parser, 0, sizeof(*parser));

	parser->debug_frame_seq = debug_frame_seq;
	parser->debug_last_timestamp_ms = debug_last_timestamp_ms;
	parser->debug_same_timestamp_count = debug_same_timestamp_count;
}

report_mouse_t trackpad_gesture_parser_process(trackpad_gesture_parser_t *parser, const trackpad_gesture_frame_t *frame) {
	report_mouse_t report = {0};
	const uint8_t valid_points = iqs_count_valid_points(frame);
	const uint8_t effective_touch_count = valid_points < frame->touch_count ? valid_points : frame->touch_count;

	parser->debug_frame_seq++;
	if (parser->debug_last_timestamp_ms == frame->timestamp_ms) {
		if (parser->debug_same_timestamp_count < UINT16_MAX) {
			parser->debug_same_timestamp_count++;
		}
	} else {
		parser->debug_last_timestamp_ms = frame->timestamp_ms;
		parser->debug_same_timestamp_count = 0;
	}

	if (IQS_TRACKPAD_GESTURE_FRAME_DEBUG) {
		IQS_GDBG("GEST FRAME seq=%lu ts=%lu same_ts=%u tc=%u valid=%u eff=%u rel=(%d,%d) state:1f=%d 2f=%d(%s hold=%d pend=%d) 3f=%d(hold=%d swipe=%d pend=%d)\n",
				 (unsigned long)parser->debug_frame_seq,
				 (unsigned long)frame->timestamp_ms,
				 (unsigned)parser->debug_same_timestamp_count,
				 (unsigned)frame->touch_count,
				 (unsigned)valid_points,
				 (unsigned)effective_touch_count,
				 frame->rel_x,
				 frame->rel_y,
				 (int)parser->one_active,
				 (int)parser->two_active,
				 iqs_2f_mode_name(parser->two_mode),
				 (int)parser->two_hold,
				 (int)parser->two_release_pending,
				 (int)parser->three_active,
				 (int)parser->three_hold,
				 (int)parser->three_swipe_sent,
				 (int)parser->three_release_pending);
	} else if (frame->touch_count != valid_points) {
		IQS_GDBG("GEST FRAME mismatch seq=%lu ts=%lu tc=%u valid=%u eff=%u\n",
				 (unsigned long)parser->debug_frame_seq,
				 (unsigned long)frame->timestamp_ms,
				 (unsigned)frame->touch_count,
				 (unsigned)valid_points,
				 (unsigned)effective_touch_count);
	}

	/*
	 * IQS9151 occasionally reports transient non-zero touch_count with no valid points.
	 * Treat these as corrupted frames and ignore them so they cannot force false releases/taps.
	 */
	if (frame->touch_count > 0 && valid_points == 0) {
		IQS_GDBG("GEST FRAME drop-invalid seq=%lu ts=%lu tc=%u valid=%u\n",
				 (unsigned long)parser->debug_frame_seq,
				 (unsigned long)frame->timestamp_ms,
				 (unsigned)frame->touch_count,
				 (unsigned)valid_points);
		return report;
	}

	if (effective_touch_count == 0) {
		if (parser->three_active) {
			const bool tap = iqs_is_tap(frame->timestamp_ms, parser->three_start_ms, parser->three_dx, parser->three_dy);
			const bool tap_suppressed = iqs_is_tap_suppressed(parser);
			IQS_GDBG("GEST 3F release: hold=%d swipe=%d dx=%d dy=%d tap=%d\n",
					 (int)parser->three_hold,
					 (int)parser->three_swipe_sent,
					 parser->three_dx,
					 parser->three_dy,
					 (int)tap);
			if (parser->three_hold) {
				parser->three_hold = false;
			} else if (!parser->three_swipe_sent && tap && !tap_suppressed) {
				report.buttons |= MOUSE_BTN3;
			}
		}

		if (parser->two_active) {
			const int16_t center_dx = parser->two_center_prev_x - parser->two_center_start_x;
			const int16_t center_dy = parser->two_center_prev_y - parser->two_center_start_y;
			const bool tap = iqs_is_tap(frame->timestamp_ms, parser->two_start_ms, center_dx, center_dy);
			const bool tap_suppressed = iqs_is_tap_suppressed(parser);
			IQS_GDBG("GEST 2F release: hold=%d mode=%u dx=%d dy=%d tap=%d\n",
					 (int)parser->two_hold,
					 (unsigned)parser->two_mode,
					 center_dx,
					 center_dy,
					 (int)tap);
			if (parser->two_hold) {
				parser->two_hold = false;
			} else if (parser->two_mode == IQS_2F_MODE_NONE && tap && !tap_suppressed) {
				report.buttons |= MOUSE_BTN2;
			}
		}

		if (parser->one_active) {
			const int16_t dx = (int16_t)(parser->one_prev_x - parser->one_start_x);
			const int16_t dy = (int16_t)(parser->one_prev_y - parser->one_start_y);
			const bool tap = iqs_is_tap(frame->timestamp_ms, parser->one_start_ms, dx, dy);
			const bool tap_suppressed = iqs_is_tap_suppressed(parser);
			IQS_GDBG("GEST 1F release: hold=%d dx=%d dy=%d tap=%d\n",
					 (int)parser->one_hold,
					 dx,
					 dy,
					 (int)tap);
			if (parser->one_hold) {
				parser->one_hold = false;
			} else if (tap && !tap_suppressed) {
				report.buttons |= MOUSE_BTN1;
			}
		}

		trackpad_gesture_parser_reset(parser);
		return report;
	}

	if (effective_touch_count == 1) {
		if (parser->two_active && (parser->two_mode != IQS_2F_MODE_NONE || parser->two_hold)) {
			/*
			 * Consume the first 2F->1F transition frame so a lifted finger after
			 * two-finger scroll/pinch cannot become an immediate 1F tap source.
			 */
			iqs_suppress_tap(parser, IQS_TRACKPAD_TAP_SUPPRESS_AFTER_SCROLL_MS);
			IQS_GDBG("GEST 2F->1F transition consume mode=%u hold=%d\n", (unsigned)parser->two_mode, (int)parser->two_hold);
			parser->two_active = false;
			parser->two_hold = false;
			parser->two_mode = IQS_2F_MODE_NONE;
			parser->two_release_pending = false;
			return report;
		}

		if (parser->three_active && !parser->three_hold && !parser->three_swipe_sent) {
			if (!parser->three_release_pending) {
				parser->three_release_pending = true;
				parser->three_release_pending_ms = frame->timestamp_ms;
				IQS_GDBG("GEST 3F release pending start (1f)\n");
			}

			if (timer_elapsed32(parser->three_release_pending_ms) <= IQS_TRACKPAD_3F_RELEASE_PENDING_MAX_MS) {
				IQS_GDBG("GEST 3F release pending keep (1f) dt=%lu\n", (unsigned long)timer_elapsed32(parser->three_release_pending_ms));
				return report;
			}

			IQS_GDBG("GEST 3F release pending timeout (1f), fallback\n");
			parser->three_active = false;
			parser->three_release_pending = false;
		}

		if (parser->two_active && !parser->two_hold && parser->two_mode == IQS_2F_MODE_NONE) {
			if (!parser->two_release_pending) {
				parser->two_release_pending = true;
				parser->two_release_pending_ms = frame->timestamp_ms;
				IQS_GDBG("GEST 2F release pending start\n");
			}

			if (timer_elapsed32(parser->two_release_pending_ms) <= IQS_TRACKPAD_2F_RELEASE_PENDING_MAX_MS) {
				IQS_GDBG("GEST 2F release pending keep dt=%lu\n", (unsigned long)timer_elapsed32(parser->two_release_pending_ms));
				return report;
			}

			IQS_GDBG("GEST 2F release pending timeout, fallback to 1F\n");
			parser->two_active = false;
			parser->two_release_pending = false;
		}

		uint16_t x = 0;
		uint16_t y = 0;
		const bool have_primary = iqs_get_primary_point(frame, &x, &y);

		if (!parser->one_active) {
			parser->one_active = true;
			parser->one_start_ms = frame->timestamp_ms;
			parser->one_start_x = have_primary ? x : 0;
			parser->one_start_y = have_primary ? y : 0;
			parser->one_prev_x = parser->one_start_x;
			parser->one_prev_y = parser->one_start_y;
		}

		if (have_primary) {
			parser->one_prev_x = x;
			parser->one_prev_y = y;
		}

		if (!parser->one_hold) {
			const uint32_t dt = timer_elapsed32(parser->one_start_ms);
			const int16_t dx = (int16_t)(parser->one_prev_x - parser->one_start_x);
			const int16_t dy = (int16_t)(parser->one_prev_y - parser->one_start_y);
			if (dt >= IQS_TRACKPAD_1F_HOLD_MS && iqs_abs16(dx) <= IQS_TRACKPAD_TAP_MOVE && iqs_abs16(dy) <= IQS_TRACKPAD_TAP_MOVE) {
				parser->one_hold = true;
			}
		}

		if (parser->one_hold) {
			report.buttons |= MOUSE_BTN1;
		}

		report.x = CONSTRAIN_HID_XY(frame->rel_x);
		report.y = CONSTRAIN_HID_XY(frame->rel_y);

		parser->two_active = false;
		parser->three_active = false;
		return report;
	}

	if (effective_touch_count == 2) {
		if (parser->three_active && !parser->three_hold && !parser->three_swipe_sent) {
			if (!parser->three_release_pending) {
				parser->three_release_pending = true;
				parser->three_release_pending_ms = frame->timestamp_ms;
				IQS_GDBG("GEST 3F release pending start (2f)\n");
			}

			if (timer_elapsed32(parser->three_release_pending_ms) <= IQS_TRACKPAD_3F_RELEASE_PENDING_MAX_MS) {
				IQS_GDBG("GEST 3F release pending keep (2f) dt=%lu\n", (unsigned long)timer_elapsed32(parser->three_release_pending_ms));
				return report;
			}

			IQS_GDBG("GEST 3F release pending timeout (2f), fallback to 2F\n");
			parser->three_active = false;
			parser->three_release_pending = false;
		}

		uint16_t f1x = 0;
		uint16_t f1y = 0;
		uint16_t f2x = 0;
		uint16_t f2y = 0;
		const bool have_two = iqs_get_nth_valid_point(frame, 0, &f1x, &f1y) && iqs_get_nth_valid_point(frame, 1, &f2x, &f2y);
		const int16_t center_x = have_two ? (int16_t)(((int32_t)f1x + f2x) / 2) : parser->two_center_prev_x;
		const int16_t center_y = have_two ? (int16_t)(((int32_t)f1y + f2y) / 2) : parser->two_center_prev_y;
		const int16_t distance = have_two ? iqs_two_finger_distance(f1x, f1y, f2x, f2y) : parser->two_distance_prev;

		parser->one_active = false;
		parser->three_active = false;

		if (!parser->two_active) {
			IQS_GDBG("GEST 2F start\n");
			parser->two_active = true;
			parser->two_hold = false;
			parser->two_release_pending = false;
			parser->two_start_ms = frame->timestamp_ms;
			parser->two_center_start_x = center_x;
			parser->two_center_start_y = center_y;
			parser->two_center_prev_x = center_x;
			parser->two_center_prev_y = center_y;
			parser->two_distance_start = distance;
			parser->two_distance_prev = distance;
			parser->two_mode = IQS_2F_MODE_NONE;
			parser->two_pinch_remainder = 0;
			return report;
		}

		const int16_t step_x = (int16_t)(center_x - parser->two_center_prev_x);
		const int16_t step_y = (int16_t)(center_y - parser->two_center_prev_y);
		const int16_t step_dist = (int16_t)(distance - parser->two_distance_prev);
		const int16_t total_center_dx = (int16_t)(center_x - parser->two_center_start_x);
		const int16_t total_center_dy = (int16_t)(center_y - parser->two_center_start_y);
		const int16_t total_dist = (int16_t)(distance - parser->two_distance_start);
		const int16_t total_center_move = (int16_t)(iqs_abs16(total_center_dx) + iqs_abs16(total_center_dy));

		parser->two_center_prev_x = center_x;
		parser->two_center_prev_y = center_y;
		parser->two_distance_prev = distance;
		parser->two_release_pending = false;

		if (parser->two_mode == IQS_2F_MODE_NONE) {
			const int16_t abs_dist = iqs_abs16(total_dist);
			const bool pinch_dominant = ((int32_t)abs_dist * 100) >= ((int32_t)total_center_move * IQS_TRACKPAD_2F_PINCH_DOMINANCE_PCT);
			const bool pinch_step_ok = iqs_abs16(step_dist) >= IQS_TRACKPAD_2F_PINCH_STEP_MIN;

			if (abs_dist > IQS_TRACKPAD_2F_PINCH_START_DISTANCE && pinch_dominant && pinch_step_ok) {
				parser->two_mode = IQS_2F_MODE_PINCH;
				iqs_suppress_tap(parser, IQS_TRACKPAD_TAP_SUPPRESS_AFTER_SCROLL_MS);
				IQS_GDBG("GEST 2F mode=PINCH d=%d dx=%d dy=%d move=%d stepd=%d\n", total_dist, total_center_dx, total_center_dy, total_center_move, step_dist);
			} else if (iqs_abs16(total_center_dx) > IQS_TRACKPAD_2F_SCROLL_START_MOVE || iqs_abs16(total_center_dy) > IQS_TRACKPAD_2F_SCROLL_START_MOVE || total_center_move >= IQS_TRACKPAD_2F_SCROLL_START_MOVE_SUM) {
				parser->two_mode = IQS_2F_MODE_SCROLL;
				iqs_suppress_tap(parser, IQS_TRACKPAD_TAP_SUPPRESS_AFTER_SCROLL_MS);
				IQS_GDBG("GEST 2F mode=SCROLL dx=%d dy=%d d=%d move=%d stepd=%d\n", total_center_dx, total_center_dy, total_dist, total_center_move, step_dist);
			}
		}

		if (!parser->two_hold && parser->two_mode == IQS_2F_MODE_NONE) {
			const uint32_t dt = timer_elapsed32(parser->two_start_ms);
			if (dt >= IQS_TRACKPAD_2F_HOLD_MS && iqs_abs16(total_center_dx) <= IQS_TRACKPAD_TAP_MOVE && iqs_abs16(total_center_dy) <= IQS_TRACKPAD_TAP_MOVE) {
				parser->two_hold = true;
			}
		}

		if (parser->two_hold) {
			report.buttons |= MOUSE_BTN2;
		} else if (parser->two_mode == IQS_2F_MODE_SCROLL) {
			report.h = CONSTRAIN_HID((int8_t)iqs_clamp16((-step_x) / IQS_TRACKPAD_SCROLL_DIV));
			report.v = CONSTRAIN_HID((int8_t)iqs_clamp16((step_y) / IQS_TRACKPAD_SCROLL_DIV));
		} else if (parser->two_mode == IQS_2F_MODE_PINCH) {
			const int16_t wheel_acc = (int16_t)(parser->two_pinch_remainder + step_dist);
			const int16_t wheel = wheel_acc / IQS_TRACKPAD_2F_PINCH_WHEEL_DIV;
			parser->two_pinch_remainder = (int16_t)(wheel_acc - wheel * IQS_TRACKPAD_2F_PINCH_WHEEL_DIV);
			report.v = CONSTRAIN_HID((int8_t)iqs_clamp16(wheel));
		}

		return report;
	}

	if (effective_touch_count >= 3) {
		int16_t center_x = 0;
		int16_t center_y = 0;
		uint8_t valid_count = 0;
		const bool have_centroid = iqs_get_centroid(frame, 3, &center_x, &center_y, &valid_count);
		if (IQS_TRACKPAD_GESTURE_FRAME_DEBUG) {
			IQS_GDBG("GEST 3F candidate seq=%lu have_centroid=%d valid_count=%u center=(%d,%d)\n",
					 (unsigned long)parser->debug_frame_seq,
					 (int)have_centroid,
					 (unsigned)valid_count,
					 center_x,
					 center_y);
		}

		parser->one_active = false;
		parser->two_active = false;

		if (!parser->three_active) {
			if (!have_centroid || valid_count < 3) {
				IQS_GDBG("GEST 3F start rejected seq=%lu tc=%u valid=%u eff=%u\n",
						 (unsigned long)parser->debug_frame_seq,
						 (unsigned)frame->touch_count,
						 (unsigned)valid_count,
						 (unsigned)effective_touch_count);
				return report;
			}
			IQS_GDBG("GEST 3F start\n");
			parser->three_active = true;
			parser->three_hold = false;
			parser->three_swipe_sent = false;
			parser->three_release_pending = false;
			parser->three_start_ms = frame->timestamp_ms;
			parser->three_start_x = have_centroid ? center_x : 0;
			parser->three_start_y = have_centroid ? center_y : 0;
			parser->three_prev_x = parser->three_start_x;
			parser->three_prev_y = parser->three_start_y;
			parser->three_dx = 0;
			parser->three_dy = 0;
			return report;
		}

		int16_t step_x = frame->rel_x;
		int16_t step_y = frame->rel_y;

		if (have_centroid && valid_count >= 3) {
			step_x = (int16_t)(center_x - parser->three_prev_x);
			step_y = (int16_t)(center_y - parser->three_prev_y);
			parser->three_prev_x = center_x;
			parser->three_prev_y = center_y;
		}

		parser->three_dx += step_x;
		parser->three_dy += step_y;
		parser->three_release_pending = false;

		if (!parser->three_hold && !parser->three_swipe_sent) {
			const uint32_t dt = timer_elapsed32(parser->three_start_ms);
			if (dt >= IQS_TRACKPAD_3F_HOLD_MS && iqs_abs16(parser->three_dx) <= IQS_TRACKPAD_TAP_MOVE && iqs_abs16(parser->three_dy) <= IQS_TRACKPAD_TAP_MOVE) {
				parser->three_hold = true;
			}
		}

		if (!parser->three_hold && !parser->three_swipe_sent) {
			if (iqs_abs16(parser->three_dx) >= IQS_TRACKPAD_3F_SWIPE_THRESHOLD && iqs_abs16(parser->three_dx) >= iqs_abs16(parser->three_dy)) {
				report.buttons |= parser->three_dx > 0 ? MOUSE_BTN4 : MOUSE_BTN5;
				parser->three_swipe_sent = true;
				iqs_suppress_tap(parser, IQS_TRACKPAD_TAP_SUPPRESS_AFTER_SWIPE_MS);
				IQS_GDBG("GEST 3F swipe X dx=%d dy=%d btn=0x%02X\n", parser->three_dx, parser->three_dy, report.buttons);
			} else if (iqs_abs16(parser->three_dy) >= IQS_TRACKPAD_3F_SWIPE_THRESHOLD && iqs_abs16(parser->three_dy) > iqs_abs16(parser->three_dx)) {
				report.buttons |= parser->three_dy > 0 ? MOUSE_BTN7 : MOUSE_BTN6;
				parser->three_swipe_sent = true;
				iqs_suppress_tap(parser, IQS_TRACKPAD_TAP_SUPPRESS_AFTER_SWIPE_MS);
				IQS_GDBG("GEST 3F swipe Y dx=%d dy=%d btn=0x%02X\n", parser->three_dx, parser->three_dy, report.buttons);
			}
		}

		if (parser->three_hold) {
			report.buttons |= MOUSE_BTN3;
		}
	}

	return report;
}

#endif // AUTO_KDK_TRACKPAD_IQS9151_ENABLED