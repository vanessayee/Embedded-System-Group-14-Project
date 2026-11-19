/*
 *	hump.c
 *	Hump detection and height estimation (see hump.h).
 */

#include "hump.h"
#include "imu_proc.h"

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

EXPORT void hump_init(HUMP *h)
{
	h->state = HS_IDLE;
	h->rise_count = 0;
	h->level_count = 0;
	h->pre_elev_um = 0;
	h->elev_um = 0;
	h->peak_um = 0;
	h->travel_um = 0;
	h->last_height_um = -1;
	h->max_height_um = -1;
	h->last_len_mm = 0;
	h->count = 0;
}

EXPORT BOOL hump_active(const HUMP *h)
{
	return (h->state == HS_CLIMB || h->state == HS_DESCENT) ? TRUE : FALSE;
}

LOCAL void restart(HUMP *h)
{
	h->state = HS_IDLE;
	h->rise_count = 0;
	h->level_count = 0;
	h->pre_elev_um = 0;
	h->elev_um = 0;
	h->peak_um = 0;
	h->travel_um = 0;
}

LOCAL void complete(HUMP *h, HUMP_EVT *ev)
{
	W	height = h->peak_um;

	if (height >= HUMP_MIN_HEIGHT_UM) {
		h->last_height_um = height;
		if (height > h->max_height_um) {
			h->max_height_um = height;	/* FR6: highest of the run */
		}
		h->last_len_mm = h->travel_um / 1000;
		if (h->count < 255U) {
			h->count++;
		}
		ev->completed = TRUE;
		ev->height_um = height;
		ev->length_mm = h->last_len_mm;
	} else {
		ev->rejected = TRUE;
	}
	restart(h);
}

EXPORT void hump_step(HUMP *h, W pitch_cdeg, W ds_um, BOOL valid, HUMP_EVT *ev)
{
	W	dz;

	ev->completed = FALSE;
	ev->rejected = FALSE;
	ev->height_um = 0;
	ev->length_mm = 0;

	if (!valid) {
		if (h->state != HS_IDLE) {
			ev->rejected = TRUE;		/* never measure on bad data */
		}
		restart(h);
		return;
	}
	if (ds_um < 0) {
		if (h->state != HS_IDLE) {
			ev->rejected = TRUE;		/* forward integral is no longer valid */
		}
		restart(h);
		return;
	}
	dz = (W)(((D)ds_um * imath_sin_q15(pitch_cdeg)) / 32768);

	switch (h->state) {
	case HS_IDLE:
		if (pitch_cdeg >= HUMP_START_CDEG) {
			/* Keep only the consecutive samples that satisfy the start
			   condition.  Accumulating every small positive road slope here
			   can otherwise turn a long 1-2 degree incline into a false hump
			   when a later vibration briefly crosses the threshold. */
			h->pre_elev_um += dz;
			h->travel_um += ds_um;
			/* Attitude samples arrive every 10 ms; whole-mm odometry is
			   updated every 20 ms. Zero-distance samples between real wheel
			   updates must not erase an otherwise continuous uphill pitch.
			   Still require actual forward travel before starting. */
			if (h->rise_count < 2U) h->rise_count++;
			if (h->rise_count >= 2U && h->travel_um > 0) {
				h->state = HS_CLIMB;
				h->elev_um = h->pre_elev_um;	/* include the start of the climb */
				h->peak_um = h->elev_um;
				h->level_count = 0;
			}
		} else {
			h->rise_count = 0;
			h->pre_elev_um = 0;
			h->travel_um = 0;
		}
		break;

	case HS_CLIMB:
	case HS_DESCENT:
		h->elev_um += dz;
		h->travel_um += ds_um;
		if (h->elev_um > h->peak_um) {
			h->peak_um = h->elev_um;
		}
		if (pitch_cdeg <= -HUMP_START_CDEG) {
			h->state = HS_DESCENT;
		}
		if (h->state == HS_DESCENT && iabs(pitch_cdeg) < HUMP_LEVEL_CDEG) {
			if (++h->level_count >= HUMP_LEVEL_SAMPLES) {
				complete(h, ev);
				break;
			}
		} else {
			h->level_count = 0;
		}
		if (h->travel_um > HUMP_MAX_LEN_MM * 1000) {
			ev->rejected = TRUE;		/* a slope, not a hump */
			restart(h);
		}
		break;

	default:
		restart(h);
		break;
	}
}

EXPORT void hump_roll_step(HUMP *h, W roll_cdeg, W ds_um, BOOL valid, HUMP_EVT *ev)
{
	W angle = iabs(roll_cdeg);
	W height;
	BOOL active = hump_active(h);
	BOOL blocked = active || h->state == HS_ROLL_WAIT_LEVEL;

	ev->completed = FALSE;
	ev->rejected = FALSE;
	ev->height_um = ev->length_mm = 0;
	if (!valid || ds_um < 0 || angle >= IMU_HANDLE_TILT_CDEG) {
		ev->rejected = active;
		restart(h);
		if (blocked || angle >= IMU_HANDLE_TILT_CDEG) h->state = HS_ROLL_WAIT_LEVEL;
		return;
	}
	if (h->state == HS_ROLL_WAIT_LEVEL) {
		if (angle < HUMP_LEVEL_CDEG) {
			if (++h->level_count >= HUMP_LEVEL_SAMPLES) restart(h);
		} else h->level_count = 0;
		return;
	}
	height = (W)(((D)HUMP_PHYSICAL_TRACK_MM * 1000 * imath_sin_q15(angle)) / 32768);
	if (h->state == HS_IDLE) {
		if (angle >= HUMP_START_CDEG) {
			h->travel_um += ds_um;
			if (h->travel_um > 0 && height > h->pre_elev_um) h->pre_elev_um = height;
			if (h->rise_count < 2U) h->rise_count++;
			if (h->rise_count >= 2U && h->travel_um > 0) {
				h->state = HS_CLIMB;
				h->elev_um = height;
				h->peak_um = h->pre_elev_um;
				h->level_count = 0;
			}
		} else restart(h);
		return;
	}
	h->travel_um += ds_um;
	h->elev_um = height;
	if (height > h->peak_um) h->peak_um = height;
	if (h->peak_um - height >= HUMP_ROLL_FALL_UM) h->state = HS_DESCENT;
	if (h->state == HS_DESCENT && angle < HUMP_LEVEL_CDEG) {
		if (++h->level_count >= HUMP_LEVEL_SAMPLES) {
			complete(h, ev);
			return;
		}
	} else h->level_count = 0;
	if (h->travel_um > HUMP_MAX_LEN_MM * 1000) {
		ev->rejected = TRUE;
		restart(h);
		h->state = HS_ROLL_WAIT_LEVEL;
	}
}

EXPORT UB hump_code(W height_um, BOOL *saturated)
{
	W	step = HUMP_STEP_MM * 1000;
	W	code;

	*saturated = FALSE;
	if (height_um < 0) {
		return HUMP_CODE_NONE;
	}
	code = (height_um + step / 2) / step;		/* round to nearest step */
	if (code > HUMP_CODE_MAX) {
		code = HUMP_CODE_MAX;
		*saturated = TRUE;
	}
	return (UB)code;
}

EXPORT W hump_code_to_um(UB code)
{
	if (code == HUMP_CODE_NONE || code > HUMP_CODE_MAX) {
		return -1;
	}
	return (W)code * HUMP_STEP_MM * 1000;
}

EXPORT void hump_status(const HUMP *h, S4_HUMP_STATUS *s)
{
	BOOL	sat_last, sat_max;
	W	len_cm = h->last_len_mm / 10;

	s->last_code = hump_code(h->last_height_um, &sat_last);
	s->max_code = hump_code(h->max_height_um, &sat_max);
	s->count = h->count;			/* packing saturates it at 15 */
	s->active = hump_active(h) ? 1U : 0U;
	s->last_sat = sat_last ? 1U : 0U;
	s->max_sat = sat_max ? 1U : 0U;
	s->last_len_cm = (UB)((len_cm > 255) ? 255 : len_cm);
}
