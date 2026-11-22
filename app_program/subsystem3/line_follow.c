/*
 *	line_follow.c
 *	Line position, steering and line-loss recovery (see line_follow.h).
 */

#include "line_follow.h"

#if LF_HOLD_MS > 511U || LF_SETTLE_MS > 511U || LF_RIGHT_TURN_STRAIGHT_MS > 255U
#error "Update Buddy 3 bounded timer storage for these settings"
#endif

/* This robot's LM393 D0 wiring is high over black (no reflection). */
#define LF_DEFAULT_BLACK	LF_CONFIGURED_BLACK_LEVEL

EXPORT void lf_reset_tracking(LINE_FOLLOW *lf)
{
	lf->state = lf->calibrated ? LS_ON_LINE : LS_UNKNOWN;
	lf->pos = LP_CENTER;
	lf->err = 0;
	lf->steer_cmd = 0;
	lf->edge_side = 0;
	lf->edge_ms = 0;
	lf->hold_side = 0;
	lf->hold_ms = 0;
	lf->lost_ms = 0;
	lf->lost_start_mm = 0;
	lf->lost_pending = FALSE;
	lf->settle_ms = 0;
	lf->straight_ms = 0;
	lf->turning = FALSE;
	lf->turn_start_mm = 0;
	lf->turn_ms = 0;
	lf->in_junction = FALSE;
	lf->junction_reported = FALSE;
	lf->junction_ms = 0;
	lf->post_turn = FALSE;
	lf->post_turn_ms = 0;
	lf->post_pivot = 0;
}

EXPORT void lf_turn_done(LINE_FOLLOW *lf, BOOL centre_acquired)
{
	lf_reset_tracking(lf);
	if (lf->calibrated) {
		lf->post_turn = !centre_acquired;
		lf->straight_ms = centre_acquired ? LF_RIGHT_TURN_STRAIGHT_MS : 0;
	}
}

EXPORT void lf_init(LINE_FOLLOW *lf)
{
	lf->black_levels = LF_DEFAULT_BLACK ? 7U : 0U;
	lf->calibrated = FALSE;
	lf_reset_tracking(lf);
}

EXPORT INT lf_calibrate(LINE_FOLLOW *lf, const UB *raw, UINT n)
{
	UINT	ones[2] = {0, 0};
	UINT	i, k;

	if (n == 0U) {
		return LF_CAL_UNSTABLE;
	}
	for (k = 0; k < n; k++) {
		for (i = 0; i < 2U; i++) {
			ones[i] += (raw[k] >> i) & 1U;
		}
	}
	/* GP26 may be crossing barcode bars while GP6/GP2 are stable.
	   Its transitions must never block qualification of line tracking. */
	for (i = 0; i < 2U; i++) {
		UINT	minority;
		UB	major;

		major = (ones[i] * 2U > n) ? 1U : 0U;
		minority = major ? (n - ones[i]) : ones[i];
		if (minority * 5U > n) {
			return LF_CAL_UNSTABLE;		/* > 20 % flicker while still */
		}
	}
	/* Starting position is deliberately irrelevant: all-white, all-black,
	   centre-on-line and right-sensor-on-line must all enter NORMAL. */
	lf->black_levels = LF_CONFIGURED_BLACK_LEVEL ? 7U : 0U;
	lf->calibrated = TRUE;
	lf_reset_tracking(lf);
	return LF_CAL_OK;
}

EXPORT UB lf_black_bits(const LINE_FOLLOW *lf, UB raw)
{
	UB	b = 0;
	UINT	i;

	for (i = 0; i < 3U; i++) {
		if (((raw >> i) & 1U) == lf_black_level(lf, i)) {
			b |= (UB)IR_BIT(i);
		}
	}
	return b;
}

LOCAL void classify(LINE_FOLLOW *lf, UB black, UW dt_ms, LF_OUT *out)
{
	BOOL	l = (black & IR_BIT(IR_L)) != 0U;
	BOOL	c = (black & IR_BIT(IR_C)) != 0U;

	if (l && c) {
		/* Only the two tracking sensors define a junction. GP26 is
		   the separate barcode sensor and need not see the crossing. */
		if (!lf->in_junction) {
			lf->in_junction = TRUE;
			lf->junction_reported = FALSE;
			lf->junction_ms = 0;
		}
		lf->junction_ms += dt_ms;
		if (!lf->junction_reported
		    && lf->junction_ms >= LF_JUNCTION_CONFIRM_MS) {
			out->events |= LF_EV_JUNCTION;
			lf->junction_reported = TRUE;	/* report once */
		}
		lf->pos = LP_JUNCTION;
		lf->err = 0;
		return;
	}
	lf->in_junction = FALSE;
	lf->junction_ms = 0;
	/* Otherwise IR_R is the barcode sensor: whatever it reads, it never
	   steers.  C and L bracket the desired white corridor. */
	if (c) {
		lf->pos = LP_RIGHT_NEAR;
		lf->err = 1;	/* task_line mirrors this to physical LEFT */
	} else if (l) {
		lf->pos = LP_LEFT_NEAR;
		lf->err = -1;
	} else if (lf->state == LS_ON_LINE && !lf->turning) {
		lf->pos = LP_CENTER;
		lf->err = LF_EDGE_GAP_ERR;
	} else {
		lf->pos = LP_LOST_LEFT;	/* search towards the tracking gap */
		lf->err = 3 * LF_LOST_DIR;
	}
}

LOCAL INT steer_slew(INT current, INT target)
{
	if (target > current + LF_STEER_SLEW_STEP) {
		return current + LF_STEER_SLEW_STEP;
	}
	if (target < current - LF_STEER_SLEW_STEP) {
		return current - LF_STEER_SLEW_STEP;
	}
	return target;
}

/* Steering target while on the line.  A sensor on black turns the car
   towards it (task_line mirrors the sign: C black -> right wheel faster),
   harder the longer it stays black.  When a sustained contact ends, the
   line may have slipped past the sensor rather than back into the
   corridor, so keep turning that way for LF_HOLD_MS instead of driving
   straight off it. */
LOCAL INT edge_steer(LINE_FOLLOW *lf, UW dt_ms)
{
	INT	mag;

	if (lf->pos == LP_JUNCTION) {
		lf->edge_side = 0;
		lf->edge_ms = 0;
		lf->hold_ms = 0;
		return 0;			/* crossing line: straight */
	}
	if (lf->err != 0) {
		if (lf->err != lf->edge_side) {
			lf->edge_side = lf->err;
			lf->edge_ms = 0;
		}
		mag = LF_EDGE_STEER_MIN + (INT)(lf->edge_ms / LF_EDGE_GROW_MS);
		if (mag > LF_EDGE_STEER_MAX) {
			mag = LF_EDGE_STEER_MAX;
		}
		lf->edge_ms += dt_ms;
		lf->hold_ms = 0;
		return lf->err * mag;
	}
	if (lf->edge_side != 0 && lf->edge_ms >= LF_HOLD_MIN_MS) {
		lf->hold_side = lf->edge_side;	/* sustained contact just ended */
		lf->hold_ms = LF_HOLD_MS;
	}
	lf->edge_side = 0;
	lf->edge_ms = 0;
	if (lf->hold_ms > 0U) {
		lf->hold_ms = (lf->hold_ms > dt_ms) ? lf->hold_ms - dt_ms : 0U;
		return lf->hold_side * LF_HOLD_STEER;
	}
	return 0;
}

/* Realign after a barcode turn (see LF_POST_TURN_*).  Returns TRUE while
   it owns the steering; FALSE once done (normal tracking from then on).
   err sign convention: +1 centre black, -1 GP2 (IR_L) black. */
LOCAL BOOL post_turn_step(LINE_FOLLOW *lf, BOOL c, BOOL l, UW dt_ms,
			  LF_OUT *out)
{
	INT	side = (c && !l) ? 1 : ((l && !c) ? -1 : 0);

	lf->post_turn_ms += dt_ms;
	if (lf->post_pivot == 0 && side != 0) {
		/* turned past the line (left turn: GP2 on it) or short of it
		   (centre on it): pivot towards that sensor */
		lf->post_pivot = side;
		lf->post_turn_ms = 0;
	}
	if (lf->post_pivot != 0) {
		if (side == lf->post_pivot
		    && lf->post_turn_ms < LF_POST_TURN_PIVOT_MAX_MS) {
			out->steer = side * LF_POST_TURN_PIVOT_STEER;
			out->speed_frac = 0;
			lf->steer_cmd = out->steer;
			return TRUE;
		}
		/* sensor white again: the line is between the two sensors */
		lf->settle_ms = LF_SETTLE_MS;
	} else if (!c && !l && lf->post_turn_ms < LF_POST_TURN_MS) {
		/* line already between the sensors: straight on */
		out->steer = 0;
		out->speed_frac = LF_CRUISE_SPEED;
		lf->steer_cmd = 0;
		return TRUE;
	}
	lf->post_turn = FALSE;
	lf->post_pivot = 0;
	lf->post_turn_ms = 0;
	lf->steer_cmd = 0;		/* aligned: start from straight */
	lf->edge_side = 0;
	lf->edge_ms = 0;
	lf->hold_ms = 0;
	return FALSE;
}

EXPORT void lf_step(LINE_FOLLOW *lf, UB black, W odo_mm, UW dt_ms,
		    BOOL navigating, LF_OUT *out)
{
	BOOL	c;
	BOOL	l;
	BOOL	on_line;

	out->steer = 0;
	out->speed_frac = LF_CRUISE_SPEED;
	out->events = 0;
	out->steering = FALSE;
	if (!lf->calibrated) {
		return;
	}

	/* Both-white is the intended tracking corridor, not evidence of
	   losing the line. Elapsed time alone cannot distinguish those cases. */
	c = (black & IR_BIT(IR_C)) != 0U;
	l = (black & IR_BIT(IR_L)) != 0U;

	classify(lf, black, dt_ms, out);
	/* Ordinary edge contact is always a moving correction. */
	/* During avoidance Buddy 5 needs true sensor-based LOST/FOUND events,
	   so the virtual white corridor applies only while Buddy 3 owns
	   steering in Normal Navigation. */
	on_line = c || l
		  || (navigating && lf->state == LS_ON_LINE && !lf->turning);
	if (on_line) {
		if (lf->turning) {
			lf->steer_cmd = 0;	/* found: ramp from straight, not the pivot */

		}
		lf->turning = FALSE;
	} else {
		if (!lf->turning) {
			lf->turning = TRUE;
			lf->turn_start_mm = odo_mm;
			lf->turn_ms = 0;
		}
		lf->turn_ms += dt_ms;
	}

	if (on_line) {
		if (lf->state != LS_ON_LINE) {
			out->events |= LF_EV_FOUND;	/* line reacquired */
		}
		lf->state = LS_ON_LINE;
		lf->lost_ms = 0;
		lf->lost_pending = FALSE;
	} else if (lf->state == LS_ON_LINE) {
		if (!lf->lost_pending) {
			lf->lost_pending = TRUE;
			lf->lost_start_mm = odo_mm;
			lf->lost_ms = 0;
		}
		lf->lost_ms += dt_ms;
		if (odo_mm - lf->lost_start_mm >= LF_LOST_CONFIRM_MM
		    || lf->lost_ms >= LF_LOST_CONFIRM_MS) {
			lf->state = navigating ? LS_SEARCHING : LS_LOST;
			out->events |= LF_EV_LOST;
		}
	} else if (lf->state == LS_LOST && navigating) {
		lf->state = LS_SEARCHING;	/* lost outside Normal Navigation */
	}

	if (!navigating) {
		lf->post_turn = FALSE;
		lf->post_pivot = 0;
		lf->straight_ms = 0;
	}
	if (navigating) {
		out->steering = TRUE;
		if (on_line && lf->straight_ms > 0U) {
			out->steer = 0;
			out->speed_frac = LF_SETTLE_SPEED;
		} else if (on_line && lf->post_turn
		    && post_turn_step(lf, c, l, dt_ms, out)) {
			/* realigning after a barcode turn */
		} else if (!on_line && lf->turn_ms <= LF_LOST_ARC_MS
			   && odo_mm - lf->turn_start_mm < LF_LOST_ARC_MM) {
			/* centre and IR_L white: arc towards IR_R (the car's
			   left) at once while moving on, a curve ... */
			out->steer = LF_LOST_DIR * LF_LOST_ARC_STEER;
			out->speed_frac = LF_LOST_ARC_SPEED;
		} else if (!on_line) {
			/* ... still white: a sharp corner, turn on the spot
			   until one of them sees black.  Never gives up. */
			out->steer = LF_LOST_DIR * LF_LOST_PIVOT_STEER;
			out->speed_frac = 0;

		} else {
			INT target = edge_steer(lf, dt_ms);

			out->steer = steer_slew(lf->steer_cmd, target);
			lf->steer_cmd = out->steer;
			out->speed_frac = LF_CRUISE_SPEED;
			if (lf->settle_ms > 0U && out->speed_frac > LF_SETTLE_SPEED) {
				out->speed_frac = LF_SETTLE_SPEED;
			}
		}
		if (!on_line) {
			/* Recovery has its own bounded arc/pivot values.  Start
			   normal slew limiting from that value when the line is found. */
			lf->steer_cmd = out->steer;
			lf->edge_side = 0;
			lf->edge_ms = 0;
			lf->hold_ms = 0;
		}
	}
	lf->straight_ms = (lf->straight_ms > dt_ms) ? lf->straight_ms - dt_ms : 0U;
	lf->settle_ms = (lf->settle_ms > dt_ms) ? lf->settle_ms - dt_ms : 0U;
}
