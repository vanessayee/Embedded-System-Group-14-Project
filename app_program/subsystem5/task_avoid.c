/*
 *	task_avoid.c
 *	Avoidance & Recovery task (Week 6: High, event-driven) -- Buddy 5
 *	statechart (6.5):
 *
 *	  IDLE/MONITORING -obstacle detected-> STOP/REDUCE SPEED (event to
 *	  Buddy 2, wait until stopped, hold) -> FULL FINE SCAN ->
 *	  GENERATE PROFILE -> PLAN AVOIDANCE -> EXECUTE AVOIDANCE
 *	  -> SEARCH FOR ORIGINAL LINE -line found-> REACQUIRE LINE (align)
 *	  -> RESUME MISSION (SYS_EV_AVOID_DONE)
 *	  scan failure / invalid profile / no clearance / failed manoeuvre /
 *	  line not found / timeout -> SAFE STOP (critical fault + event)
 *
 *	Buddy 5 never writes the motors: every movement is an AVOID-source
 *	command to Buddy 2, executed one step at a time and checked against
 *	its done/aborted/fault result.  Every wait is bounded, and so is the
 *	whole recovery (S5_RECOVERY_TIMEOUT_MS).
 */

#include "subsystem5.h"
#include "drv_ultrasonic.h"
#include "../subsystem3/drv_ir.h"

EXPORT volatile UW	s5_avoid_word;

LOCAL S5_AVOID_STATUS	ast;
LOCAL struct { unsigned char scan_id : 3; } request_state;
LOCAL UW		deadline_start;

EXPORT void s5_get_avoid_status(S5_AVOID_STATUS *s)
{
	s5_avoid_status_unpack(app_load(&s5_avoid_word), s);
}

LOCAL void set_state(UB st)
{
	ast.state = st;
	app_store(&s5_avoid_word, s5_avoid_status_pack(&ast));
}

LOCAL BOOL expired(void)
{
	return ((UW)(app_now_ms() - deadline_start) > S5_RECOVERY_TIMEOUT_MS) ? TRUE : FALSE;
}

LOCAL void request_scan(UB type, INT from, INT to, UB step)
{
	S5_SCAN_REQ	rq;
	UW		w;

	rq.type = type;
	rq.scan_id = request_state.scan_id;
	rq.start_deg = (B)from;
	rq.end_deg = (B)to;
	rq.step_deg = step;
	w = s5_req_pack(&rq);
	if (tk_snd_mbf(mbf_scan_req, &w, (INT)sizeof(w), TMO_POL) != E_OK) {
		sys_fault_set(SYS_FLT_TASK_COMM);
	}
}

/* Wait for the profile of the current scan id and phase */
LOCAL BOOL wait_profile(UB phase, S5_PROFILE *p)
{
	UW	start = app_now_ms();
	UW	pw[2];

	for (;;) {
		UW	el = app_now_ms() - start;
		INT	n;

		if (el >= S5_SCAN_TIMEOUT_MS) {
			tm_printf((UB*)"avoid: profile timeout, scan %d phase %d after %d ms\n",
				  (INT)request_state.scan_id, (INT)phase, (INT)el);
			return FALSE;
		}
		n = tk_rcv_mbf(mbf_profile, pw, (TMO)(S5_SCAN_TIMEOUT_MS - el));
		if (n != (INT)sizeof(pw)) {
			continue;
		}
		s5_profile_unpack(pw, p);
		if (p->scan_id == request_state.scan_id && p->phase == phase) {
			return TRUE;		/* stale profiles are dropped */
		}
	}
}

/* One manoeuvre step; returns the MRES_* status */
LOCAL UW move_at_speed(UW type, INT arg, UW speed_lvl)
{
	UW	cmd, res = 0;
	TMO	tmo;

	(void)tk_clr_flg(flg_avoid, ~AVEV_EMERGENCY);
	tmo = (TMO)av_motion_wait_ms(type, arg, motion_creep_speed_mms(),
				   S5_STEP_EXTRA_MS);
	if (motion_command(type, MSRC_AVOID, arg, speed_lvl, &cmd) != E_OK) {
		return MRES_REJECTED;
	}
	if (motion_wait_result(cmd, tmo, &res) != E_OK) {
		motion_stop(MSRC_AVOID);
		tm_printf((UB*)"avoid: motion wait expired, type %d arg %d after %d ms, faults 0x%x\n",
			  (INT)type, arg, (INT)tmo, (UINT)sys_fault_get());
		return MRES_FAULT;
	}
	if (mres_status(res) != MRES_DONE) {
		tm_printf((UB*)"avoid: motion failed, type %d arg %d result %d reject %d faults 0x%x\n",
			  (INT)type, arg, (INT)mres_status(res), (INT)mres_reason(res),
			  (UINT)sys_fault_get());
	}
	return mres_status(res);
}

LOCAL UW move(UW type, INT arg)
{
	return move_at_speed(type, arg, S5_AVOID_SPEED_LVL);
}

LOCAL BOOL line_contact(void)
{
	S3_LINE_STATUS	ls;

	s3_get_line_status(&ls);
	return (ls.calibrated && (UW)(app_now_ms() - ls.time_ms) <= 100U
		&& (ls.ir_black & IR_BIT(IR_C)) != 0U) ? TRUE : FALSE;
}

/* SEARCH: drive until the centre sensor, not merely a side sensor, reaches
   the line.  LEV_FOUND is only a wake-up hint because IR_L can raise it. */
LOCAL BOOL search_line(INT dist)
{
	UW	cmd, res;
	UINT	ptn;

	(void)tk_clr_flg(flg_line_evt, ~LEV_FOUND);
	if (motion_command(MCMD_MOVE, MSRC_AVOID, dist, S5_AVOID_SPEED_LVL / 2U, &cmd) != E_OK) {
		return FALSE;
	}
	for (;;) {
		(void)tk_wai_flg(flg_line_evt, LEV_FOUND, TWF_ORW | TWF_BITCLR,
				 &ptn, S3_PERIOD_MS);

		if (line_contact()) {
			motion_stop(MSRC_AVOID);	/* centre sensor is on the line */
			return motion_wait_result(cmd, 1000, &res) == E_OK
				&& (mres_status(res) == MRES_ABORTED || mres_status(res) == MRES_DONE)
				&& sys_mode() == VM_OBSTACLE;
		}
		res = motion_last_result(MSRC_AVOID);
		if ((res & MRES_VALID) != 0U && mres_seq(res) == mcmd_seq(cmd)) {
			UW grace = app_now_ms();

			/* The 20 ms IR task can publish just after the motion result.
			   Hold still briefly instead of declaring failure one sample early. */
			do {
				if (line_contact()) return TRUE;
				(void)tk_dly_tsk(S3_PERIOD_MS);
			} while (mres_status(res) == MRES_DONE
				 && (UW)(app_now_ms() - grace) < S5_LINE_END_GRACE_MS);
			return FALSE;			/* search distance used up */
		}
		if (expired()) {
			motion_stop(MSRC_AVOID);
			return FALSE;
		}
	}
}

/* The angle is a bounded search limit. Only fresh IR samples end the turn
   successfully. GP2 is internal IR_L on the mirrored sensor assembly. */
LOCAL BOOL turn_to_line(INT direction, INT limit)
{
	S3_LINE_STATUS ls;
	AV_LINE_LOCK lock = { 0 };
	UW cmd, res, start = app_now_ms(), last;
	BOOL acquired = FALSE;

	s3_get_line_status(&ls);
	last = ls.time_ms;
	if (motion_command(MCMD_TURN, MSRC_AVOID, direction * limit,
			   S5_AVOID_SPEED_LVL, &cmd) != E_OK) return FALSE;
	while (!expired() && sys_mode() == VM_OBSTACLE
	       && (UW)(app_now_ms() - start) < S5_LINE_ALIGN_TIMEOUT_MS) {
		(void)tk_dly_tsk(S3_PERIOD_MS);
		s3_get_line_status(&ls);
		if (!ls.calibrated || (UW)(app_now_ms() - ls.time_ms) > 100U) break;
		if (ls.time_ms != last) {
			last = ls.time_ms;
			if (av_line_lock_sample(&lock,
				(ls.ir_black & IR_BIT(IR_C)) != 0U,
				(ls.ir_black & IR_BIT(IR_SIDES_MIRRORED ? IR_L : IR_R)) != 0U)) {
				acquired = TRUE;
				break;
			}
		}
		res = motion_last_result(MSRC_AVOID);
		if ((res & MRES_VALID) && mres_seq(res) == mcmd_seq(cmd)) break;
	}

	if (!acquired) {
		res = motion_last_result(MSRC_AVOID);
		tm_printf((UB*)"avoid: alignment incomplete, elapsed %d ms, mode %d, motion result %d, GP6 %d GP2 %d, faults 0x%x\n",
			  (INT)(app_now_ms() - start), (INT)sys_mode(),
			  ((res & MRES_VALID) && mres_seq(res) == mcmd_seq(cmd)) ? (INT)mres_status(res) : -1,
			  (ls.ir_black & IR_BIT(IR_C)) != 0U,
			  (ls.ir_black & IR_BIT(IR_SIDES_MIRRORED ? IR_L : IR_R)) != 0U,
			  (UINT)sys_fault_get());
	}
	motion_stop(MSRC_AVOID);
	if (motion_wait_result(cmd, 1000, &res) != E_OK) return FALSE;
	return acquired && sys_mode() == VM_OBSTACLE
		&& (mres_status(res) == MRES_ABORTED || mres_status(res) == MRES_DONE);
}

/* Once the outgoing line is confirmed, Buddy 3 must own the next moving
   correction. A second in-place left pivot and restart-on-contact loop
   repeatedly swept past the line instead of resuming forward tracking. */
LOCAL BOOL align_line(INT side)
{
	UW start;
	MOTION_STATUS ms;
	S3_LINE_STATUS ls;

	tm_printf((UB*)"avoid: returning %s, seeking GP6 centre contact\n",
		  side < 0 ? "from left" : "from right");
	if (!turn_to_line(side, S5_LINE_ALIGN_LIMIT_DEG)) return FALSE;
	start = app_now_ms();
	while (!expired() && sys_mode() == VM_OBSTACLE
	       && (UW)(app_now_ms() - start) < S5_STOP_WAIT_MS) {
		motion_get_status(&ms);
		s3_get_line_status(&ls);
		if (!ls.calibrated || (UW)(app_now_ms() - ls.time_ms) > 100U) return FALSE;
		if (ms.mstate == MS_IDLE && ms.speed_l_mms >= -10 && ms.speed_l_mms <= 10
		    && ms.speed_r_mms >= -10 && ms.speed_r_mms <= 10) {
			tm_printf((UB*)"avoid: line confirmed, GP6=%d GP2=%d; handing to moving line following\n",
				  (ls.ir_black & IR_BIT(IR_C)) != 0U,
				  (ls.ir_black & IR_BIT(IR_SIDES_MIRRORED ? IR_L : IR_R)) != 0U);
			return TRUE;
		}
		(void)tk_dly_tsk(S3_PERIOD_MS);
	}
	return FALSE;
}
LOCAL void fail(UB reason, UW fault_bits)
{
	motion_stop(MSRC_AVOID);
	ast.fail = reason;
	ast.outcome = AVO_FAILED;
	set_state(AVST_FAILED);
	sys_fault_set(fault_bits);
	sys_post_event(SYS_EV_RECOVERY_FAILED);
	request_scan(SREQ_MONITOR, 0, 0, 0);
	tm_printf((UB*)"avoid: FAILED (reason %d) -> Safe Stop\n", (INT)reason);
}

LOCAL void finish(UB outcome)
{
	ast.outcome = outcome;
	ast.fail = AVF_NONE;
	ast.count = (UB)((ast.count + 1U) & 0xFU);
	set_state(AVST_IDLE);
	request_scan(SREQ_MONITOR, 0, 0, 0);
	sys_post_event(SYS_EV_AVOID_DONE);		/* resume mission (Buddy 2) */
	tm_printf((UB*)"avoid: done (%s)\n",
		  (outcome == AVO_CLEARED) ? "obstacle gone" : "bypassed, line reacquired");
}

LOCAL BOOL wait_stopped(void)
{
	MOTION_STATUS	ms;
	UW		start = app_now_ms();

	while ((UW)(app_now_ms() - start) < S5_STOP_WAIT_MS) {
		motion_get_status(&ms);
		if (ms.mstate == MS_IDLE && ms.speed_l_mms > -20 && ms.speed_l_mms < 20
		    && ms.speed_r_mms > -20 && ms.speed_r_mms < 20) {
			(void)tk_dly_tsk(S5_HOLD_MS);	/* hold position briefly */
			return TRUE;
		}
		(void)tk_dly_tsk(S3_PERIOD_MS);
	}
	return FALSE;
}

/* One full-resolution scan; returns FALSE after reporting the failure. */
LOCAL BOOL profile_obstacle(S5_PROFILE *p, BOOL *cleared)
{
	W	front_gap;

	*cleared = FALSE;
	request_state.scan_id = (UB)((request_state.scan_id + 1U) & 0x7U);
	/* A complete 3-degree sweep already contains the information supplied
	   by the old 15-degree coarse pass, while avoiding a second left-to-right
	   servo movement.  The full limits also prove as much side clearance as
	   this sensor can observe without deriving a second scan window. */
	set_state(AVST_FINE);
	tm_printf((UB*)"avoid: full scan %d, %d..%d deg step %d, timeout %d ms\n",
		  (INT)request_state.scan_id, -S5_SCAN_LIMIT_DEG, S5_SCAN_LIMIT_DEG,
		  S5_FINE_STEP, S5_SCAN_TIMEOUT_MS);
	request_scan(SREQ_FINE, -S5_SCAN_LIMIT_DEG, S5_SCAN_LIMIT_DEG,
		     S5_FINE_STEP);
	if (!wait_profile(1, p)) {
		fail(AVF_PROFILE, SYS_FLT_PROFILE_INVALID);
		return FALSE;
	}
	if (p->status == PRF_NO_OBSTACLE) {
		*cleared = TRUE;
		return FALSE;
	}
	if (p->status != PRF_VALID) {
		fail(AVF_PROFILE, SYS_FLT_PROFILE_INVALID);
		return FALSE;
	}
	front_gap = p->dist_mm - AV_SENSOR_BEHIND_FRONT_MM;
	if (front_gap < 0) {
		front_gap = 0;
	}
	tm_printf((UB*)"avoid: obstacle sensor %d mm, bumper gap %d mm, width %d mm, "
		  "clear L %d R %d mm\n",
		  (INT)p->dist_mm, (INT)front_gap, (INT)p->width_mm,
		  (INT)p->clear_left_mm, (INT)p->clear_right_mm);
	return TRUE;
}

#define EXEC_DONE	0
#define EXEC_EMERGENCY	1	/* stopped for something new in the way */
#define EXEC_FAILED	2	/* failure already reported           */
#define EXEC_ABANDONED	3	/* mode left Obstacle Handling        */

/* Drive parallel to the obstacle while the ultrasonic looks toward it.
   The obstacle must first be seen reliably; only then can consecutive clear
   readings mark its far edge. Use the measured gap to calculate the return
   corner-clearance move, instead
   of waiting for the complete body length to pass while still parallel. */
LOCAL INT pass_obstacle(const AV_PLAN *plan, const S5_PROFILE *profile)
{
	S5_SCAN_STATUS	ss;
	UW		cmd, res, start;
	UINT		ptn;
	S5_PASS_STATE watch = {0};
	INT		look = (plan->side == AV_SIDE_RIGHT) ? -S5_SIDE_LOOK_DEG
							 : S5_SIDE_LOOK_DEG;
	W		odo0 = 0, odo = 0, travelled, depth;
	W		clear_start_mm = -1;
	W		closest_side_mm = 0;
	W		farthest_side_mm = 0;
	W		lateral_mm = AV_ROBOT_WIDTH_MM / 2 + plan->side_gap_mm;
	W		side_sin = op_sin_q15(S5_SIDE_LOOK_DEG);
	W		side_limit = av_side_watch_limit_mm(profile, plan, S5_SIDE_LOOK_DEG,
						       S5_SIDE_ECHO_ALLOWANCE_MM);
	W		acquire_limit = plan->front_gap_mm + AV_SENSOR_BEHIND_FRONT_MM
				      - plan->forward_before_pass_mm + AV_ROBOT_LENGTH_MM
				      + S5_SIDE_ECHO_ALLOWANCE_MM;
	UW		debug_at = 0;
	BOOL		debug_sent = FALSE;
	INT		away = (plan->side == AV_SIDE_RIGHT) ? 1 : -1;	/* turn sign away from obstacle */
	W		confirm_travel = 0, ref_mm = -1, ref_travel = 0, last_side_mm = 0;
	INT		heading_fix = 0;

	s5_get_scan_status(&ss);
	watch.last_seq = ss.seq;
	request_scan(SREQ_SIDE, look, 0, 0);
	start = app_now_ms();
	while ((UW)(app_now_ms() - start) < S5_SIDE_READY_MS) {
		(void)tk_dly_tsk(S3_PERIOD_MS);
		s5_get_scan_status(&ss);
		if (ss.seq != watch.last_seq && ss.servo_deg == look) {
			watch.ready = TRUE;
			break;
		}
		watch.last_seq = ss.seq;
	}
	if (!watch.ready) {
		tm_printf((UB*)"avoid: side watch failed to reach %d deg\n", look);
		fail(AVF_SENSOR, SYS_FLT_AVOID_FAILED);
		return EXEC_FAILED;
	}

	tm_printf((UB*)"avoid: passing obstacle, ultrasonic looking %d deg; "
		  "expected side %d mm, present limit %d mm, far edge needs %d clears\n",
		  look, (INT)((D)lateral_mm * 32768 / side_sin), (INT)side_limit,
		  S5_SIDE_CLEAR_CONFIRM);
	motion_get_odometry(&odo0, NULL);
	if (motion_command(MCMD_MOVE, MSRC_AVOID, AV_SIDE_PASS_MAX_MM,
			   S5_AVOID_SPEED_LVL, &cmd) != E_OK) {
		fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
		return EXEC_FAILED;
	}
	watch.last_seq = ss.seq;

	for (;;) {
		W	side_mm;

		(void)tk_dly_tsk(S3_PERIOD_MS);
		if (sys_mode() != VM_OBSTACLE) {
			motion_stop(MSRC_AVOID);
			return EXEC_ABANDONED;
		}
		if (expired()) {
			motion_stop(MSRC_AVOID);
			fail(AVF_TIMEOUT, SYS_FLT_RECOVERY_TIMEOUT);
			return EXEC_FAILED;
		}

		motion_get_odometry(&odo, NULL);
		travelled = odo - odo0;
		if (travelled < 0) travelled = -travelled;
		s5_get_scan_status(&ss);
		if (ss.seq != watch.last_seq && ss.servo_deg == look) {
			watch.last_seq = ss.seq;
			side_mm = s5_dist_mm(ss.dist_code);
			if (!debug_sent || (UW)(app_now_ms() - debug_at) >= 1000U) {
				debug_sent = TRUE;
				debug_at = app_now_ms();
				tm_printf((UB*)"avoid: side watch travel %d mm, range %d mm result %d, limit %d, seen %d clears %d\n",
					  (INT)travelled, (INT)side_mm, (INT)ss.us_result,
					  (INT)side_limit, (INT)watch.seen, (INT)watch.clear_run);
			}
			if (ss.us_result == US_OK && side_mm > 0
			    && side_mm <= side_limit) {
				if (side_mm > farthest_side_mm) farthest_side_mm = side_mm;
				if (closest_side_mm == 0 || side_mm < closest_side_mm) {
					closest_side_mm = side_mm;
				}
				if (watch.seen_run < S5_SIDE_SEEN_CONFIRM) watch.seen_run++;
				if (watch.seen_run >= S5_SIDE_SEEN_CONFIRM && !watch.seen) {
					W measured_gap = (W)((D)side_mm * side_sin / 32768)
						- AV_ROBOT_WIDTH_MM / 2;

					watch.seen = TRUE;
					confirm_travel = travelled;
					tm_printf((UB*)"avoid: side obstacle confirmed, measured body gap %d mm, "
						  "planned %d mm (requested %d + reserve %d)\n",
						  (INT)measured_gap, (INT)plan->side_gap_mm,
						  AV_SIDE_GAP_MM, AV_EDGE_UNCERTAINTY_MM);
				} else if (watch.seen && !watch.corrected
					   && travelled - confirm_travel >= S5_HEADING_SETTLE_MM) {
					/* Compare ranges S5_HEADING_BASELINE_MM apart; slide the
					   reference while the car is parallel enough. */
					if (ref_mm < 0) {
						ref_mm = side_mm;
						ref_travel = travelled;
					} else if (travelled - ref_travel >= S5_HEADING_BASELINE_MM) {
						INT conv = av_side_convergence_deg(ref_mm, side_mm,
							travelled - ref_travel, S5_SIDE_LOOK_DEG);

						if (conv >= S5_HEADING_FIX_MIN_DEG) {
							heading_fix = (conv > S5_HEADING_FIX_MAX_DEG)
								    ? S5_HEADING_FIX_MAX_DEG : conv;
							tm_printf((UB*)"avoid: converging %d deg on the obstacle "
								  "(side %d -> %d mm over %d mm)\n", conv,
								  (INT)ref_mm, (INT)side_mm,
								  (INT)(travelled - ref_travel));
						} else {
							ref_mm = side_mm;
							ref_travel = travelled;
						}
					}
				}
				last_side_mm = side_mm;
				/* Once established, track this side surface rather than accepting
				   distant background anywhere inside the broad acquisition gate. */
				if (watch.seen && side_mm + S5_SIDE_ECHO_ALLOWANCE_MM < side_limit)
					side_limit = side_mm + S5_SIDE_ECHO_ALLOWANCE_MM;
				watch.clear_run = 0;
				clear_start_mm = -1;
				watch.bad_run = 0;
			} else if (ss.us_result == US_NO_ECHO
				   || (ss.us_result == US_OK
				       && side_mm > side_limit)) {
				watch.bad_run = 0;
				if (watch.seen) {
					if (watch.clear_run == 0U) clear_start_mm = travelled;
					if (watch.clear_run < S5_SIDE_CLEAR_CONFIRM) watch.clear_run++;
				} else {
					watch.seen_run = 0;
				}
			} else {
				watch.clear_run = 0;
				if (!watch.seen) watch.seen_run = 0;
				if (watch.bad_run < S5_SIDE_BAD_MAX) watch.bad_run++;
			}
		}

		if (heading_fix > 0) {
			W remaining = AV_SIDE_PASS_MAX_MM - travelled;
			UW r;

			watch.corrected = TRUE;
			motion_stop(MSRC_AVOID);
			if (motion_wait_result(cmd, 1000, &res) != E_OK
			    || (mres_status(res) != MRES_ABORTED
				&& mres_status(res) != MRES_DONE)) {
				fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
				return EXEC_FAILED;
			}
			tm_printf((UB*)"avoid: turning %d deg away to run parallel again\n",
				  away * heading_fix);
			r = move(MCMD_TURN, away * heading_fix);
			if (r != MRES_DONE) {
				if (sys_mode() != VM_OBSTACLE) return EXEC_ABANDONED;
				if (tk_wai_flg(flg_avoid, AVEV_EMERGENCY, TWF_ORW | TWF_BITCLR,
					       &ptn, TMO_POL) == E_OK) return EXEC_EMERGENCY;
				fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
				return EXEC_FAILED;
			}
			/* A pivot leaves the mean wheel odometry unchanged, so the
			   pass distance continues from 'travelled'. */
			if (remaining < 1) remaining = 1;
			if (motion_command(MCMD_MOVE, MSRC_AVOID, (INT)remaining,
					   S5_AVOID_SPEED_LVL, &cmd) != E_OK) {
				fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
				return EXEC_FAILED;
			}
			/* New heading: track the side surface from the latest range. */
			closest_side_mm = last_side_mm;
			farthest_side_mm = last_side_mm;
			side_limit = last_side_mm + S5_SIDE_ECHO_ALLOWANCE_MM;
			watch.clear_run = 0;
			clear_start_mm = -1;
			heading_fix = 0;
			continue;
		}
		if (!watch.seen && travelled > acquire_limit) {
			motion_stop(MSRC_AVOID);
			tm_printf((UB*)"avoid: side obstacle never confirmed after %d mm (bound %d); stopping instead of driving full pass limit\n",
				  (INT)travelled, (INT)acquire_limit);
			fail(AVF_SENSOR, SYS_FLT_AVOID_FAILED);
			return EXEC_FAILED;
		}
		if (watch.bad_run >= S5_SIDE_BAD_MAX) {
			motion_stop(MSRC_AVOID);
			tm_printf((UB*)"avoid: side ultrasonic unusable during pass\n");
			fail(AVF_SENSOR, SYS_FLT_AVOID_FAILED);
			return EXEC_FAILED;
		}
		if (watch.seen && watch.clear_run >= S5_SIDE_CLEAR_CONFIRM) {
			W clear_travel = (clear_start_mm >= 0)
				       ? travelled - clear_start_mm : 0;
			W edge_ahead = (W)(((D)farthest_side_mm
						 * op_cos_q15(S5_SIDE_LOOK_DEG)) / 32768)
				       - clear_travel + S5_FAR_EDGE_ALLOWANCE_MM;
			W measured_gap = (W)(((D)closest_side_mm * side_sin) / 32768)
					 - AV_ROBOT_WIDTH_MM / 2
					 - S5_SIDE_GAP_MEAS_MARGIN_MM;
			W body_path;
			W return_move;

			if (edge_ahead < 0) edge_ahead = 0;
			if (measured_gap < 0) measured_gap = 0;
			body_path = av_return_approach_mm(plan->return_turn_deg, measured_gap);
			return_move = av_post_edge_advance_mm(body_path, edge_ahead);
			if (return_move > MCMD_MOVE_MAX_MM) return_move = MCMD_MOVE_MAX_MM;
			motion_stop(MSRC_AVOID);
			if (motion_wait_result(cmd, 1000, &res) != E_OK
			    || (mres_status(res) != MRES_ABORTED
				&& mres_status(res) != MRES_DONE)) {
				fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
				return EXEC_FAILED;
			}
			depth = plan->forward_before_pass_mm + travelled
			      - (plan->front_gap_mm + AV_SENSOR_BEHIND_FRONT_MM);
			if (depth < 0) depth = 0;
			tm_printf((UB*)"avoid: far edge found after %d mm parallel travel; "
				  "estimated obstacle length %d mm; measured side gap %d mm; "
				  "edge allowance %d mm, body minimum %d mm; advance %d mm, "
				  "then return at %d deg\n",
				  (INT)travelled, (INT)depth, (INT)measured_gap,
				  (INT)edge_ahead, (INT)body_path, (INT)return_move,
				  (INT)plan->return_turn_deg);
			request_scan(SREQ_MONITOR, 0, 0, 0);
			(void)tk_dly_tsk(S5_SIDE_CENTER_MS);
			/* A fully cleared body can require no extra straight travel.
			   MOVE 0 is rejected by Buddy 2; proceed directly to the return. */
			res = (return_move > 0) ? move(MCMD_MOVE, return_move) : MRES_DONE;
			if (res == MRES_DONE) {
				return EXEC_DONE;
			}
			if (tk_wai_flg(flg_avoid, AVEV_EMERGENCY,
				       TWF_ORW | TWF_BITCLR, &ptn, TMO_POL) == E_OK) {
				return EXEC_EMERGENCY;
			}
			fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
			return EXEC_FAILED;
		}

		res = motion_last_result(MSRC_AVOID);
		if ((res & MRES_VALID) != 0U && mres_seq(res) == mcmd_seq(cmd)) {
			if (mres_status(res) == MRES_DONE) {
				tm_printf((UB*)"avoid: no far edge within %d mm pass limit\n",
					  AV_SIDE_PASS_MAX_MM);
				fail(AVF_CLEARANCE, SYS_FLT_OBSTACLE_UNAVOIDABLE);
			} else {
				fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
			}
			return EXEC_FAILED;
		}
	}
}

LOCAL INT execute_plan(const AV_PLAN *plan, const S5_PROFILE *profile)
{
	UINT	i, ptn;
	INT	s = (plan->side == AV_SIDE_RIGHT) ? 1 : -1;

	ast.side = plan->side;
	set_state(AVST_EXECUTING);
	request_scan(SREQ_MONITOR, 0, 0, 0);		/* emergency watch while moving */

	for (i = 0; i < plan->n; i++) {
		const AV_STEP	*st = &plan->step[i];
		UW		r;
		UB		*step_name = (st->kind == AVS_TURN) ? (UB*)"turn"
				    : (st->kind == AVS_MOVE) ? (UB*)"move"
				    : (st->kind == AVS_PASS) ? (UB*)"pass"
				    : (UB*)"search";

		ast.step = (UB)i;
		tm_printf((UB*)"avoid: step %d/%d %s %d\n", (INT)(i + 1U),
			  (INT)plan->n, step_name, (INT)st->arg);
		if (sys_mode() != VM_OBSTACLE) {
			return EXEC_ABANDONED;		/* Safe Stop or operator stop */
		}
		if (expired()) {
			fail(AVF_TIMEOUT, SYS_FLT_RECOVERY_TIMEOUT);
			return EXEC_FAILED;
		}
		if (st->kind == AVS_SEARCH) {
			BOOL	found;

			set_state(AVST_SEARCHING);
			found = search_line(st->arg);
			if (!found && ast.retries < S5_SEARCH_RETRIES
			    && sys_mode() == VM_OBSTACLE) {
				/* retry once with a steeper approach, shorter run */
				ast.retries++;
				set_state(AVST_SEARCHING);
				if (move(MCMD_TURN, -s * S5_SEARCH_RETRY_DEG) == MRES_DONE) {
					found = search_line(st->arg / 2);
				}
			}
			if (!found) {
				if (sys_mode() != VM_OBSTACLE) {
					return EXEC_ABANDONED;
				}
				fail(AVF_LINE, SYS_FLT_RECOVERY_TIMEOUT);
				return EXEC_FAILED;
			}

			/* The IR board is ahead of the wheel axle.  If the final turn
			   starts when the front sensor first touches the line, the body is
			   still offset.  Advance the calibrated sensor-to-axle distance so
			   the axle reaches the line before aligning with it. */
			tm_printf((UB*)"avoid: centre sensor found line; advancing %d mm "
				  "to place wheel axle on line\n", AV_LINE_SENSOR_AHEAD_MM);
			r = move(MCMD_MOVE, AV_LINE_SENSOR_AHEAD_MM);
			if (r != MRES_DONE) {
				if (sys_mode() != VM_OBSTACLE) {
					return EXEC_ABANDONED;
				}
				fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
				return EXEC_FAILED;
			}
			set_state(AVST_ALIGNING);	/* REACQUIRE LINE */
			if (!align_line(s)) {
				if (sys_mode() != VM_OBSTACLE) return EXEC_ABANDONED;
				if (sys_fault_get() & (SYS_FLT_MOTOR_STALL | SYS_FLT_ENCODER))
					fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
				else fail(AVF_LINE, SYS_FLT_RECOVERY_TIMEOUT);
				return EXEC_FAILED;
			}
			/* Sensor alignment replaces the final planned encoder turn. */
			if (i + 1U < plan->n && plan->step[i + 1U].kind == AVS_TURN) i++;
			continue;
		}
		if (st->kind == AVS_PASS) {
			INT pass_result = pass_obstacle(plan, profile);

			if (pass_result != EXEC_DONE) {
				return pass_result;
			}
			continue;
		}
		r = move((st->kind == AVS_TURN) ? MCMD_TURN : MCMD_MOVE, st->arg);
		if (r == MRES_DONE) {
			continue;
		}
		if (sys_mode() != VM_OBSTACLE) {
			return EXEC_ABANDONED;
		}
		if (tk_wai_flg(flg_avoid, AVEV_EMERGENCY, TWF_ORW | TWF_BITCLR, &ptn,
			       TMO_POL) == E_OK) {
			return EXEC_EMERGENCY;
		}
		fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
		return EXEC_FAILED;
	}
	return EXEC_DONE;
}

LOCAL void handle_obstacle(void)
{
	S5_PROFILE	prof;
	AV_PLAN		plan;
	BOOL		cleared;
	UINT		attempt;
	INT		pr;
	W		reverse_mm;
	BOOL		reverse_used = FALSE;
	UW		t0;
	UW		mode = sys_mode();

	if (mode != VM_NORMAL && mode != VM_NAV_ACTION) {
		return;					/* not navigating: ignore */
	}
	deadline_start = app_now_ms();
	ast.retries = 0;
	ast.side = AV_SIDE_NONE;
	ast.step = 0;
	set_state(AVST_STOPPING);
	sys_post_event(SYS_EV_OBSTACLE);		/* Buddy 2: Obstacle Handling + stop */

	t0 = app_now_ms();
	while (sys_mode() != VM_OBSTACLE) {
		if ((UW)(app_now_ms() - t0) > 500U || sys_mode() == VM_SAFE_STOP) {
			set_state(AVST_IDLE);		/* Buddy 2 did not hand over */
			return;
		}
		(void)tk_dly_tsk(S3_PERIOD_MS);
	}

	/* first attempt + bounded re-plans after an emergency stop */
	for (attempt = 0; attempt <= S5_REPLANS; attempt++) {
		if (!wait_stopped()) {
			fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
			return;
		}
		for (;;) {
			if (!profile_obstacle(&prof, &cleared)) {
				if (cleared) {
					finish(AVO_CLEARED);	/* path clear again: resume */
				}
				return;
			}
			set_state(AVST_PLANNING);
			pr = av_plan(&prof, &plan);
			reverse_mm = av_reverse_needed_mm(&prof);
			if (pr == AVP_NO_CLEARANCE && !reverse_used && reverse_mm > 0) {
				tm_printf((UB*)"avoid: front gap %d mm, calculated need %d mm; "
					  "reversing only %d mm at creep, then rescanning\n",
					  (INT)plan.front_gap_mm, (INT)plan.required_front_mm,
					  (INT)reverse_mm);
				if (move_at_speed(MCMD_MOVE, -(INT)reverse_mm,
						  S5_REVERSE_SPEED_LVL) != MRES_DONE) {
					fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
					return;
				}
				reverse_used = TRUE;
				if (!wait_stopped()) {
					fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);
					return;
				}
				tm_printf((UB*)"avoid: reverse complete; starting one full rescan\n");
				continue;		/* repeat the full fine scan */
			}
			break;
		}
		if (pr == AVP_NO_CLEARANCE) {
			tm_printf((UB*)"avoid: no safe path (have %d mm bumper gap; "
				  "need %d mm side, %d mm front for calculated turn)\n",
				  (INT)plan.front_gap_mm, (INT)plan.required_mm,
				  (INT)((plan.required_front_mm > 0)
					? plan.required_front_mm : AV_MIN_FRONT_CLEARANCE_MM));
			fail(AVF_CLEARANCE, SYS_FLT_OBSTACLE_UNAVOIDABLE);
			return;
		}
		if (pr != AVP_OK) {
			fail(AVF_PROFILE, SYS_FLT_PROFILE_INVALID);
			return;
		}
		tm_printf((UB*)"avoid: plan %s, edge %d + half robot %d + gap %d "
			  "+ allowance %d "
			  "= centre offset %d mm; approach %d mm, turn %d deg, %d steps\n",
			  (plan.side == AV_SIDE_RIGHT) ? "right" : "left",
			  (INT)((plan.side == AV_SIDE_RIGHT) ? prof.right_mm : -prof.left_mm),
			  AV_ROBOT_WIDTH_MM / 2, AV_SIDE_GAP_MM, AV_EDGE_UNCERTAINTY_MM,
			  (INT)plan.offset_mm, (INT)plan.approach_mm,
			  (INT)plan.turn_deg, (INT)plan.n);
		switch (execute_plan(&plan, &prof)) {
		case EXEC_DONE:
			finish(AVO_SUCCESS);
			return;
		case EXEC_EMERGENCY:
			break;				/* re-scan from where we stand */
		case EXEC_ABANDONED:
			set_state(AVST_IDLE);
			request_scan(SREQ_MONITOR, 0, 0, 0);
			return;
		default:
			return;				/* failure already reported */
		}
	}
	fail(AVF_MOTION, SYS_FLT_AVOID_FAILED);		/* re-plans exhausted */
}

EXPORT void avoid_task(INT stacd, void *exinf)
{
	UINT	ptn;

	(void)stacd;
	(void)exinf;
	ast.state = AVST_IDLE;
	ast.side = AV_SIDE_NONE;
	ast.step = 0;
	ast.retries = 0;
	ast.outcome = AVO_NONE;
	ast.count = 0;
	ast.fail = AVF_NONE;
	set_state(AVST_IDLE);
	request_scan(SREQ_MONITOR, 0, 0, 0);
	sys_task_ready(SYS_TASK_AVOID);

	for (;;) {
		if (tk_wai_flg(flg_avoid, AVEV_DETECTED, TWF_ORW | TWF_BITCLR, &ptn,
			       TMO_FEVR) < E_OK) {
			(void)tk_dly_tsk(100);
			continue;
		}
		handle_obstacle();
		(void)tk_clr_flg(flg_avoid, 0);	/* drop detections from during handling */
	}
}
