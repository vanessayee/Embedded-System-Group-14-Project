/*
 *	s2_unit_tests.c
 *	Unit tests of subsystem 2's pure logic (motion command format,
 *	arbitration, PID, move/turn execution, stall detection and bounded
 *	escalation, vehicle mode FSM, fault policy).
 *
 *	Built into the on-target test image (make TEST=1, called from
 *	subsystem2_test.c) and into the host harness (testing/host/), so the
 *	same checks run in both places.  Test IDs: S2-Uxx.
 *	A simple first-order wheel model stands in for the motors.
 */

#include <tk/typedef.h>

#if APP_TEST || HOST_TEST

#include "../test_report.h"
#include "motion_msg.h"
#include "motion_ctrl.h"
#include "motion_cfg.h"
#include "vehicle_fsm.h"
#include "sys_defs.h"
#include "pid.h"
#include "s2_tests.h"

/* ------------------------------------------------------------------ */
/*  Simulated plant: v_ss = (duty - FF_OFFSET) / 2 mm/s per permille,   */
/*  first-order lag, optional stuck wheel, encoder quantisation.        */

EXPORT void plant_init(PLANT *p)
{
	p->v_l = 0;
	p->v_r = 0;
	p->d_l_um = 0;
	p->d_r_um = 0;
	p->stuck_l = FALSE;
	p->stuck_r = FALSE;
}

LOCAL W wheel_ss(W duty)
{
	W	a = (duty < 0) ? -duty : duty;
	W	v = (a > FF_OFFSET_PM) ? (a - FF_OFFSET_PM) * 256 / FF_GAIN_Q8 : 0;

	return (duty < 0) ? -v : v;
}

EXPORT void plant_step(PLANT *p, W duty_l, W duty_r, UW dt_ms)
{
	W	ssl = p->stuck_l ? 0 : wheel_ss(duty_l);
	W	ssr = p->stuck_r ? 0 : wheel_ss(duty_r);
	W	dl = (ssl - p->v_l) * (W)dt_ms / (60 + (W)dt_ms);
	W	dr = (ssr - p->v_r) * (W)dt_ms / (60 + (W)dt_ms);

	/* Integer division otherwise leaves an unpowered wheel drifting at
	   +/-3 mm/s forever, preventing a real stationary alignment check. */
	if (dt_ms != 0U && ssl == 0 && dl == 0 && p->v_l != 0) dl = (p->v_l > 0) ? -1 : 1;
	if (dt_ms != 0U && ssr == 0 && dr == 0 && p->v_r != 0) dr = (p->v_r > 0) ? -1 : 1;
	p->v_l += dl;
	p->v_r += dr;
	if (p->stuck_l) {
		p->v_l = 0;
	}
	if (p->stuck_r) {
		p->v_r = 0;
	}
	p->d_l_um += p->v_l * (W)dt_ms;		/* mm/s * ms = um */
	p->d_r_um += p->v_r * (W)dt_ms;
}

EXPORT void plant_input(const PLANT *p, MC_IN *in, UW vmode)
{
	/* measured distance is quantised to whole encoder ticks */
	in->dist_l_um = (p->d_l_um / (W)ENC_UM_PER_TICK) * (W)ENC_UM_PER_TICK;
	in->dist_r_um = (p->d_r_um / (W)ENC_UM_PER_TICK) * (W)ENC_UM_PER_TICK;
	in->speed_l_mms = p->v_l;
	in->speed_r_mms = p->v_r;
	in->enc_valid = TRUE;
	in->vmode = vmode;
	in->armed = TRUE;
	in->steer = 0;
	in->steer_fresh = FALSE;
	in->cap_mms = 0;
	in->line_lost_cap = FALSE;
	in->line_seen = 0;
	in->line_seen_fresh = FALSE;
}

/* Run the controller for up to max_ms; returns elapsed ms when a result
   for 'src' arrives (its word in *res), or max_ms */
EXPORT UW run_until_result(MOTION_CTRL *mc, PLANT *p, UW vmode, UW src,
			   UW max_ms, UW *res)
{
	MC_IN	in;
	MC_OUT	out;
	UW	t;

	*res = 0;
	for (t = 0; t < max_ms; t += MOTION_PERIOD_MS) {
		plant_input(p, &in, vmode);
		mc_out_clear(&out);
		mc_step(mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(p, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
		if ((out.result_mask & (1UL << src)) != 0U) {
			*res = out.result[src];
			return t;
		}
	}
	return max_ms;
}

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

/* Barcode TURN of 'deg' with simulated IR sensors. Heading is in 0.1 deg,
   derived from the configured wheel-centre spacing, magnitude only.
   While driving to the turn point both sensors are on the junction line
   until 'line_mm' of travel (0 = never).  While turning the right sensor
   sees black for headings in [r0, r1], the centre for [c0, c1]. */
typedef struct {
	UW	res;
	W	h;		/* heading after settling, 0.1 deg */
	W	adv_um;		/* mean travel before the turn began */
	W	tl_um;		/* wheel travel during the turn */
	W	tr_um;
	UW	elapsed_ms;
	UW	align_drive_run;
} SEEK_RUN;

LOCAL void seek_turn(PLANT *p, INT deg, BOOL ir, W line_mm, W r0, W r1,
		     W c0, W c1, SEEK_RUN *s)
{
	MOTION_CTRL	mc;
	MC_IN		in;
	MC_OUT		out;
	UW		t, settle = 0;
	UW		drive_run = 0;
	BOOL		turning = FALSE;
	W		l0 = 0, r0w = 0;

	plant_init(p);
	mc_init(&mc);
	mc_out_clear(&out);
	s->res = 0;
	s->align_drive_run = 0;
	s->adv_um = 0;
	(void)mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_BARCODE, 1, 0, 2, deg),
			VM_NORMAL, 0, &out);
	for (t = 0; t < 20000U && settle < 25U; t += MOTION_PERIOD_MS) {
		W	h = (W)((D)iabs(p->d_l_um - p->d_r_um) * 57296
				/ ((D)TRACK_WIDTH_MM * 100000));
		W	trav = (p->d_l_um + p->d_r_um) / 2;
		BOOL	jl = !turning && trav < line_mm * 1000;

		plant_input(p, &in, VM_NAV_ACTION);
		in.line_seen = lseen_pack(t / MOTION_PERIOD_MS, FALSE,
			jl || (turning && h >= c0 && h <= c1),
			jl || (turning && h >= r0 && h <= r1));
		in.line_seen_fresh = ir;
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		if (mc.turn_align && out.duty_l_pm != 0 && out.duty_r_pm != 0) {
			drive_run++;
			if (drive_run > s->align_drive_run) s->align_drive_run = drive_run;
		} else {
			drive_run = 0;
		}
		if (!turning && !mc.advancing) {
			turning = TRUE;		/* turn point reached */
			s->adv_um = trav;
			l0 = in.dist_l_um;
			r0w = in.dist_r_um;
		}
		plant_step(p, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
		if ((out.result_mask & (1UL << MSRC_BARCODE)) != 0U) {
			s->res = out.result[MSRC_BARCODE];
		}
		if (s->res != 0U) {
			settle++;
		}
	}
	s->h = (W)((D)iabs(p->d_l_um - p->d_r_um) * 57296
			/ ((D)TRACK_WIDTH_MM * 100000));
	s->tl_um = p->d_l_um - l0;
	s->tr_um = p->d_r_um - r0w;
	s->elapsed_ms = t;
}

/* ------------------------------------------------------------------ */

LOCAL INT test_formats(void)
{
	UW		w, r, s, c;
	MOTION_STATUS	a, b;
	UW		pw[2];

	TEST_INIT();

	w = mcmd_pack(MCMD_MOVE, MSRC_AVOID, 15, 7, 15, -2048);
	TEST_CHECK("S2-U01", mcmd_type(w) == MCMD_MOVE && mcmd_src(w) == MSRC_AVOID
		   && mcmd_seq(w) == 15 && mcmd_epoch(w) == 7 && mcmd_speed(w) == 15
		   && mcmd_arg(w) == -2048 && (w & MCMD_RSV_MASK) == 0,
		   "command word round trip at field limits (arg -2048)");
	w = mcmd_pack(MCMD_TURN, MSRC_BARCODE, 0, 0, 0, 2047);
	TEST_CHECK("S2-U01b", mcmd_arg(w) == 2047 && mcmd_type(w) == MCMD_TURN,
		   "command word arg +2047");

	TEST_CHECK("S2-U02", mcmd_validate(w | 0x10000000UL) == MREJ_FORMAT
		   && mcmd_validate(mcmd_pack(6, MSRC_AVOID, 0, 0, 0, 0)) == MREJ_FORMAT
		   && mcmd_validate(mcmd_pack(MCMD_MOVE, MSRC_LINE, 0, 0, 0, 10)) == MREJ_SOURCE
		   && mcmd_validate(mcmd_pack(MCMD_MOVE, MSRC_SUPER, 0, 0, 0, 10)) == MREJ_SOURCE
		   && mcmd_validate(mcmd_pack(MCMD_SET_SPEED, MSRC_BARCODE, 0, 0, 0, 200)) == MREJ_SOURCE,
		   "reserved bits, unknown type and source rules rejected");
	TEST_CHECK("S2-U03", mcmd_validate(mcmd_pack(MCMD_MOVE, MSRC_AVOID, 0, 0, 0, 0)) == MREJ_RANGE
		   && mcmd_validate(mcmd_pack(MCMD_MOVE, MSRC_AVOID, 0, 0, 0, 2001)) == MREJ_RANGE
		   && mcmd_validate(mcmd_pack(MCMD_MOVE, MSRC_AVOID, 0, 0, 0, -2000)) == MREJ_NONE
		   && mcmd_validate(mcmd_pack(MCMD_TURN, MSRC_AVOID, 0, 0, 0, 361)) == MREJ_RANGE
		   && mcmd_validate(mcmd_pack(MCMD_TURN, MSRC_AVOID, 0, 0, 0, -360)) == MREJ_NONE
		   && mcmd_validate(mcmd_pack(MCMD_SET_SPEED, MSRC_SUPER, 0, 0, 0, 49)) == MREJ_RANGE
		   && mcmd_validate(mcmd_pack(MCMD_SET_SPEED, MSRC_SUPER, 0, 0, 0, 401)) == MREJ_RANGE
		   && mcmd_validate(mcmd_pack(MCMD_SET_SPEED, MSRC_SUPER, 0, 0, 0, 400)) == MREJ_NONE
		   && mcmd_validate(mcmd_pack(MCMD_STOP, MSRC_SUPER, 0, 0, 0, 1)) == MREJ_RANGE,
		   "argument ranges enforced at the boundaries");

	r = mres_pack(mcmd_pack(MCMD_TURN, MSRC_AVOID, 9, 0, 0, 90), MRES_REJECTED, MREJ_QUEUE);
	s = steer_pack(15, -100, 15);
	c = scap_pack(3, 2000);
	TEST_CHECK("S2-U04", mres_seq(r) == 9 && mres_status(r) == MRES_REJECTED
		   && mres_type(r) == MCMD_TURN && mres_reason(r) == MREJ_QUEUE
		   && (r & MRES_VALID) != 0
		   && steer_value(s) == -63 && steer_seq(s) == 15 && steer_speed(s) == 15
		   && steer_value(steer_pack(0, 63, 0)) == 63
		   && scap_value(c) == 1023 && scap_seq(c) == 3,
		   "result, steering (clamped -63) and speed-cap words");

	a.speed_l_mms = -1023;
	a.speed_r_mms = MSTAT_SPEED_INVALID;
	a.mstate = MS_FAULT;
	a.source = MSRC_SUPER;
	a.cmd_active = 1;
	a.stall_l = 1;
	a.stall_r = 0;
	a.enc_ok = 1;
	a.odometer_cm = 65535;
	a.heading_deg = 358;
	a.cruise_mms = 630;
	mstat_pack(&a, pw);
	mstat_unpack(pw, &b);
	TEST_CHECK("S2-U05", b.speed_l_mms == -1023 && b.speed_r_mms == MSTAT_SPEED_INVALID
		   && b.mstate == MS_FAULT && b.source == MSRC_SUPER && b.cmd_active == 1
		   && b.stall_l == 1 && b.stall_r == 0 && b.enc_ok == 1
		   && b.odometer_cm == 65535 && b.heading_deg == 358 && b.cruise_mms == 630,
		   "motion status 2-word pack/unpack at limits");
	TEST_SUMMARY("s2 formats");
	return test_fail_;
}

LOCAL INT test_pid(void)
{
	PID	p;
	W	out, i_before;
	INT	k;

	TEST_INIT();
	pid_init(&p, 512, 64, 0, 2000, -1000, 1000);
	out = pid_update(&p, 100, 100, 320);
	TEST_CHECK("S2-U06", out == 320, "zero error: output = feed-forward");

	pid_reset(&p);
	for (k = 0; k < 200; k++) {
		out = pid_update(&p, 400, 0, 0);	/* saturates high */
	}
	i_before = p.i_acc;
	out = pid_update(&p, 400, 0, 0);
	TEST_CHECK("S2-U07", out == 1000 && p.i_acc == i_before && p.i_acc <= 2000,
		   "saturated output: integrator frozen (anti-windup)");
	TEST_SUMMARY("s2 pid");
	return test_fail_;
}

LOCAL INT test_execution(void)
{
	MOTION_CTRL	mc;
	PLANT		pl;
	MC_OUT		out;
	MC_IN		in;
	UW		res, t, cmd;
	W		arc = mc_turn_arc_um(90);
	W		creep;
	W		min_v;
	UINT		k;

	TEST_INIT();
	TEST_CHECK("S2-U08", iabs(arc - (W)((D)TRACK_WIDTH_MM * 3141593 / 4000)) <= 1
		   && mc_turn_arc_um(-90) == arc,
		   "turn arc for 90 deg = pi*track/4 using the measured wheel spacing");
	plant_init(&pl);
	pl.v_l = 3;
	pl.v_r = -3;
	for (k = 0; k < 25U; k++) plant_step(&pl, 0, 0, MOTION_PERIOD_MS);
	TEST_CHECK("S2-U08a", pl.v_l == 0 && pl.v_r == 0,
		   "unpowered simulated wheels decay to rest without integer-rounding drift");

	/* MOVE 500 mm in Obstacle Handling (AVOID source), at an explicit
	   200 mm/s (level 8) so the check does not depend on the cruise
	   default: 500 mm at the 20 mm/s cruise alone would take 25 s */
	mc_init(&mc);
	plant_init(&pl);
	mc_out_clear(&out);
	cmd = mcmd_pack(MCMD_MOVE, MSRC_AVOID, 1, 2, 8, 500);
	TEST_CHECK("S2-U09a", mc_submit(&mc, cmd, VM_OBSTACLE, 2, &out) == MREJ_NONE,
		   "MOVE accepted in Obstacle Handling");
	t = run_until_result(&mc, &pl, VM_OBSTACLE, MSRC_AVOID, 20000, &res);
	TEST_CHECK("S2-U09", t < 20000 && mres_status(res) == MRES_DONE && mres_seq(res) == 1
		   && iabs((pl.d_l_um + pl.d_r_um) / 2 - 500000) < 25000
		   && iabs(pl.d_l_um - pl.d_r_um) < 15000,
		   "MOVE 500 mm: DONE, within 25 mm, wheels within 15 mm");

	/* Reverse remains globally disabled, but Buddy 5 may make one bounded
	   negative MOVE after entering Obstacle Handling. */
	mc_init(&mc);
	plant_init(&pl);
	mc_out_clear(&out);
	cmd = mcmd_pack(MCMD_MOVE, MSRC_AVOID, 2, 2, 1, -120);
	TEST_CHECK("S2-U09b", mc_submit(&mc, cmd, VM_OBSTACLE, 2, &out) == MREJ_NONE,
		   "bounded AVOID reverse accepted only in Obstacle Handling");
	t = run_until_result(&mc, &pl, VM_OBSTACLE, MSRC_AVOID, 15000, &res);
	TEST_CHECK("S2-U09c", t < 15000 && mres_status(res) == MRES_DONE
		   && (pl.d_l_um + pl.d_r_um) / 2 < -90000,
		   "negative MOVE drives both wheels backward and completes");

	/* TURN +90 */
	plant_init(&pl);
	mc_init(&mc);
	mc_out_clear(&out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_AVOID, 2, 2, 0, 90), VM_OBSTACLE, 2, &out);
	t = run_until_result(&mc, &pl, VM_OBSTACLE, MSRC_AVOID, 20000, &res);
	TEST_CHECK("S2-U10", t < 20000 && mres_status(res) == MRES_DONE
		   && pl.d_l_um > 0 && pl.d_r_um < 0
		   && iabs((pl.d_l_um - pl.d_r_um) / 2 - arc) < 12000,
		   "TURN +90: left forward, right backward, arc within 12 mm");

	/* Barcode RIGHT at a junction.  Drive on: the right sensor leaves
	   the junction line after 15 mm (reference point), then the advance
	   minus half a line width.  Turn: the line passes the right sensor at
	   70-76 deg and the centre at 85-90 deg (wheel slip: before the
	   90 deg the encoders expect).  Stop when the centre sees tape. */
	{
		SEEK_RUN	s;
		W		right_advance;
		W		right_outer, right_inner, right_heading;
		W		adv = 15000 + (TURN_ADVANCE_MM
					       - TURN_LINE_WIDTH_MM / 2) * 1000;

		seek_turn(&pl, 90, TRUE, 15, 840, 890, 980, 1030, &s);
		right_advance = s.adv_um;
		right_outer = s.tl_um;
		right_inner = s.tr_um;
		right_heading = s.h;
		TEST_CHECK("S2-U10b", mres_status(s.res) == MRES_DONE
			   && iabs(s.adv_um - adv) <= 3000
			   && s.h > 890 && s.h < 980
			   && s.tl_um > 0 && s.tr_um < 0
			   && iabs(s.tl_um + s.tr_um) < 6000,
			   "barcode RIGHT: equal wheel arcs, right-leading contact then confirmed white gap");
		adv = 15000 + (TURN_ADVANCE_MM - TURN_LINE_WIDTH_MM / 2) * 1000;
		/* Left has the opposite order: centre first, then GP2. The
		   successful stop is in the white gap, after centre leaves. */
		seek_turn(&pl, -90, TRUE, 15, 980, 1030, 840, 890, &s);
		TEST_CHECK("S2-U10c", mres_status(s.res) == MRES_DONE
			   && iabs(s.adv_um - adv) <= 3000
		   && s.h > 890 && s.h < 980
		   && s.elapsed_ms < 15000U
			   && s.tr_um > 0 && s.tl_um < 0
		   && iabs(s.tl_um + s.tr_um) < 6000,
			   "barcode LEFT: cross the centre black edge and stop in the centre-right white gap");
		TEST_CHECK("S2-U10s", s.align_drive_run >= 3U,
			   "final alignment sustains motor drive instead of ineffective one-tick pulses");
		TEST_CHECK("S2-U10o", iabs(s.adv_um - right_advance) <= 3000,
			   "left and right physically advance the same distance before pivoting");
		TEST_CHECK("S2-U10p", iabs(s.tr_um - right_outer) < 3000
			   && iabs(s.tl_um - right_inner) < 3000 && iabs(s.h - right_heading) < 20,
			   "mirrored sensor order produces mirrored left/right wheel paths and equal turn angle");
		/* Hardware trace: outgoing centre contact happened before the
		   former 70-degree window, followed by white and then right black. */
		seek_turn(&pl, -90, TRUE, 15, 800, 850, 600, 650, &s);
		TEST_CHECK("S2-U10q", mres_status(s.res) == MRES_DONE
			   && s.h > 650 && s.h < 800,
			   "LEFT outgoing line before 70 encoder degrees is accepted after clearing junction");
		seek_turn(&pl, 90, TRUE, 15, 600, 650, 800, 850, &s);
		TEST_CHECK("S2-U10r", mres_status(s.res) == MRES_DONE
			   && s.h > 650 && s.h < 800,
			   "RIGHT uses the same early contact acquisition with mirrored sensors");
		seek_turn(&pl, -90, TRUE, 15, 840, 950, 0, 0, &s);
		TEST_CHECK("S2-U10j", mres_status(s.res) == MRES_FAULT
			   && s.h > 1020 && s.h < 1110,
			   "barcode LEFT: GP2 alone cannot falsely complete the turn");
		seek_turn(&pl, -90, TRUE, 15, 0, 0, 0, 0, &s);
		TEST_CHECK("S2-U10h", mres_status(s.res) == MRES_FAULT
			   && s.h > 1020 && s.h < 1110,
			   "barcode LEFT: missing tape faults at the angle limit");
		seek_turn(&pl, -90, TRUE, 15, 1040, 1100, 950, 980, &s);
		TEST_CHECK("S2-U10k", mres_status(s.res) == MRES_DONE
		   && s.h > 980 && s.h < 1040,
			   "barcode LEFT: late centre contact must release before turn completion");
		seek_turn(&pl, -90, TRUE, 15, 0, 0, 800, 1100, &s);
		TEST_CHECK("S2-U10l", mres_status(s.res) == MRES_FAULT,
			   "barcode LEFT: centre remaining black cannot report corridor alignment");
		seek_turn(&pl, -90, TRUE, 15, 0, 0, 300, 400, &s);
		TEST_CHECK("S2-U10m", mres_status(s.res) == MRES_FAULT,
			   "barcode LEFT: early crossing contact cannot validate later white space");
		seek_turn(&pl, -90, TRUE, 15, 800, 900, 800, 900, &s);
		TEST_CHECK("S2-U10n", mres_status(s.res) == MRES_FAULT,
			   "barcode LEFT: both-black junction cannot validate a white corridor");
		seek_turn(&pl, 90, TRUE, 15, 750, 790, 750, 790, &s);
		TEST_CHECK("S2-U10d", mres_status(s.res) == MRES_FAULT
			   && s.h > 1020 && s.h < 1110,
			   "both sensors black together do not confirm the right branch");
		seek_turn(&pl, 90, TRUE, 15, 300, 340, 0, 0, &s);
		TEST_CHECK("S2-U10e", mres_status(s.res) == MRES_FAULT
			   && s.h > 1020 && s.h < 1110,
			   "line seen only far from 90 deg: fault at the angle limit");
		seek_turn(&pl, 90, FALSE, 15, 800, 860, 950, 1000, &s);
		TEST_CHECK("S2-U10f", mres_status(s.res) == MRES_FAULT
			   && iabs(s.adv_um - TURN_ADVANCE_MM * 1000) <= 3000
			   && s.h > 870 && s.h < 940,
			   "no fresh IR word: stop at encoder target and report unconfirmed turn");
		seek_turn(&pl, 180, FALSE, 0, 0, 0, 0, 0, &s);
		TEST_CHECK("S2-U10g", mres_status(s.res) == MRES_DONE
			   && s.adv_um < 3000 && s.h > 1750 && s.h < 1850
			   && iabs(s.tl_um + s.tr_um) < 10000,
			   "barcode U-turn: no advance, turns on the spot");
	}

	/* GP2 clears the cross before the barcode task submits the turn. */
	plant_init(&pl);
	mc_init(&mc);
	mc_out_clear(&out);
	plant_input(&pl, &in, VM_NORMAL);
	in.line_seen_fresh = TRUE;
	in.line_seen = lseen_pack(1, FALSE, TRUE, TRUE);
	mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
	in.dist_l_um = in.dist_r_um = 18000;
	in.line_seen = lseen_pack(2, FALSE, FALSE, FALSE);
	mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
	in.dist_l_um = in.dist_r_um = 25000;
	mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
	cmd = mcmd_pack(MCMD_TURN, MSRC_BARCODE, 1, 0, 2, -90);
	res = mc_submit(&mc, cmd, VM_NORMAL, 0, &out);
	in.vmode = VM_NAV_ACTION;
	mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
	TEST_CHECK("S2-U10i", res == MREJ_NONE && mc.adv_ref
		   && mc.advance_um == (TURN_ADVANCE_MM - TURN_LINE_WIDTH_MM / 2 - 7) * 1000,
		   "left turn uses the recorded GP2 exit 7 mm before command start");

	/* NFR12: MOVE followed by a queued TURN keeps at least creep speed */
	plant_init(&pl);
	mc_init(&mc);
	mc_out_clear(&out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 3, 1, 0, 300), VM_OBSTACLE, 1, &out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_AVOID, 4, 1, 0, 90), VM_OBSTACLE, 1, &out);
	creep = mc_creep_speed(&mc);
	min_v = 100000;
	for (k = 0; k < 1000 && mcmd_type(mc.active) == MCMD_MOVE && mc.has_active; k++) {
		plant_input(&pl, &in, VM_OBSTACLE);
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
		if (k > 25 && mc.has_active && mcmd_type(mc.active) == MCMD_MOVE
		    && mc.tgt_l_mms < min_v) {
			min_v = mc.tgt_l_mms;
		}
	}
	TEST_CHECK("S2-U11", mc.has_active && mcmd_type(mc.active) == MCMD_TURN
		   && min_v >= creep - 20,
		   "queued TURN starts straight from creep speed (no full stop)");

	/* Hardware trace: avoidance slowed a pivot to 14 mm/s on one wheel,
	   below the configured controllable floor, while correcting imbalance. */
	{
		INT dir;
		BOOL floor_ok = TRUE;
		for (dir = -1; dir <= 1; dir += 2) {
			mc_init(&mc);
			plant_init(&pl);
			mc_out_clear(&out);
			(void)mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_AVOID, 1, 0, 2, dir * 69),
					VM_OBSTACLE, 0, &out);
			plant_input(&pl, &in, VM_OBSTACLE);
			in.dist_l_um = dir * (mc.target_um - 3000);
			in.dist_r_um = -dir * (mc.target_um - 15000);
			for (k = 0; k < 5U; k++) {
				mc_out_clear(&out);
				mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
			}
			creep = mc_creep_speed(&mc);
			if (!mc.has_active || iabs(mc.tgt_l_mms) < creep
			    || iabs(mc.tgt_r_mms) < creep
			    || mc.tgt_l_mms * dir <= 0 || mc.tgt_r_mms * dir >= 0)
				floor_ok = FALSE;
		}
		TEST_CHECK("S2-U11a", floor_ok,
			   "imbalanced avoidance pivots retain controllable wheel speed in both directions");
	}
	TEST_SUMMARY("s2 execution");
	return test_fail_;
}

LOCAL INT test_arbitration(void)
{
	MOTION_CTRL	mc;
	MC_OUT		out;
	MC_IN		in;
	PLANT		pl;
	UINT		k;

	TEST_INIT();
	mc_init(&mc);
	mc_out_clear(&out);
	TEST_CHECK("S2-U12", mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 1, 0, 0, 100),
				       VM_NORMAL, 0, &out) == MREJ_MODE
		   && mres_status(out.result[MSRC_AVOID]) == MRES_REJECTED,
		   "AVOID command rejected outside Obstacle Handling");
	TEST_CHECK("S2-U13", mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_BARCODE, 1, 0, 0, 90),
				       VM_OBSTACLE, 1, &out) != MREJ_NONE,
		   "BARCODE command rejected in Obstacle Handling");
	TEST_CHECK("S2-U14", mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_BARCODE, 2, 3, 0, 90),
				       VM_NORMAL, 4, &out) == MREJ_STALE,
		   "command from an older mode epoch rejected as stale");
	TEST_CHECK("S2-U15", mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_BARCODE, 3, 0, 0, 100),
				       VM_INIT, 0, &out) == MREJ_MODE
		   && mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 3, 0, 0, 100),
				VM_SAFE_STOP, 0, &out) == MREJ_MODE,
		   "nothing accepted in INIT or SAFE_STOP");

	/* queue depth: 1 active + MC_QUEUE_DEPTH queued, then QUEUE */
	mc_init(&mc);
	for (k = 0; k <= MC_QUEUE_DEPTH; k++) {
		(void)mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, k, 0, 0, 100),
				VM_OBSTACLE, 0, &out);
	}
	TEST_CHECK("S2-U16", mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 9, 0, 0, 100),
				       VM_OBSTACLE, 0, &out) == MREJ_QUEUE,
		   "bounded command queue: overflow rejected (QUEUE)");

	/* supervisory STOP pre-empts an active barcode command */
	mc_init(&mc);
	mc_out_clear(&out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_TURN, MSRC_BARCODE, 5, 0, 0, 90), VM_NORMAL, 0, &out);
	mc_out_clear(&out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_STOP, MSRC_SUPER, 1, 0, 0, 0), VM_NAV_ACTION, 0, &out);
	TEST_CHECK("S2-U17", (out.result_mask & (1UL << MSRC_BARCODE)) != 0
		   && mres_status(out.result[MSRC_BARCODE]) == MRES_ABORTED
		   && out.nav_finished && !mc.has_active,
		   "STOP bypass aborts the running command (ABORTED, nav finished)");
	plant_init(&pl);
	pl.v_l = 200;
	pl.v_r = 200;
	plant_input(&pl, &in, VM_NAV_ACTION);
	mc_out_clear(&out);
	mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
	TEST_CHECK("S2-U18", out.duty_l_pm == 0 && out.duty_r_pm == 0,
		   "stop(): PWM forced to zero in the very next period");
	TEST_SUMMARY("s2 arbitration");
	return test_fail_;
}

LOCAL INT test_faults(void)
{
	MOTION_CTRL	mc;
	MC_OUT		out;
	MC_IN		in;
	PLANT		pl;
	UW		res, t;
	UINT		round;
	BOOL		escalated = FALSE, cleared = FALSE;
	UW		fault_set = 0;

	TEST_INIT();
	mc_init(&mc);
	plant_init(&pl);
	pl.stuck_l = TRUE;
	pl.stuck_r = TRUE;
	mc_out_clear(&out);
	/* explicit 200 mm/s (level 8): at a slow cruise the duty needs
	   longer to pass STALL_DUTY_MIN_PM before the 400 ms window starts */
	(void)mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 1, 0, 8, 500), VM_OBSTACLE, 0, &out);
	t = run_until_result(&mc, &pl, VM_OBSTACLE, MSRC_AVOID, 5000, &res);
	TEST_CHECK("S2-U19", t <= STALL_TIME_MS + 200 && mres_status(res) == MRES_FAULT
		   && mc.state == MS_FAULT && (mc.fault_bits & SYS_FLT_MOTOR_STALL) != 0,
		   "stall (duty, no ticks for 400 ms): FAULT result and MOTOR_STALL");

	/* condition clear for 300 ms -> IDLE, fault bit cleared */
	pl.stuck_l = FALSE;
	pl.stuck_r = FALSE;
	for (t = 0; t < 1000 && mc.state == MS_FAULT; t += MOTION_PERIOD_MS) {
		plant_input(&pl, &in, VM_OBSTACLE);
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		if ((out.flt_clr & SYS_FLT_MOTOR_STALL) != 0) {
			cleared = TRUE;
		}
		if (out.duty_l_pm != 0 || out.duty_r_pm != 0) {
			break;
		}
	}
	TEST_CHECK("S2-U20", mc.state == MS_IDLE && cleared && t >= FAULT_CLEAR_MS - 40,
		   "FAULT -> IDLE only after 300 ms clear; motors off meanwhile");

	/* repeated stalls escalate instead of cycling Fault <-> Idle */
	mc_init(&mc);
	for (round = 0; round < MAX_FAULT_RETRIES + 1U; round++) {
		plant_init(&pl);
		pl.stuck_l = TRUE;
		pl.stuck_r = TRUE;
		mc_out_clear(&out);
		(void)mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, round, 0, 0, 500),
				VM_OBSTACLE, 0, &out);
		for (t = 0; t < 2000; t += MOTION_PERIOD_MS) {
			plant_input(&pl, &in, VM_OBSTACLE);
			mc_out_clear(&out);
			mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
			fault_set |= out.flt_set;
			plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
			if (mc.state == MS_IDLE && t > 500) {
				break;
			}
		}
		if ((fault_set & SYS_FLT_MOTION_ESCALATED) != 0) {
			escalated = TRUE;
			break;
		}
	}
	TEST_CHECK("S2-U21", escalated && mc.fault_retries >= MAX_FAULT_RETRIES,
		   "3rd consecutive stall raises MOTION_ESCALATED (bounded recovery)");

	/* one wheel silent while the other really moves: encoder fault */
	mc_init(&mc);
	plant_init(&pl);
	pl.stuck_r = TRUE;
	mc_out_clear(&out);
	(void)mc_submit(&mc, mcmd_pack(MCMD_MOVE, MSRC_AVOID, 1, 0, 0, 800), VM_OBSTACLE, 0, &out);
	fault_set = 0;
	for (t = 0; t < 4000 && mc.state != MS_FAULT; t += MOTION_PERIOD_MS) {
		plant_input(&pl, &in, VM_OBSTACLE);
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		fault_set |= out.flt_set;
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
	}
	TEST_CHECK("S2-U22", mc.state == MS_FAULT
		   && (fault_set & (SYS_FLT_ENCODER | SYS_FLT_MOTOR_STALL)) != 0,
		   "one wheel without ticks while driven: motion FAULT raised");
	TEST_SUMMARY("s2 faults");
	return test_fail_;
}

LOCAL INT test_steering(void)
{
	MOTION_CTRL	mc;
	MC_OUT		out;
	MC_IN		in;
	PLANT		pl;
	UINT		k;
	W		capped_max = 0;

	TEST_INIT();
	mc_init(&mc);
	plant_init(&pl);
	for (k = 0; k < 50; k++) {
		plant_input(&pl, &in, VM_NORMAL);
		in.steer = steer_pack(k, 20, 15);	/* turn right */
		in.steer_fresh = TRUE;
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
	}
	TEST_CHECK("S2-U23", mc.tgt_l_mms > mc.tgt_r_mms && pl.v_l > pl.v_r
		   && mc.state == MS_STRAIGHT,
		   "positive steer turns right (left wheel faster)");

	for (k = 0; k < 30; k++) {
		plant_input(&pl, &in, VM_NORMAL);
		in.steer = steer_pack(0, 0, 15);
		in.steer_fresh = FALSE;			/* line task silent */
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
	}
	TEST_CHECK("S2-U24", mc.tgt_l_mms == 0 && mc.tgt_r_mms == 0,
		   "stale steering: the car stops instead of driving blind");

	for (k = 0; k < 60; k++) {
		plant_input(&pl, &in, VM_NORMAL);
		in.steer = steer_pack(k, 0, 15);
		in.steer_fresh = TRUE;
		in.cap_mms = 90;			/* Buddy 5 caution cap */
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
		if (mc.tgt_l_mms > capped_max) {
			capped_max = mc.tgt_l_mms;
		}
	}
	TEST_CHECK("S2-U25", capped_max <= 90 && capped_max > 0,
		   "obstacle speed cap limits line-following speed (FR8)");

	plant_input(&pl, &in, VM_NAV_ACTION);
	in.steer = steer_pack(1, 30, 15);
	in.steer_fresh = TRUE;
	in.cap_mms = 0;
	mc_out_clear(&out);
	mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
	TEST_CHECK("S2-U26", mc.state == MS_IDLE,
		   "steering ignored outside Normal Navigation");

	plant_input(&pl, &in, VM_SAFE_STOP);
	in.steer = steer_pack(2, 30, 15);
	in.steer_fresh = TRUE;
	mc_out_clear(&out);
	mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
	TEST_CHECK("S2-U27", out.duty_l_pm == 0 && out.duty_r_pm == 0,
		   "SAFE_STOP: zero motor output whatever the inputs");

	/* Driving, then an obstacle: no steering, the targets ramp to 0
	   while the wheels still roll.  Without ENC_QUADRATURE the encoder
	   cannot see direction and counts a reversing wheel as forward
	   (the last driven direction), as modelled here. */
	mc_init(&mc);
	plant_init(&pl);
	for (k = 0; k < 50; k++) {
		plant_input(&pl, &in, VM_NORMAL);
		in.steer = steer_pack(k, 0, 15);
		in.steer_fresh = TRUE;
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
	}
	capped_max = 0;				/* here: most reverse duty seen */
	for (k = 0; k < 150; k++) {
		plant_input(&pl, &in, VM_OBSTACLE);
#if !ENC_QUADRATURE
		in.speed_l_mms = iabs(pl.v_l);
		in.speed_r_mms = iabs(pl.v_r);
#endif
		mc_out_clear(&out);
		mc_step(&mc, &in, MOTION_PERIOD_MS, &out);
		plant_step(&pl, out.duty_l_pm, out.duty_r_pm, MOTION_PERIOD_MS);
		if (-out.duty_l_pm > capped_max) {
			capped_max = -out.duty_l_pm;
		}
		if (-out.duty_r_pm > capped_max) {
			capped_max = -out.duty_r_pm;
		}
	}
	TEST_CHECK("S2-U40", (ENC_QUADRATURE || capped_max == 0)
		   && out.duty_l_pm == 0 && out.duty_r_pm == 0
		   && iabs(pl.v_l) < MIN_CTRL_SPEED_MMS && iabs(pl.v_r) < MIN_CTRL_SPEED_MMS,
		   "stop for an obstacle: wheels coast to rest, no reverse duty (no runaway backwards)");
	TEST_SUMMARY("s2 steering");
	return test_fail_;
}

LOCAL void vstep(VFSM *f, UW ev, UW faults, BOOL armed, BOOL ns, BOOL nf,
		 BOOL resume, VFSM_OUT *o)
{
	VFSM_IN	in;

	in.events = ev;
	in.faults = faults;
	in.crit_mask = sys_fault_critical_mask();
	in.armed = armed;
	in.ready = SYS_READY_ALL;
	in.nav_started = ns;
	in.nav_finished = nf;
	in.resume = resume;
	in.dt_ms = MOTION_PERIOD_MS;
	vfsm_step(f, &in, o);
}

LOCAL INT test_vehicle_fsm(void)
{
	VFSM		f;
	VFSM_OUT	o;
	UB		e0;
	UW		t;
	UW		init_flt;
	UW		crit = sys_fault_critical_mask();

	TEST_INIT();
	vfsm_init(&f);
	vstep(&f, SYS_EV_IR_CAL_OK, 0, TRUE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U28", f.mode == VM_INIT, "INIT waits for both IR and IMU calibration");
	vstep(&f, SYS_EV_IMU_CAL_OK, 0, FALSE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U29", f.mode == VM_INIT, "not armed (test image): stays in INIT");
	vstep(&f, 0, 0, TRUE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U30", f.mode == VM_NORMAL && o.changed, "INIT -> NORMAL when calibrated and armed");

	vstep(&f, 0, 0, TRUE, TRUE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U31", f.mode == VM_NAV_ACTION, "barcode command -> NAVIGATION ACTION");
	e0 = f.epoch;
	vstep(&f, SYS_EV_OBSTACLE, 0, TRUE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U32", f.mode == VM_OBSTACLE && f.epoch == ((e0 + 1U) & 7U),
		   "obstacle during a barcode action -> OBSTACLE, new epoch");
	vstep(&f, SYS_EV_AVOID_DONE, 0, TRUE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U33", f.mode == VM_NORMAL, "avoidance done + line found -> NORMAL");

	vstep(&f, 0, SYS_FLT_COLLISION, TRUE, FALSE, FALSE, FALSE, &o);
	TEST_CHECK("S2-U34", f.mode == VM_SAFE_STOP, "critical fault (collision) -> SAFE_STOP");
	vstep(&f, SYS_EV_AVOID_DONE | SYS_EV_IR_CAL_OK, SYS_FLT_COLLISION, TRUE, FALSE, FALSE,
	      FALSE, &o);
	TEST_CHECK("S2-U35", f.mode == VM_SAFE_STOP, "SAFE_STOP holds; events cannot leave it");
	vstep(&f, 0, 0, TRUE, FALSE, FALSE, TRUE, &o);
	TEST_CHECK("S2-U36", f.mode == VM_INIT && !f.ir_ok && !f.imu_ok,
		   "validated RESUME -> INIT with calibration required again");

	init_flt = 0;
	for (t = 0; t <= INIT_TIMEOUT_MS + 100; t += MOTION_PERIOD_MS) {
		vstep(&f, 0, 0, TRUE, FALSE, FALSE, FALSE, &o);
		init_flt |= o.flt_set;		/* raised once, on the timeout step */
	}
	TEST_CHECK("S2-U37", f.mode == VM_SAFE_STOP && (init_flt & SYS_FLT_INIT) != 0,
		   "initialisation timeout -> SAFE_STOP with SYS_FLT_INIT");

	TEST_CHECK("S2-U38", (crit & SYS_FLT_WIFI_LOSS) == 0 && (crit & SYS_FLT_MQTT_LOSS) == 0
		   && (crit & SYS_FLT_LINE_LOST) == 0 && (crit & SYS_FLT_MOTOR_STALL) == 0
		   && (crit & SYS_FLT_RECOVERY_TIMEOUT) != 0 && (crit & SYS_FLT_OBSTACLE_UNAVOIDABLE) != 0
		   && (crit & SYS_FLT_MOTION_ESCALATED) != 0 && (crit & SYS_FLT_REMOTE_STOP) != 0,
		   "severity: network/line-lost/stall degraded; FR12 and escalation critical");
	TEST_CHECK("S2-U39", (sys_fault_event_mask() & SYS_FLT_US_NO_RESPONSE) == 0
		   && (sys_fault_event_mask() & SYS_FLT_COLLISION) != 0,
		   "condition faults clear themselves; event faults need CLEAR");
    {
        VFSM_IN in = {0};
        UW bit;
        BOOL gates = TRUE;
        in.armed = TRUE;
        in.dt_ms = MOTION_PERIOD_MS;
        in.crit_mask = crit;
        in.events = SYS_EV_IR_CAL_OK | SYS_EV_IMU_CAL_OK;
        for (bit = 1; bit <= SYS_READY_B5; bit <<= 1) {
            vfsm_init(&f);
            in.ready = SYS_READY_ALL & ~bit;
            vfsm_step(&f, &in, &o);
            gates = gates && f.mode == VM_INIT;
        }
        TEST_CHECK("S2-U60", gates, "each of five buddies independently gates startup");
        vfsm_init(&f);
        in.ready = SYS_READY_LOCAL;
        in.dt_ms = INIT_TIMEOUT_MS + 5000U;
        vfsm_step(&f, &in, &o);
        TEST_CHECK("S2-U61", f.mode == VM_INIT && o.flt_set == 0,
                   "calibrated local buddies wait indefinitely for hotspot and MQTT");
        in.events = 0; in.ready = SYS_READY_ALL; in.dt_ms = MOTION_PERIOD_MS;
        vfsm_step(&f, &in, &o);
        TEST_CHECK("S2-U62", f.mode == VM_NORMAL, "MQTT ready automatically releases startup hold");
        in.ready = SYS_READY_LOCAL; in.faults = SYS_FLT_WIFI_LOSS;
        vfsm_step(&f, &in, &o);
        TEST_CHECK("S2-U63", f.mode == VM_NORMAL, "later network outage retains existing autonomous policy");
        vfsm_init(&f); in.faults = 0; in.events = SYS_EV_IR_CAL_OK | SYS_EV_IMU_CAL_OK;
        in.ready = SYS_READY_ALL & ~SYS_READY_B5; in.dt_ms = INIT_TIMEOUT_MS + 1;
        vfsm_step(&f, &in, &o);
        TEST_CHECK("S2-U64", f.mode == VM_SAFE_STOP && (o.flt_set & SYS_FLT_INIT),
                   "missing local ultrasonic readiness times out safely");

        /* Local buddies ready, hotspot slow: past the timeout, one invalid
           ping (B5) and one I2C error (B4) each drop a bit for one cycle */
        {
            BOOL held = TRUE;
            UW wait_ms;

            vfsm_init(&f); in.faults = 0; in.dt_ms = MOTION_PERIOD_MS;
            in.events = SYS_EV_IR_CAL_OK | SYS_EV_IMU_CAL_OK;
            in.ready = SYS_READY_LOCAL;
            for (wait_ms = 0; wait_ms <= INIT_TIMEOUT_MS + 2000U;
                 wait_ms += MOTION_PERIOD_MS) {
                vfsm_step(&f, &in, &o);
                in.events = 0;
                held = held && f.mode == VM_INIT && o.flt_set == 0;
            }
            in.ready = SYS_READY_LOCAL & ~SYS_READY_B5;
            vfsm_step(&f, &in, &o);
            held = held && f.mode == VM_INIT && o.flt_set == 0;
            in.ready = SYS_READY_LOCAL;
            vfsm_step(&f, &in, &o);
            in.ready = SYS_READY_LOCAL & ~SYS_READY_B4;
            vfsm_step(&f, &in, &o);
            held = held && f.mode == VM_INIT && o.flt_set == 0;
            TEST_CHECK("S2-U65", held,
                       "one invalid ping after long MQTT wait does not latch INIT fault");
            /* NORMAL still needs all five in the same cycle */
            in.ready = SYS_READY_ALL & ~SYS_READY_B5;
            vfsm_step(&f, &in, &o);
            held = f.mode == VM_INIT;
            in.ready = SYS_READY_ALL;
            vfsm_step(&f, &in, &o);
            TEST_CHECK("S2-U66", held && f.mode == VM_NORMAL,
                       "fresh ping and MQTT release startup after transient sensor failure");
        }
    }
	TEST_SUMMARY("s2 vehicle fsm");
	return test_fail_;
}

LOCAL INT test_storage(void)
{
 S2_FRESH f = {0};
 MOTION_CTRL mc;
 MC_OUT out;
 PID p;
 VFSM vf;
 UW i;
 BOOL ok;
 TEST_INIT();
 TEST_CHECK("S2-U70", sizeof(MOTION_CTRL) == 184U && sizeof(MC_IN) == 28U
            && sizeof(MC_OUT) == 20U && sizeof(VFSM) == 8U
            && sizeof(VFSM_IN) == 20U && sizeof(VFSM_OUT) == 4U
            && sizeof(MOTION_STATUS) == 12U && sizeof(S2_FRESH) == 8U,
            "bounded local records compact without changing shared wire words");
 ok = s2_fresh_sample(&f, 15, 0xfffffff0U, 100U);
 ok = ok && s2_fresh_sample(&f, 15, 0x20U, 100U);
 ok = ok && !s2_fresh_sample(&f, 15, 0x60U, 100U);
 ok = ok && s2_fresh_sample(&f, 0, 0x60U, 100U);
 ok = ok && s2_fresh_sample(&f, 0, 0xc4U, 100U);
 ok = ok && !s2_fresh_sample(&f, 0, 0xc5U, 100U);
 TEST_CHECK("S2-U71", ok, "freshness preserves timestamp wrap, sequence wrap and exact stale boundary");
 mc_init(&mc); mc_out_clear(&out);
 for (i=0;i<=MC_QUEUE_DEPTH;i++)
  (void)mc_submit(&mc,mcmd_pack(MCMD_MOVE,MSRC_AVOID,i,0,2,100),VM_OBSTACLE,0,&out);
 ok = mc_submit(&mc,mcmd_pack(MCMD_MOVE,MSRC_AVOID,5,0,2,100),VM_OBSTACLE,0,&out)==MREJ_QUEUE;
 TEST_CHECK("S2-U72", ok && mc.has_active && mc.q_count == MC_QUEUE_DEPTH && mc.q_head == 0,
            "three-bit count holds all four queued commands without wrapping");
 vf.mode = VM_SAFE_STOP; vf.epoch = 7; vf.ir_ok=TRUE; vf.imu_ok=TRUE; vf.line_lost=TRUE;
 TEST_CHECK("S2-U73", vf.mode == VM_SAFE_STOP && vf.epoch == 7 && vf.ir_ok
            && vf.imu_ok && vf.line_lost,
            "vehicle mode, maximum epoch and calibration flags coexist without overlap");
 pid_init(&p,512,0,0,100000,-100000,100000);
 TEST_CHECK("S2-U74", pid_update(&p,20000,0,0)==40000 && p.i_acc==20000 && p.primed,
            "PID arithmetic retains values beyond motor-specific narrow limits");
 out.duty_l_pm=-1000;out.duty_r_pm=1000;out.dir_l=-1;out.dir_r=1;
 out.flt_set=SYS_FLT_ALL;out.flt_clr=SYS_FLT_ALL;out.result_mask=15;
 out.nav_started=TRUE;out.nav_finished=TRUE;
 TEST_CHECK("S2-U75", out.duty_l_pm==-1000 && out.duty_r_pm==1000
            && out.dir_l==-1 && out.dir_r==1 && out.flt_set==SYS_FLT_ALL
            && out.flt_clr==SYS_FLT_ALL && out.result_mask==15
            && out.nav_started && out.nav_finished,
            "signed duties, both directions and all fault/result flags retain their full range");
 mc.turn_line_seq=S2_LINE_SEQ_NONE;mc.turn_wait_ms=TURN_SETTLE_MS;
 mc.junction_edge_ms=TURN_JUNCTION_EDGE_MS;mc.fault_retries=255;
 TEST_CHECK("S2-U76", mc.turn_line_seq>15 && mc.turn_wait_ms==TURN_SETTLE_MS
            && mc.junction_edge_ms==TURN_JUNCTION_EDGE_MS && mc.fault_retries==255,
            "turn sentinel, settle/handoff timers and retry saturation remain representable");
 TEST_SUMMARY("s2 packed storage");
 return test_fail_;
}

EXPORT INT s2_run_unit_tests(void)
{
	return test_formats() + test_pid() + test_execution() + test_arbitration()
	     + test_faults() + test_steering() + test_vehicle_fsm() + test_storage();
}

#endif /* APP_TEST || HOST_TEST */
