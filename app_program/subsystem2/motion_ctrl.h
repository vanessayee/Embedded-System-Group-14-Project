/*
 *	motion_ctrl.h
 *	Motion Control core (Buddy 2): command arbitration, move/turn
 *	execution, line-steering, PID speed control, stall/encoder fault
 *	detection with bounded retry and escalation.
 *
 *	Pure logic: no kernel calls and no hardware access, so the whole
 *	statechart can be exercised on the host against a simulated plant.
 *	The Motion Control task (task_motion.c) feeds it measurements every
 *	MOTION_PERIOD_MS and applies the duties it returns.
 *
 *	Statechart (Week 6 section 6.2 + Tribunal 1 changes):
 *	    IDLE -> MOVING STRAIGHT / TURNING   on a command (or line steering)
 *	    STRAIGHT <-> TURNING                next queued command, via creep
 *	                                        speed instead of a full stop
 *	    any -> IDLE                          queue empty / stop()
 *	    any -> FAULT                         stall, encoder fault, timeout
 *	    FAULT -> IDLE                        condition clear for 300 ms
 *	    FAULT count >= MAX_FAULT_RETRIES     -> SYS_FLT_MOTION_ESCALATED
 *	                                        (system Safe Stop; no endless
 *	                                        Fault <-> Idle cycling)
 */

#ifndef MOTION_CTRL_H
#define MOTION_CTRL_H

#include <tk/typedef.h>
#include "motion_msg.h"
#include "sys_defs.h"
#include "pid.h"
#include "s2_storage.h"

#define MC_QUEUE_DEPTH	4	/* commands queued behind the active one */
/* These bounds are part of the local representation, not the wire format. */
typedef char s2_queue_depth_check[(MC_QUEUE_DEPTH == 4) ? 1 : -1];

/* Inputs for one control step */
typedef struct S2_LOCAL_BITS {
	W	dist_l_um;	/* cumulative signed wheel travel, um */
	W	dist_r_um;
	W	speed_l_mms;	/* measured wheel speeds, mm/s */
	W	speed_r_mms;

	UW	steer;		/* steering word (motion_msg.h) */

	UW	line_seen;	/* line-seen word (motion_msg.h LSEEN_*) */


	/* Bounded input information; numeric measurements retain full precision. */
	unsigned int enc_valid : 1;	/* encoder snapshot readable */
	unsigned int vmode : 3;		/* VM_* vehicle mode */
	unsigned int armed : 1;		/* motors allowed to move */
	unsigned int steer_fresh : 1;	/* seq changed within STEER_STALE_MS */
	unsigned int cap_mms : 10;	/* obstacle speed cap, 0 = none */
	unsigned int line_lost_cap : 1;	/* cap to creep while the line is lost */
	unsigned int line_seen_fresh : 1; /* seq changed within LSEEN_STALE_MS */
} MC_IN;

/* Outputs of submit/step calls (accumulate over one task cycle) */
typedef struct S2_LOCAL_BITS {
	UH	result[MSRC_COUNT];	/* newest result word per source */

	/* Task-owned bounded fields; never publish this C bit-field layout. */
	signed int duty_l_pm : 11; /* -1000..1000 */
	signed int duty_r_pm : 11;
	signed int dir_l : 2; /* commanded direction, for 1-channel encoders */
	signed int dir_r : 2;
	unsigned int result_mask : 4; /* bit n set: result[n] is new */
	unsigned int flt_set : 27; /* fault bits to raise */
	unsigned int flt_clr : 27; /* fault bits to clear */
	unsigned int nav_started : 1; /* a barcode command began */
	unsigned int nav_finished : 1; /* the barcode command ended */
} MC_OUT;

typedef struct S2_LOCAL_BITS {
	UW	active;		/* running command word */
	UW	queue[MC_QUEUE_DEPTH];

	W	start_l_um;	/* wheel travel when the command started */
	W	start_r_um;
	W	target_um;	/* distance (MOVE) or wheel arc (TURN) */
	W	seek_from_um;	/* barcode TURN: IR sensors used from here ... */
	W	seek_extra_um;	/* ... and up to target + this */

	W	advance_um;	/* ... until this much travel ...           */

	W	junction_edge_um;	/* mean wheel position when GP2 cleared the tape */
	UW	elapsed_ms;
	UW	timeout_ms;
	W	tgt_l_mms;	/* ramped wheel speed targets */
	W	tgt_r_mms;
	PID	pid_l;
	PID	pid_r;
	W	cruise_mms;
	W	last_l_um;
	W	last_r_um;

	UW	stall_ms_l;
	UW	stall_ms_r;
	W	moved_l_um;	/* left travel while the right wheel was silent */
	W	moved_r_um;

	UW	fault_clear_ms;

	UW	healthy_ms;

	/* Task-owned bounded fields; never publish this C bit-field layout. */
	unsigned int state : 2; /* MS_* */
	unsigned int has_active : 1;
	unsigned int q_head : 2;
	unsigned int q_count : 3;
	unsigned int seek_seen : 1; /* leading IR sensor has crossed the new line */
	unsigned int turn_contact_samples : 2; /* direction-dependent leading sensor edge */
	unsigned int turn_gap_samples : 2; /* confirmed centre/right white after edge */
	unsigned int turn_line_seq : 5; /* 0..15 sequence, 16 means no previous sample */
	unsigned int turn_align : 1; /* settle and trim the final sensor gap */
	unsigned int turn_wait_ms : 8;
	unsigned int advancing : 1; /* junction turn: driving to the turn point */
	unsigned int adv_r_black : 1; /* ... right sensor seen on the crossing line */
	unsigned int adv_ref : 1; /* ... and has left it: advance_um re-based  */
	unsigned int junction_black : 1; /* both tracking sensors saw the crossing before a command */
	unsigned int junction_edge_ms : 8; /* remaining lifetime of the saved GP2 falling edge */
	unsigned int primed : 1;
	unsigned int hard_stop : 1; /* next step outputs zero immediately */
	unsigned int stall_l : 1;
	unsigned int stall_r : 1;
	unsigned int fault_retries : 8; /* faults since the last successful command */
	unsigned int fault_bits : 27; /* local fault bits currently raised */
	unsigned int escalated : 1;
} MOTION_CTRL;

/* Shared classification for execution and turn diagnostics. */
IMPORT BOOL mc_is_junction_turn(UW cmd);

IMPORT void mc_init(MOTION_CTRL *mc);
IMPORT void mc_out_clear(MC_OUT *out);

/* Offer a command word.  Checks format, source, epoch, mode and
   priority; returns MREJ_NONE if accepted (a REJECTED result is posted
   in 'out' otherwise).  RESUME/CLEAR are handled by the task itself. */
IMPORT UW   mc_submit(MOTION_CTRL *mc, UW cmd, UW vmode, UW epoch, MC_OUT *out);

/* stop() bypass: abort everything, zero output on the next step */
IMPORT void mc_force_stop(MOTION_CTRL *mc, MC_OUT *out);

/* Called after the vehicle mode changed */
IMPORT void mc_mode_changed(MOTION_CTRL *mc, UW new_mode, MC_OUT *out);

/* One control period */
IMPORT void mc_step(MOTION_CTRL *mc, const MC_IN *in, UW dt_ms, MC_OUT *out);

IMPORT W    mc_creep_speed(const MOTION_CTRL *mc);
IMPORT void mc_set_cruise(MOTION_CTRL *mc, W mms);

/* Wheel arc for an in-place turn of 'deg' degrees, micrometres */
IMPORT W    mc_turn_arc_um(INT deg);

#endif /* MOTION_CTRL_H */
