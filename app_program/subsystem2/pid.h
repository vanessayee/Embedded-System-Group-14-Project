/*
 *	pid.h
 *	Integer PID speed controller (pure logic, host-testable).
 *
 *	Controller state is included in Change Requirement 1. The validity
 *	flag uses one bit. Arithmetic fields retain the full signed range
 *	accepted by pid_init/update; narrowing those generic fixed-point
 *	values would change overflow/precision behaviour.
 */

#ifndef PID_H
#define PID_H

#include <tk/typedef.h>
#include "s2_storage.h"

typedef struct S2_LOCAL_BITS {
	W	kp_q8;		/* output units per (mm/s) error, *256        */
	W	ki_q8;		/* per (mm/s * cycle) of accumulated error    */
	W	kd_q8;		/* per (mm/s) change of measurement per cycle */
	W	i_acc;		/* integral accumulator, (mm/s) * cycles      */
	W	i_lim;		/* |i_acc| limit (anti-windup)                */
	W	prev_meas;	/* derivative on measurement (no kick)        */
	W	out_min;	/* output clamp, permille                     */
	W	out_max;

	/* Task-owned bounded fields; never publish this C bit-field layout. */
	unsigned int primed : 1; /* prev_meas valid                            */
} PID;

IMPORT void pid_init(PID *p, W kp_q8, W ki_q8, W kd_q8, W i_lim,
		     W out_min, W out_max);
IMPORT void pid_reset(PID *p);

/* One control step.  ff is a feed-forward term in output units.
   Returns the clamped output (permille duty). */
IMPORT W pid_update(PID *p, W target, W measured, W ff);

#endif /* PID_H */
