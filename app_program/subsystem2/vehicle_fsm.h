/*
 *	vehicle_fsm.h
 *	Vehicle operating-mode state machine (Week 6 system-level statechart,
 *	section 5), owned by Buddy 2 (Tribunal 1, Version B).  Runs inside the
 *	Motion Control task; there is deliberately no separate Vehicle
 *	Controller task (Week 6 section 4).  Pure logic.
 *
 *	    INIT ------ all five buddies + MQTT ready -----> NORMAL
 *	    INIT ------ init timeout / calibration fail ---> SAFE_STOP
 *	    NORMAL ---- barcode command starts ------------> NAV_ACTION
 *	    NAV_ACTION  command done/aborted --------------> NORMAL
 *	    NORMAL/NAV  obstacle detected (Buddy 5) -------> OBSTACLE
 *	    OBSTACLE -- avoidance done + line reacquired --> NORMAL
 *	    OBSTACLE -- recovery failed -------------------> SAFE_STOP
 *	    any ------- critical fault bit ----------------> SAFE_STOP
 *	    SAFE_STOP - validated supervisory RESUME ------> INIT
 *
 *	The epoch (3 bits) increments on entering INIT, OBSTACLE and
 *	SAFE_STOP; commands built under an older epoch are rejected as stale.
 */

#ifndef VEHICLE_FSM_H
#define VEHICLE_FSM_H

#include <tk/typedef.h>
#include "sys_defs.h"
#include "s2_storage.h"

typedef struct S2_LOCAL_BITS {
	UW	in_mode_ms;	/* time spent in the current mode */

	/* Task-owned bounded fields; never publish this C bit-field layout. */
	unsigned int mode : 3; /* VM_* */
	unsigned int epoch : 3; /* 0..7 */
	unsigned int ir_ok : 1; /* IR calibrated in this INIT epoch */
	unsigned int imu_ok : 1; /* IMU calibrated in this INIT epoch */
	unsigned int line_lost : 1; /* last line event was LOST */
	unsigned int local_seen : 1; /* B2..B5 all ready together in this INIT */
} VFSM;

typedef struct S2_LOCAL_BITS {
	UW	dt_ms;

	/* Task-owned bounded fields; never publish this C bit-field layout. */
	unsigned int events : 10; /* SYS_EV_* consumed this cycle */
	unsigned int faults : 27; /* shared fault bitmask */
	unsigned int crit_mask : 27; /* critical fault bits */
	unsigned int ready : 5; /* SYS_READY_B1..B5, fresh runtime checks */
	unsigned int armed : 1; /* motors allowed (FALSE in test builds) */
	unsigned int nav_started : 1; /* barcode command began */
	unsigned int nav_finished : 1; /* barcode command ended */
	unsigned int resume : 1; /* validated RESUME accepted */
} VFSM_IN;

typedef struct S2_LOCAL_BITS {

	/* Task-owned bounded fields; never publish this C bit-field layout. */
	unsigned int changed : 1;
	unsigned int old_mode : 3;
	unsigned int flt_set : 27; /* faults raised by the FSM itself */
} VFSM_OUT;

IMPORT void vfsm_init(VFSM *f);
IMPORT void vfsm_step(VFSM *f, const VFSM_IN *in, VFSM_OUT *out);

#endif /* VEHICLE_FSM_H */
