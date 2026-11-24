/*
 *	avoid_plan.h
 *	Avoidance planning (Buddy 5, Avoidance & Recovery).  Pure logic.
 *
 *	Project Briefing p.8 workflow: detect -> measure width -> choose a
 *	bypass direction -> go off line &
 *	bypass -> search & reacquire the line behind the obstacle -> return
 *	back to the line.  If there is not enough room to pivot in front of
 *	the obstacle, planning fails and the vehicle enters Safe Stop.
 *
 *	Board dimensions are not assumed. Each side must have enough observed
 *	free corridor for the robot width, requested gap and uncertainty reserve.
 *	Choose the shorter qualifying route, preferring right for an equal offset.
 *	If neither side qualifies, fail planning instead of attempting a bypass.
 *
 *	Plan for side s (+1 right, -1 left): optionally move forward, calculate
 *	a safe outward angle T from measured front clearance and lateral offset,
 *	then calculate a shallow return so forward and inward motion happen together:
 *	    [MOVE approach] TURN s*T MOVE d1 TURN -s*T PASS TURN -s*R
 *	    SEARCH (move until the line is found, at most dS) TURN s*R
 */

#ifndef AVOID_PLAN_H
#define AVOID_PLAN_H

#include <tk/typedef.h>
#include "s5_msg.h"
#include "../subsystem2/motion_msg.h"

/* Vehicle geometry and clearances [CAL]: measured on the actual car */
#define AV_ROBOT_WIDTH_MM	125
#define AV_ROBOT_LENGTH_MM	175
#define AV_SENSOR_BEHIND_FRONT_MM 0	/* sensor is directly above front bumper */
#define AV_SIDE_GAP_MM		50	/* requested closest side gap (5 cm; was 8 cm,
					   measured ~95-100 mm on the robot) */
#define AV_EDGE_UNCERTAINTY_MM	15	/* [CAL] reserve beyond the requested body gap */
#define AV_REQUIRED_CORRIDOR_MM	(AV_ROBOT_WIDTH_MM + AV_SIDE_GAP_MM \
				 + AV_EDGE_UNCERTAINTY_MM)
#define AV_TURN_MIN_DEG		30	/* calculated outward turn bounds */
#define AV_TURN_MAX_DEG		75	/* steeper departure reduces reverse distance */
#define AV_FRONT_PATH_ALLOWANCE_MM 10	/* [CAL] turn/rounding allowance */
#define AV_RETURN_TURN_MIN_DEG	30	/* encoder-reliable shallow return */
#define AV_RETURN_TURN_MAX_DEG	60	/* shortest body-clearance-checked return */
#define AV_RETURN_PATH_ALLOWANCE_MM 10	/* retain margin during return */
#define AV_DETECT_FRONT_GAP_MM	150	/* stop near 15 cm before planning */
#define AV_MIN_FRONT_CLEARANCE_MM	120	/* closest calculated turn-start gap */
#define AV_EMERGENCY_FRONT_CLEARANCE_MM	100	/* bumper gap while moving */
#define AV_REVERSE_MAX_MM	150	/* bounded retreat on travelled path */
#define AV_REVERSE_RESCAN_MARGIN_MM 20	/* absorb profile/odometry variation */
#define AV_SIDE_PASS_MAX_MM	MCMD_MOVE_MAX_MM /* actuator safety bound, not obstacle length */
#define AV_SEARCH_EXTRA_MM	150
#define AV_LINE_LOCK_SAMPLES	3U

/* Confirm the centre-only black edge using consecutive fresh IR samples. */
typedef struct {
	UINT run;
} AV_LINE_LOCK;
IMPORT BOOL av_line_lock_sample(AV_LINE_LOCK *lock,
			       BOOL centre_black, BOOL right_black);
IMPORT W av_post_edge_advance_mm(W body_path, W edge_allowance);
#define AV_LINE_SENSOR_AHEAD_MM	80	/* [CAL] centre IR sensor to wheel axle */
#define AV_PREFER_RIGHT		1	/* tie-break when clearances equal */
#define AV_MAX_STEPS		8
#if AV_MAX_STEPS > 15 || MCMD_MOVE_MAX_MM > 2047 || AV_TURN_MAX_DEG > 2047 || AV_RETURN_TURN_MAX_DEG > 2047
#error "Update Buddy 5 plan storage for the configured actuator bounds"
#endif

#define AV_SIDE_NONE		0U
#define AV_SIDE_LEFT		1U
#define AV_SIDE_RIGHT		2U

#define AVS_TURN		0U
#define AVS_MOVE		1U
#define AVS_SEARCH		2U	/* move forward until the line is found */
#define AVS_PASS		3U	/* side-ultrasonic terminated move */

#define AVP_OK			0
#define AVP_NO_PROFILE		(-1)
#define AVP_NO_CLEARANCE	(-2)

typedef struct S5_LOCAL_BITS {

	unsigned int kind : 2;

	signed int arg : 12;
} AV_STEP;

typedef struct S5_LOCAL_BITS {

	unsigned int side : 2;

	unsigned int n : 4;
	AV_STEP	step[AV_MAX_STEPS];
	W	offset_mm;	/* lateral offset of the bypass */
	W	required_mm;	/* clearance the car needs */
	W	front_gap_mm;	/* obstacle gap from front bumper */
	W	side_gap_mm;	/* predicted closest side gap */
	W	approach_mm;	/* forward move before outward turn */
	W	turn_deg;	/* calculated outward turn, 30..75 */
	W	required_front_mm; /* geometry needed at maximum turn */
	W	forward_before_pass_mm; /* approach + diagonal forward progress */
	W	return_turn_deg;	/* configured encoder-reliable return */
} AV_PLAN;

/* Maximum side range covered by the original measured profile after
   departure; not just the requested gap from its apparent near edge. */
IMPORT W av_side_watch_limit_mm(const S5_PROFILE *p, const AV_PLAN *plan,
			       INT look_deg, W allowance_mm);
IMPORT UW av_motion_wait_ms(UW type, INT arg, W creep_mms, UW extra_ms);
/* Pass heading correction.  The departure turn pair does not cancel on the
   robot (24 Nov right bypass: side range 190 -> 165 mm over 107 mm, ~13 deg
   toward the obstacle).  The same error points a LEFT bypass away from the
   obstacle, which is why only the right side failed.  Correct convergence
   once per pass from the live side ranges; never turn toward the obstacle. */
#define S5_HEADING_SETTLE_MM	20	/* skip near-corner echoes after confirmation */
#define S5_HEADING_BASELINE_MM	40	/* travel spanned by the side-range trend
					   (was a 100 mm two-point comparison:
					   too slow for a 170 mm box, 25 Nov) */
#define S5_HEADING_MIN_SAMPLES	6	/* ~3 mm per ping at 50 mm/s */
#define S5_HEADING_FIX_MIN_DEG	5	/* trend angle that triggers a correction */
#define S5_HEADING_FIX_MAX_DEG	20	/* bound one correction turn */
#define S5_SIDE_MIN_GAP_MM	20	/* body gap guard while passing */

/* Least-squares trend of the perpendicular side distance against travel */
typedef struct {
	W	n;
	W	t_first, t_last;
	D	st, sd, stt, std;
} AV_TREND;
IMPORT void av_trend_reset(AV_TREND *t);
IMPORT void av_trend_add(AV_TREND *t, W travel_mm, W side_mm);
/* TRUE when the trend spans S5_HEADING_BASELINE_MM with enough samples;
   *deg = angle toward the obstacle (+ closing, - opening). */
IMPORT BOOL av_trend_closing_deg(const AV_TREND *t, INT *deg);

/* Heading toward the obstacle side from two side-watch ranges taken
   travel_mm apart at look_deg: + = converging, - = diverging, degrees. */
IMPORT INT av_side_convergence_deg(W r_from_mm, W r_to_mm, W travel_mm,
				   INT look_deg);

IMPORT INT av_plan(const S5_PROFILE *p, AV_PLAN *plan);
/* -1 = reversing is unsafe/unhelpful, 0 = not needed, positive = reverse mm */
IMPORT W   av_reverse_needed_mm(const S5_PROFILE *p);
/* Safe straight distance after the sensor reaches the far edge, using the
   live obstacle-to-body side gap measured during PASS. */
IMPORT W   av_return_approach_mm(INT deg, W measured_side_gap_mm);

#endif /* AVOID_PLAN_H */
