/*
 *	obstacle_profile.h
 *	Obstacle profile generation (Buddy 5, Obstacle Profile task): convert
 *	servo-angle / distance samples into obstacle position, width and the
 *	free lateral clearance on each side.  Pure logic, host-testable.
 *
 *	Project Briefing p.8: "Obstacle Width W ~ f(dL, dC, dR, theta)
 *	(estimated using distance geometry & scan angle)".  Each valid sample
 *	becomes a point (x = d sin a lateral, y = d cos a forward).  Samples
 *	within DEPTH_TOL of the nearest forward return belong to the obstacle;
 *	its edges lie between the outermost "hit" and the first "miss" on each
 *	side (the fine scan narrows that gap to the fine step).  A sweep that
 *	reaches the scan limit while still on the obstacle cannot establish
 *	that side's edge: that side's clearance is UNKNOWN and is never
 *	treated as free (the brief: never invent a bypass).
 *
 *	Tribunal 1: "Assuming ultrasonic scanning determines exact obstacle
 *	geometry" was rejected as unrealistic -- the HC-SR04 beam widens the
 *	apparent obstacle. Preserve the observed edges by default; an optional
 *	calibrated correction is bounded to one scan cell per edge.
 */

#ifndef OBSTACLE_PROFILE_H
#define OBSTACLE_PROFILE_H

#include <tk/typedef.h>
#include "s5_msg.h"

#define OP_MAX_SAMPLES		64
#if OP_MAX_SAMPLES > 127
#error "Update Buddy 5 profile count storage for this capacity"
#endif
#define OP_FORWARD_CONE_DEG	30	/* nearest face searched in +-30 deg */
#define OP_FRONT_ESTIMATE_DEG	3	/* median of central -3/0/+3 rays   */
#define OP_DETECT_RANGE_MM	700	/* farther = no obstacle to avoid    */
#define OP_DEPTH_TOL_MM		50	/* maximum front-plane tolerance */
#define OP_DEPTH_MIN_MM		10	/* near-range front-plane noise allowance */
#define OP_DEPTH_RANGE_DIV	20	/* 5% of front distance, capped above */
#define OP_MAX_HIT_GAP		1	/* bridge one isolated fine-scan miss */
#define OP_MIN_HITS		2
#define OP_MIN_VALID		5
#define OP_FROZEN_SPREAD_MM	10	/* identical readings everywhere ...  */
#define OP_FROZEN_SPAN_DEG	60	/* ... over this sweep = servo stuck  */
#define OP_CLEAR_CAP_MM		2540
#define S5_BEAM_COMP_DEG	0	/* preserve measured edges until calibrated */
#define OP_EDGE_RESOLUTION_DEG	3	/* one measured fine-scan cell      */
/* Corner echo at an edge (24 Nov scan of a 130 mm box at 205 mm: rays at
   +-21 deg returned the 215 mm corner range and projected into the face
   band, widening it by 44 mm).  A smooth face echoes straight back only
   while its perpendicular is inside the beam; beyond that the corner is
   heard at sqrt(face^2 + edge^2).  The outermost face ray is dropped, at
   most once per side, when it looks like such a corner echo. */
#define OP_BEAM_DIRECT_DEG	15	/* HC-SR04 nominal beam half-angle   */
#define OP_CORNER_RANGE_MM	8	/* > one 5 mm range step farther ... */
#define OP_CORNER_DEPTH_MM	3	/* ... and projected nearer than face */

/* One scan sample as the Profile task keeps it, at the widths its ranges
   need (servo +-80 deg, HC-SR04 0..4000 mm): 4 bytes, where an INT / W /
   UB layout takes 12 (Change Requirement 1; 64 samples = 256 bytes). */
typedef struct S5_LOCAL_BITS {

	signed int angle_deg : 8;

	unsigned int status : 2;

	signed int dist_mm : 13;
} OP_SAMPLE;

/* samples need not be sorted; scan_limit_deg = largest |angle| scanned */
IMPORT void op_build(const OP_SAMPLE *s, UINT n, INT scan_limit_deg,
		     S5_PROFILE *out);

/* Optional calibrated correction, at most one scan cell per known edge.
   Disabled by default. Never increases measured free side clearance. */
IMPORT void op_beam_compensate(S5_PROFILE *p, INT half_angle_deg);

IMPORT W    op_sin_q15(INT deg);	/* integer sine, Q15 */
IMPORT W    op_cos_q15(INT deg);
IMPORT INT  op_atan_deg(W y, W x);	/* bearing of lateral y at forward x, deg */

/* Two or three valid pings: require a pair within 10% (minimum 10 mm).
   Returns -1 for disagreement, insufficient samples or invalid range. */
IMPORT W op_range_consensus_mm(const UW *ranges, UINT n);

#endif /* OBSTACLE_PROFILE_H */
