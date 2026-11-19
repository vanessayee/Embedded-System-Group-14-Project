/*
 *	line_follow.h
 *	IR line following (Buddy 3).  Pure logic, host-testable.
 *
 * Digital inputs: GP6 centre, GP2 physical right, GP26 physical left
 * barcode sensor. Internal IR_L names GP2; task_line mirrors steering
 * into the physical motor convention (+ is clockwise/right).
 * Centre-only black: turn left (right wheel faster, left wheel slower).
 * GP2-only black: turn right (left wheel faster, right wheel slower).
 * The wheel-speed difference starts at LF_EDGE_STEER_MIN and grows while
 * the same sensor stays black, up to LF_EDGE_STEER_MAX (curves).  After a
 * sustained contact the turn is held for LF_HOLD_MS once both sensors are
 * white, so the line cannot slip past the sensor while the car drives
 * straight.  Both GP6 and GP2 black for 60 ms: one junction event until
 * either sensor clears. GP26 never participates in junction detection.
 * Both tracking sensors white: continue straight in normal corridor
 * tracking, with no elapsed-time search. Known loss outside NORMAL still
 * reports LOST/FOUND for avoidance. Barcode turns execute at the next junction; normal
 * line tracking resumes when Buddy 2 completes the encoder-based turn.
 */

#ifndef LINE_FOLLOW_H
#define LINE_FOLLOW_H

#include <tk/typedef.h>

#define IR_L		0U	/* bit index in a 3-bit IR state */
#define IR_C		1U
#define IR_R		2U
#define IR_BIT(i)	(1U << (i))

/* Line position (3 bits) */
#define LP_CENTER	0U
#define LP_LEFT_NEAR	1U
#define LP_LEFT_FAR	2U
#define LP_RIGHT	3U	/* no longer produced (IR_R does not steer) */
#define LP_LOST_LEFT	4U	/* known loss: search towards tracking gap */
#define LP_LOST_RIGHT	5U	/* line lost: turning towards IR_R */
#define LP_JUNCTION	6U
#define LP_RIGHT_NEAR	7U	/* centre sensor on black */

/* Line state (2 bits) */
#define LS_ON_LINE	0U
#define LS_SEARCHING	1U	/* Error/recover: lost-line turn running */
#define LS_LOST		2U	/* lost outside Normal Navigation       */
#define LS_UNKNOWN	3U	/* not calibrated yet                   */

/* Events returned by lf_step() */
#define LF_EV_LOST		0x01U
#define LF_EV_FOUND		0x02U
#define LF_EV_JUNCTION		0x04U
#define LF_EV_RECOVERY_FAIL	0x08U /* reserved; recovery does not emit this */

/* Calibration results */
#define LF_CAL_OK		0
#define LF_CAL_UNSTABLE		(-2)	/* readings flickered while still   */

/* Tuning [CAL] */
#define LF_CONFIGURED_BLACK_LEVEL 1U	/* this robot's LM393 D0 is high on black */
#define LF_CRUISE_SPEED		4	/* 4/15 of Buddy 2 cruise: about
					   42 mm/s with the current 158 mm/s
					   (was 5 = 53 mm/s): slower over the
					   barcodes so GP26 sees every bar */
#define LF_NEAR_SPEED		LF_CRUISE_SPEED /* constant speed avoids a correction jerk */
/* Edge correction as a wheel-speed difference.  One steer unit adds
   cruise/STEER_UNIT_DIV = 2 mm/s to one wheel and takes it from the other
   (130 mm track), so at the 42 mm/s line speed:
       steer  8 (first contact):  inner 26, outer 58 mm/s, ~14 deg/s
       steer 14 (after 150 ms):   inner 14, outer 70 mm/s, ~25 deg/s
       steer 21 (after 325 ms):   inner  0, outer 84 mm/s, ~37 deg/s
   The longer the sensor stays black (a bend), the faster the outer wheel
   and the slower the inner one, until the inner wheel stands still and
   the car turns about it (65 mm radius).  (Was capped at steer 12: inner
   18 / outer 66 mm/s, too gentle for the end of a bend.)
   Hold times are 1.25 x the 53 mm/s values: the same heading change and
   the same distance travelled at the lower speed. */
#define LF_EDGE_STEER_MIN	8	/* first sample on an edge sensor     */
#define LF_EDGE_STEER_MAX	21	/* inner wheel stopped: 21 x 2 mm/s =
					   the 42 mm/s line speed             */
#define LF_EDGE_GROW_MS		25	/* +1 steer per 25 ms the same sensor
					   stays black (MIN -> MAX in 325 ms) */
#define LF_HOLD_MIN_MS		120	/* sensor black at least this long: a
					   curve, not a small drift ...       */
#define LF_HOLD_MS		380	/* ... so keep turning the same way this
					   long after both sensors go white,
					   unless a sensor sees black first    */
#define LF_HOLD_STEER		LF_EDGE_STEER_MIN
#define LF_STEER_SLEW_STEP	8	/* first correction in one 20 ms cycle */
#define LF_EDGE_GAP_ERR		0	/* corridor is a straight-ahead deadband */
#define LF_LOST_CONFIRM_MM	40	/* lost after this much travel ... (was 25) */
#define LF_LOST_CONFIRM_MS	250	/* ... or this long without line (was 300, 400) */
#define LF_JUNCTION_CONFIRM_MS	60U /* both GP6 and GP2 black, reject flicker */
#define LF_LOST_DIR		(-1)	/* search towards the C-to-IR_L tracking
					   corridor, never towards the barcode sensor */
#define LF_LOST_ARC_STEER	4	/* ... first use a slow corrective arc */
#define LF_LOST_ARC_SPEED	3	/* ... at about 32 mm/s while searching */
#define LF_LOST_ARC_MM		40	/* ... for this much travel ... */
#define LF_LOST_ARC_MS		400	/* ... or this long, then ... */
#define LF_LOST_PIVOT_STEER	12	/* ... pivoting on the spot at this
					   steer (same magnitude as post-turn correction) */
#define LF_SETTLE_MS		300	/* after a side pivot: drive on ...   */
#define LF_SETTLE_SPEED		3	/* ... at about 32 mm/s this long     */
#define LF_RIGHT_TURN_STRAIGHT_MS 150	/* short straight handoff on GP6 */
/* Realign after a barcode junction turn (lf_turn_done).  Turning left,
   the new line sweeps centre -> gap -> GP2; Buddy 2 ends the turn with
   the line in the gap or, overshooting, under GP2.  A sensor still
   black then means the car points past the line: pivot on the spot
   towards that sensor until it is white again (line back between the
   two sensors), then drive straight and resume normal tracking. */
#define LF_POST_TURN_PIVOT_STEER 12 /* corrective pivot after a barcode turn */
#define LF_POST_TURN_PIVOT_MAX_MS 2000	/* give up the pivot after this    */
#define LF_POST_TURN_MS		400	/* both white this long right after the
					   turn: already aligned, realign done */

typedef struct {
	UB	black_level[3];	/* raw level meaning "black", per sensor */
	BOOL	calibrated;
	UB	state;		/* LS_* */
	UB	pos;		/* LP_* */
	INT	err;		/* last error, -2..3 */
	INT	steer_cmd;	/* slew-limited steering sent last cycle */
	INT	edge_side;	/* +1 C black, -1 IR_L black, 0 neither */
	UW	edge_ms;	/* how long that sensor has been black */
	INT	hold_side;	/* turn kept after a sustained contact ... */
	UW	hold_ms;	/* ... for this much longer */
	UW	lost_ms;	/* time without the line */
	W	lost_start_mm;
	BOOL	lost_pending;
	UW	settle_ms;	/* half-speed time left after a pivot */
	UW	straight_ms;	/* drive straight after GP6 finds the right branch */
	BOOL	turning;	/* lost-line turn under way ... */
	W	turn_start_mm;	/* ... since this odometer reading ... */
	UW	turn_ms;	/* ... and this long */
	BOOL	in_junction;
	BOOL	junction_reported; /* one event per continuous crossing */
	UW	junction_ms;
	BOOL	post_turn;	/* realigning after a barcode turn ... */
	UW	post_turn_ms;	/* ... for this long so far ... */
	INT	post_pivot;	/* ... pivoting towards this side (err sign), 0 none */
} LINE_FOLLOW;

typedef struct {
	INT	steer;		/* -63..63 */
	UB	speed_frac;	/* 0..15 of cruise */
	UB	events;		/* LF_EV_* */
	BOOL	steering;	/* TRUE: publish steer */
} LF_OUT;

IMPORT void lf_init(LINE_FOLLOW *lf);
IMPORT void lf_reset_tracking(LINE_FOLLOW *lf);

/* A barcode junction command has just finished: forget everything seen
   while Buddy 2 was turning and realign on the new line (see
   LF_POST_TURN_*).  Call lf_reset_tracking() each period while the
   command runs so no hold/lost/junction state builds up meanwhile. */
IMPORT void lf_turn_done(LINE_FOLLOW *lf, BOOL centre_acquired);

/* Startup qualification from any position (white or black).  The samples
   must be stable; black polarity is the known hardware configuration. */
IMPORT INT  lf_calibrate(LINE_FOLLOW *lf, const UB *raw, UINT n);

/* Raw 3-bit levels -> black bits (1 = black) */
IMPORT UB   lf_black_bits(const LINE_FOLLOW *lf, UB raw);

/* One period.  'navigating' = Normal Navigation (steering allowed and
   the search is ours); otherwise only tracking/events are produced. */
IMPORT void lf_step(LINE_FOLLOW *lf, UB black, W odo_mm, UW dt_ms,
		    BOOL navigating, LF_OUT *out);

#endif /* LINE_FOLLOW_H */
