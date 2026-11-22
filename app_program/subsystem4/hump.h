/*
 *	hump.h
 *	Hump detection, height estimation and highest-hump tracking (Buddy 4,
 *	Hump & Terrain Analysis).  Pure logic, host-testable.
 *
 *	Default: one wheel climbs while the opposite wheel stays on level
 *	ground. Height = physical wheel-centre span * sin(abs(peak roll)).
 *	Both roll signs are accepted; return to level completes the hump.
 *	HUMP_SIDEWAYS=0 selects the legacy forward-traversal method:
 *	height = sum of ds * sin(pitch) over the hump, with ds from the wheel
 *	encoders (Buddy 2) and pitch from the IMU complementary filter.
 *	Double-integrating acceleration was rejected by the team (drift).
 *
 *	    IDLE ---- pitch >= HUMP_START for 2 samples + forward travel -> CLIMB
 *	    CLIMB --- pitch <= -HUMP_START ------------------> DESCENT
 *	    DESCENT - level again for 3 samples -------------> report, IDLE
 *	    any ----- longer than HUMP_MAX_LEN / invalid ----> discard, IDLE
 *
 *	Representation (Tribunal 1, Version B decision): heights are kept in
 *	micrometres internally (full resolution, also for the maximum) and
 *	compacted ONLY for storage/transmission to 0..100 mm in 2 mm steps =
 *	51 codes -> 6 bits; code 63 = no hump yet.  HUMP_STEP_MM may become 4
 *	(5 bits) after the known-hump test (Version B change log).
 */

#ifndef HUMP_H
#define HUMP_H

#include <tk/typedef.h>
#include "s4_status.h"

#define HUMP_STEP_MM		2	/* Tribunal 1 Version B           */
#define HUMP_RANGE_MM		100
#define HUMP_CODE_NONE		63U
#define HUMP_CODE_MAX		(HUMP_RANGE_MM / HUMP_STEP_MM)	/* 50 */

/* Detection thresholds [CAL] */
#define HUMP_START_CDEG		300	/* 3 deg nose-up                  */
#define HUMP_LEVEL_CDEG		150	/* back to level within 1.5 deg   */
#define HUMP_LEVEL_SAMPLES	3
#define HUMP_MIN_HEIGHT_UM	3000	/* ignore < 3 mm (noise)          */
#define HUMP_MAX_LEN_MM		1000	/* longer = a ramp, not a hump    */
#ifndef HUMP_SIDEWAYS
#define HUMP_SIDEWAYS		1	/* one wheel climbs; 0 = forward-pitch integration */
#endif
#define HUMP_PHYSICAL_TRACK_MM	110	/* measured wheel-centre spacing; not
					   the effective encoder-turn calibration */
#define HUMP_ROLL_FALL_UM		1000	/* falling peak hysteresis */

/* Two consecutive rising samples and at most three level confirmations. */
#if HUMP_LEVEL_SAMPLES > 3U
#error "Update Buddy 4 level confirmation storage for this setting"
#endif

#define HS_IDLE			0U
#define HS_CLIMB		1U
#define HS_DESCENT		2U
#define HS_ROLL_WAIT_LEVEL	3U	/* rejected roll candidate: rearm only on level */

typedef struct S4_LOCAL_BITS {

	W	pre_elev_um;	/* elevation in consecutive start samples */
	W	elev_um;
	W	peak_um;
	W	travel_um;
	W	last_height_um;
	W	max_height_um;	/* highest hump of the run, full resolution */
	W	last_len_mm;


	/* Local bounded fields, never serialized as a C layout. */
	unsigned int state : 2;
	unsigned int rise_count : 2;
	unsigned int level_count : 2;
	unsigned int count : 8;		/* humps reported (saturating) */
} HUMP;

typedef struct S4_LOCAL_BITS {

	W	height_um;
	W	length_mm;

	/* Local bounded fields, never serialized as a C layout. */
	unsigned int completed : 1;	/* a hump was measured */
	unsigned int rejected : 1;	/* candidate discarded */
} HUMP_EVT;

IMPORT void hump_init(HUMP *h);
/* One sample: filtered pitch and signed encoder displacement since the
   previous sample.  Reverse motion rejects an active candidate because
   its forward height integral is no longer valid. */
IMPORT void hump_step(HUMP *h, W pitch_cdeg, W ds_um, BOOL valid, HUMP_EVT *ev);
/* One wheel on a hump, opposite wheel on the original level surface.
   Height is physical wheel-centre span * sin(abs(roll)); distance gates
   motion and length, rather than integrating roll along the road. */
IMPORT void hump_roll_step(HUMP *h, W roll_cdeg, W ds_um, BOOL valid, HUMP_EVT *ev);
IMPORT BOOL hump_active(const HUMP *h);

/* Storage/transmission code: round to HUMP_STEP_MM, saturate at the top
   code; *saturated set when the height exceeded the range */
IMPORT UB   hump_code(W height_um, BOOL *saturated);
IMPORT W    hump_code_to_um(UB code);	/* -1 for HUMP_CODE_NONE */

/* The stored/transmitted hump status (s4_status.h) of a HUMP record */
IMPORT void hump_status(const HUMP *h, S4_HUMP_STATUS *s);

#endif /* HUMP_H */
