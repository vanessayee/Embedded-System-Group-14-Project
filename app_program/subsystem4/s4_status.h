/*
 *	s4_status.h
 *	Packed application data of subsystem 4 (pure, host-testable).
 *	Masks and shifts only.  Full-resolution values stay inside the tasks;
 *	these are the stored/shared/transmitted forms (Tribunal 1, Buddy 4).
 *
 *	IMU status, word 0 (writer: IMU Monitoring task)
 *	    bits  0..7   pitch, deg      (8)  -90..90, -128 = invalid
 *	    bits  8..15  roll, deg       (8)  -90..90, -128 = invalid
 *	    bits 16..24  yaw rate, dps   (9)  -255..255 (+ = left), -256 = invalid
 *	    bits 25..26  motion class    (2)  MCLS_*
 *	    bits 27..28  IMU status      (2)  IMU_ST_*
 *	    bit  29      tilt            (1)
 *	    bit  30      collision latch (1)  set on impact, cleared on CLEAR
 *	    bit  31      sample valid    (1)
 *	IMU status, word 1
 *	    bits  0..8   heading, deg    (9)  0..359, 511 = invalid
 *	    bits  9..13  peak shock      (5)  0.5 g steps, 0..15.5 g
 *	    bits 14..21  sample seq      (8)  modulo 256 (freshness)
 *	    bits 22..23  chip family     (2)  IMU_CHIP_*
 *	    bit  24      calibrated      (1)
 *	A third word holds the publication time in ms (Tribunal 1: "latest
 *	cleaned sample shared as a timestamped snapshot with a validity flag").
 *
 *	Hump status word (writer: Hump & Terrain Analysis task)
 *	    bits  0..5   last height     (6)  2 mm steps 0..50, 63 = none
 *	    bits  6..11  highest height  (6)  2 mm steps 0..50, 63 = none
 *	    bits 12..15  hump count      (4)  saturating at 15
 *	    bit  16      hump in progress(1)
 *	    bit  17      last saturated  (1)  > 100 mm
 *	    bit  18      max saturated   (1)
 *	    bits 19..26  last length, cm (8)  0..255
 *
 *	IMU -> Hump queue message (24 bits in a 32-bit word)
 *	    bits  0..10  pitch           (11) 0.1 deg, -102.3..102.3
 *	    bits 11..18  distance step   (8)  0.1 mm, 0..25.5 mm per sample
 *	    bit  19      valid           (1)
 *	    bits 20..23  seq             (4)  detects dropped samples
 */

#ifndef S4_STATUS_H
#define S4_STATUS_H

#include <tk/typedef.h>
#include "s4_storage.h"

#define S4_ANGLE_INVALID	(-128)
#define S4_RATE_INVALID		(-256)
#define S4_HEADING_INVALID	511U

typedef struct S4_LOCAL_BITS {
	B	pitch_deg;
	B	roll_deg;
	H	yaw_rate_dps;

	UH	heading_deg;

	UW	time_ms;	/* publication time (separate word, not packed) */

	/* Local bounded fields, never serialized as a C layout. */
	unsigned int motion_class : 2;
	unsigned int status : 2;
	unsigned int tilt : 1;
	unsigned int collision : 1;
	unsigned int valid : 1;
	unsigned int shock_half_g : 5;
	unsigned int seq : 8;
	unsigned int chip : 2;
	unsigned int calibrated : 1;
} S4_IMU_STATUS;

typedef struct S4_LOCAL_BITS {


	/* Local bounded fields, never serialized as a C layout. */
	unsigned int last_code : 6;
	unsigned int max_code : 6;
	unsigned int count : 8;
	unsigned int active : 1;
	unsigned int last_sat : 1;
	unsigned int max_sat : 1;
	unsigned int last_len_cm : 8;
} S4_HUMP_STATUS;

typedef struct S4_LOCAL_BITS {


	/* Local bounded fields, never serialized as a C layout. */
	signed int tilt_ddeg : 16;	/* 0.1 deg: roll in sideways mode, pitch in forward mode */
	unsigned int ds_100um : 8;	/* 0.1 mm */
	unsigned int valid : 1;
	unsigned int seq : 4;
} S4_HUMP_SAMPLE;

IMPORT void s4_imu_pack(const S4_IMU_STATUS *s, UW w[2]);
IMPORT void s4_imu_unpack(const UW w[2], S4_IMU_STATUS *s);
IMPORT UW   s4_hump_pack(const S4_HUMP_STATUS *s);
IMPORT void s4_hump_unpack(UW w, S4_HUMP_STATUS *s);
IMPORT UW   s4_sample_pack(const S4_HUMP_SAMPLE *s);
IMPORT void s4_sample_unpack(UW w, S4_HUMP_SAMPLE *s);

/* Saturating conversions from full-resolution values */
IMPORT B    s4_deg8(W cdeg);
IMPORT H    s4_rate9(W mdps);
IMPORT UB   s4_shock5(W mg);

#endif /* S4_STATUS_H */
