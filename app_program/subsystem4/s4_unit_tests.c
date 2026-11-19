/*
 *	s4_unit_tests.c
 *	Unit tests of subsystem 4's pure logic: integer maths, calibration,
 *	filtering, tilt, collision, motion classification, data validation,
 *	hump detection / height / maximum, quantisation and packing.
 *	Runs on the host (host_test/) and in the on-target test image.
 *	Test IDs: S4-Uxx.
 */

#include <tk/typedef.h>

#if APP_TEST || HOST_TEST

#include "../test_report.h"
#include "imu_proc.h"
#include "hump.h"
#include "s4_status.h"
#include "s4_tests.h"

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

LOCAL UH sample_no;

EXPORT void s4_make_in(IMU_IN *s, W ax, W ay, W az, W gx, W gy, W gz)
{
	s->acc_mg[0] = ax;
	s->acc_mg[1] = ay;
	s->acc_mg[2] = az;
	s->gyr_mdps[0] = gx;
	s->gyr_mdps[1] = gy;
	s->gyr_mdps[2] = gz;
	s->mag_mgauss[0] = 0;
	s->mag_mgauss[1] = 0;
	s->mag_mgauss[2] = 0;
	s->has_gyro = TRUE;
	s->mag_valid = FALSE;
	s->saturated = FALSE;
	s->raw_hash = ++sample_no;		/* a live sensor always changes */
}

/* LSM303DLHC-style sample: accelerometer + magnetometer, no gyro */
LOCAL void make_in_nogyro(IMU_IN *s, W ax, W ay, W az, W mx, W my, W mz)
{
	s4_make_in(s, ax, ay, az, 0, 0, 0);
	s->mag_mgauss[0] = mx;
	s->mag_mgauss[1] = my;
	s->mag_mgauss[2] = mz;
	s->has_gyro = FALSE;
	s->mag_valid = TRUE;
}

/* Car on level ground facing yaw_cdeg (CCW from the start direction),
   pitched nose-up by pitch_cdeg; earth field H horizontal, V vertical */
#define FIELD_H		400
#define FIELD_V		(-300)
LOCAL void make_pose(IMU_IN *s, W pitch_cdeg, W yaw_cdeg)
{
	W	sp = imath_sin_q15(pitch_cdeg), cp = imath_sin_q15(pitch_cdeg + 9000);
	W	hf = FIELD_H * imath_sin_q15(yaw_cdeg + 9000) / 32768;
	W	hr = FIELD_H * imath_sin_q15(yaw_cdeg) / 32768;

	make_in_nogyro(s, 0, 1000 * sp / 32768, 1000 * cp / 32768,
		       hr, (hf * cp + FIELD_V * sp) / 32768, (FIELD_V * cp - hf * sp) / 32768);
}

/* Pitch along a raised-cosine hump of height h_mm and length l_mm at
   distance s_mm into it (point model: pitch = atan(dh/ds)) */
EXPORT W s4_hump_pitch_cdeg(W h_mm, W l_mm, W s_mm)
{
	W	ang = (W)((D)36000 * s_mm / l_mm);			/* 2*pi*s/L */
	W	slope_milli = (W)((D)h_mm * 3142 * imath_sin_q15(ang) / ((D)l_mm * 32768));

	return imath_atan2_cdeg(slope_milli, 1000);
}

/* Drive over one hump at 200 mm/s (2 mm per 10 ms sample) with level
   track before and after; returns the event of the run */
EXPORT void s4_drive_hump(HUMP *h, W h_mm, W l_mm, HUMP_EVT *last)
{
	HUMP_EVT	ev;
	W		s;

	last->completed = FALSE;
	last->rejected = FALSE;
	for (s = -100; s < l_mm + 100; s += 2) {
		W	p = (s < 0 || s > l_mm) ? 0 : s4_hump_pitch_cdeg(h_mm, l_mm, s);

		hump_step(h, p, 2000, TRUE, &ev);
		if (ev.completed || ev.rejected) {
			*last = ev;
		}
	}
}

LOCAL INT test_math(void)
{
	TEST_INIT();
	TEST_CHECK("S4-U01", imath_isqrt(0) == 0 && imath_isqrt(1) == 1 && imath_isqrt(3) == 1
		   && imath_isqrt(4) == 2 && imath_isqrt(65535) == 255 && imath_isqrt(65536) == 256
		   && imath_isqrt(0xFFFFFFFFUL) == 65535,
		   "integer square root exact (floor) incl. 32-bit maximum");
	TEST_CHECK("S4-U02", imath_sin_q15(0) == 0 && iabs(imath_sin_q15(3000) - 16384) < 70
		   && iabs(imath_sin_q15(9000) - 32768) < 70 && iabs(imath_sin_q15(-3000) + 16384) < 70
		   && iabs(imath_sin_q15(18000)) < 70 && iabs(imath_sin_q15(27000) + 32768) < 70,
		   "sine within 0.002 at 0/30/90/-30/180/270 deg");
	TEST_CHECK("S4-U03", imath_atan2_cdeg(0, 1000) == 0 && iabs(imath_atan2_cdeg(1000, 1000) - 4500) < 30
		   && imath_atan2_cdeg(1000, 0) == 9000 && iabs(imath_atan2_cdeg(-1000, -1000) + 13500) < 30
		   && iabs(imath_atan2_cdeg(173648, 984808) - 1000) < 30,
		   "atan2 within 0.3 deg (0, 45, 90, -135, 10 deg)");
	TEST_SUMMARY("s4 math");
	return test_fail_;
}

LOCAL INT test_calibration(void)
{
	IMU_CAL_ACC	c;
	IMU_CAL		cal;
	IMU_IN		s;
	UINT		k;
	INT		r;

	TEST_INIT();
	imu_cal_reset(&c);
	for (k = 0; k < IMU_CAL_SAMPLES; k++) {
		W	n = (k & 1U) ? 5 : -5;

		s4_make_in(&s, 12 + n, -8 - n, 1003 + n, 150 + 40 * n, -220 - 40 * n, 80);
		imu_cal_add(&c, &s);
	}
	r = imu_cal_finish(&c, &cal);
	TEST_CHECK("S4-U04", r == IMU_CAL_OK && cal.valid && iabs(cal.gyro_bias[0] - 150) <= 2
		   && iabs(cal.gyro_bias[1] + 220) <= 2 && iabs(cal.gyro_bias[2] - 80) <= 2
		   && iabs(cal.acc_offset[0] - 12) <= 1 && iabs(cal.acc_offset[1] + 8) <= 1
		   && iabs(cal.acc_offset[2] - 3) <= 1,
		   "stationary calibration recovers gyro bias and level offsets");

	imu_cal_reset(&c);
	for (k = 0; k < IMU_CAL_SAMPLES; k++) {
		s4_make_in(&s, 0, 0, 1000, (k & 1U) ? 3000 : -3000, 0, 0);
		imu_cal_add(&c, &s);
	}
	TEST_CHECK("S4-U05", imu_cal_finish(&c, &cal) == IMU_CAL_NOT_STILL && !cal.valid,
		   "car moving during calibration: rejected (retry)");

	imu_cal_reset(&c);
	for (k = 0; k < IMU_CAL_SAMPLES; k++) {
		s4_make_in(&s, 1000, 0, 0, 0, 0, 0);
		imu_cal_add(&c, &s);
	}
	TEST_CHECK("S4-U06", imu_cal_finish(&c, &cal) == IMU_CAL_ORIENTATION,
		   "gravity not on the configured up axis: calibration failure");

	imu_cal_reset(&c);
	TEST_CHECK("S4-U07", imu_cal_finish(&c, &cal) == IMU_CAL_NO_DATA, "no samples: failure");
	TEST_SUMMARY("s4 calibration");
	return test_fail_;
}

LOCAL void level_cal(IMU_CAL *cal)
{
	UINT	i;

	for (i = 0; i < 3U; i++) {
		cal->gyro_bias[i] = 0;
		cal->acc_offset[i] = 0;
	}
	cal->valid = TRUE;
}

LOCAL INT test_monitoring(void)
{
	IMU_PROC	p;
	IMU_CAL		cal;
	IMU_IN		s;
	UINT		k;
	BOOL		tilt_early, coll1, coll2;
	W		max_dev = 0;

	TEST_INIT();
	level_cal(&cal);

	/* static 10 deg nose-up: gravity (fwd 174 mg, up 985 mg) */
	imu_proc_init(&p, &cal);
	for (k = 0; k < 300U; k++) {
		s4_make_in(&s, 0, 174, 985, 0, 0, 0);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U08", iabs(p.pitch_cdeg - 1000) <= 50 && iabs(p.roll_cdeg) <= 50,
		   "static 10 deg pitch measured within 0.5 deg");

	/* one acceleration spike while level must not move the pitch much */
	imu_proc_init(&p, &cal);
	for (k = 0; k < 50U; k++) {
		s4_make_in(&s, 0, (k == 25U) ? 600 : 0, 1000, 0, 0, 0);
		(void)imu_proc_step(&p, &s, 10);
		if (iabs(p.pitch_cdeg) > max_dev) {
			max_dev = iabs(p.pitch_cdeg);
		}
	}
	TEST_CHECK("S4-U09", max_dev <= 100, "short braking/bump spike: pitch error < 1 deg (filter)");

	/* tilt needs > 15 deg for >= 200 ms */
	imu_proc_init(&p, &cal);
	for (k = 0; k < 15U; k++) {
		s4_make_in(&s, -342, 0, 940, 0, 0, 0);		/* 20 deg roll */
		(void)imu_proc_step(&p, &s, 10);
	}
	tilt_early = p.tilt;
	for (k = 0; k < 10U; k++) {
		s4_make_in(&s, -342, 0, 940, 0, 0, 0);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U10", !tilt_early && p.tilt && iabs(p.roll_cdeg - 2000) < 100,
		   "tilt event only after 200 ms beyond 15 deg (no false tilt)");

	/* collision: 2.5 g horizontal impact, refractory afterwards */
	imu_proc_init(&p, &cal);
	s4_make_in(&s, 0, 0, 1000, 0, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	s4_make_in(&s, 0, -2500, 1000, 0, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	coll1 = p.collision;
	s4_make_in(&s, 0, -2500, 1000, 0, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	coll2 = p.collision;
	TEST_CHECK("S4-U11", coll1 && !coll2 && p.peak_shock_mg >= 2400,
		   "2.5 g impact detected once (refractory), peak recorded");
	imu_proc_init(&p, &cal);
	s4_make_in(&s, 0, 0, 1000, 0, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	s4_make_in(&s, 0, 900, 1000, 0, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	TEST_CHECK("S4-U12", !p.collision, "0.9 g bump below the collision threshold");

	/* motion classification */
	imu_proc_init(&p, &cal);
	for (k = 0; k < 40U; k++) {
		s4_make_in(&s, 0, 0, 1000, 0, 0, 0);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U13", p.motion_class == MCLS_STATIONARY, "still for 300 ms: STATIONARY");
	s4_make_in(&s, 0, 0, 1000, 0, 0, 30000);
	(void)imu_proc_step(&p, &s, 10);
	TEST_CHECK("S4-U14", p.motion_class == MCLS_TURN_LEFT && p.yaw_rate_mdps == 30000,
		   "+30 dps yaw: TURN_LEFT, turn rate reported");
	s4_make_in(&s, 0, 0, 1000, 0, 0, -30000);
	(void)imu_proc_step(&p, &s, 10);
	TEST_CHECK("S4-U15", p.motion_class == MCLS_TURN_RIGHT, "-30 dps yaw: TURN_RIGHT");
	s4_make_in(&s, 0, 200, 1000, 4000, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	TEST_CHECK("S4-U16", p.motion_class == MCLS_MOVING, "vibration/acceleration: MOVING");

	/* data validation: stuck sensor and saturation */
	imu_proc_init(&p, &cal);
	for (k = 0; k < 40U; k++) {
		s4_make_in(&s, 0, 0, 1000, 0, 0, 0);
		s.raw_hash = 1234;			/* bus returns the same bytes */
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U17", imu_proc_failed(&p), "frozen raw data -> invalid run -> IMU fault");
	imu_proc_init(&p, &cal);
	s4_make_in(&s, 0, 0, 1000, 0, 0, 0);
	s.saturated = TRUE;
	TEST_CHECK("S4-U18", !imu_proc_step(&p, &s, 10) && p.invalid_run == 1,
		   "saturated sample rejected, never used");
	TEST_SUMMARY("s4 monitoring");
	return test_fail_;
}

/* LSM303DLHC path: accelerometer attitude, magnetometer heading */
LOCAL INT test_nogyro(void)
{
	IMU_PROC	p;
	IMU_CAL		cal;
	IMU_IN		s;
	HUMP		h;
	HUMP_EVT	ev, last;
	UINT		k;
	W		max_dev = 0, yaw;
	BOOL		tilt_early, coll1, coll2, ok;

	TEST_INIT();
	level_cal(&cal);

	imu_proc_init(&p, &cal);
	for (k = 0; k < 100U; k++) {
		make_in_nogyro(&s, 0, 174, 985, 0, 400, -300);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U33", iabs(p.pitch_cdeg - 1000) <= 50 && iabs(p.roll_cdeg) <= 50,
		   "no gyro: static 10 deg pitch from the accelerometer within 0.5 deg");

	imu_proc_init(&p, &cal);
	for (k = 0; k < 50U; k++) {
		make_in_nogyro(&s, 0, (k == 25U) ? 600 : 0, 1000, 0, 400, -300);
		(void)imu_proc_step(&p, &s, 10);
		if (iabs(p.pitch_cdeg) > max_dev) {
			max_dev = iabs(p.pitch_cdeg);
		}
	}
	TEST_CHECK("S4-U34", max_dev <= 100, "no gyro: single bump spike rejected, pitch error < 1 deg");

	imu_proc_init(&p, &cal);
	for (k = 0; k < 15U; k++) {
		make_in_nogyro(&s, -342, 0, 940, 0, 400, -300);	/* 20 deg roll */
		(void)imu_proc_step(&p, &s, 10);
	}
	tilt_early = p.tilt;
	for (k = 0; k < 25U; k++) {
		make_in_nogyro(&s, -342, 0, 940, 0, 400, -300);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U35", !tilt_early && p.tilt && iabs(p.roll_cdeg - 2000) < 100,
		   "no gyro: tilt only after the filter settles and 200 ms beyond 15 deg");

	imu_proc_init(&p, &cal);
	make_in_nogyro(&s, 0, 0, 1000, 0, 400, -300);
	(void)imu_proc_step(&p, &s, 10);
	make_in_nogyro(&s, 0, -2500, 1000, 0, 400, -300);
	(void)imu_proc_step(&p, &s, 10);
	coll1 = p.collision;
	make_in_nogyro(&s, 0, -2500, 1000, 0, 400, -300);
	(void)imu_proc_step(&p, &s, 10);
	coll2 = p.collision;
	TEST_CHECK("S4-U36", coll1 && !coll2 && p.peak_shock_mg >= 2400,
		   "no gyro: 2.5 g impact detected once, peak recorded");

	/* 90 deg left turn at 30 dps: 0.3 deg per 10 ms sample */
	imu_proc_init(&p, &cal);
	for (k = 0; k <= 300U; k++) {
		make_pose(&s, 0, (W)k * 30);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U37", p.yaw_valid && iabs(p.yaw_rate_mdps - 30000) <= 3000
		   && p.motion_class == MCLS_TURN_LEFT && iabs(p.yaw_cdeg - 9000) <= 200,
		   "no gyro: left turn from the magnetometer, rate within 3 dps, heading within 2 deg");
	for (k = 1; k <= 100U; k++) {
		make_pose(&s, 0, 9000 - (W)k * 30);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U38", iabs(p.yaw_rate_mdps + 30000) <= 3000
		   && p.motion_class == MCLS_TURN_RIGHT,
		   "no gyro: right turn gives a negative yaw rate");

	/* heading must not move when the car pitches onto a hump: 10 deg in
	   100 ms (100 deg/s, steeper than the 40 mm hump at 200 mm/s) */
	imu_proc_init(&p, &cal);
	ok = TRUE;
	for (k = 0; k < 130U; k++) {
		W	pc = (k < 30U) ? 0 : (k < 40U) ? ((W)k - 30) * 100 : 1000;

		make_pose(&s, pc, 3000);
		(void)imu_proc_step(&p, &s, 10);
		yaw = (p.yaw_cdeg > 18000) ? p.yaw_cdeg - 36000 : p.yaw_cdeg;
		ok = ok && iabs(yaw) <= 200 && p.motion_class != MCLS_TURN_LEFT
		     && p.motion_class != MCLS_TURN_RIGHT;
	}
	TEST_CHECK("S4-U39", ok, "no gyro: heading tilt-compensated, climbing 10 deg moves it < 2 deg, no false turn");

	imu_proc_init(&p, &cal);
	make_pose(&s, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	make_pose(&s, 0, 0);
	s.mag_valid = FALSE;				/* magnetometer overflow */
	ok = imu_proc_step(&p, &s, 10) && !p.yaw_valid && p.invalid_run == 0;
	make_pose(&s, 0, 0);
	(void)imu_proc_step(&p, &s, 10);
	TEST_CHECK("S4-U40", ok && p.yaw_valid,
		   "no gyro: magnetometer overflow invalidates heading only, not the IMU");

	imu_proc_init(&p, &cal);
	for (k = 0; k < 40U; k++) {
		make_pose(&s, 0, 0);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U41", p.motion_class == MCLS_STATIONARY, "no gyro: still for 300 ms: STATIONARY");

	/* end to end: 40 mm hump through the accelerometer filter (S4-U19) */
	imu_proc_init(&p, &cal);
	hump_init(&h);
	last.completed = FALSE;
	ok = TRUE;
	for (yaw = -100; yaw < 400; yaw += 2) {
		W	pc = (yaw < 0 || yaw > 300) ? 0 : s4_hump_pitch_cdeg(40, 300, yaw);

		make_pose(&s, pc, 4500);
		(void)imu_proc_step(&p, &s, 10);
		hump_step(&h, p.pitch_cdeg, 2000, TRUE, &ev);
		if (ev.completed) {
			last = ev;
		}
		ok = ok && p.motion_class != MCLS_TURN_LEFT && p.motion_class != MCLS_TURN_RIGHT;
	}
	TEST_CHECK("S4-U42", last.completed && iabs(last.height_um - 40000) <= 4000 && h.count == 1 && ok,
		   "no gyro: 40 mm hump through the accelerometer filter within +-4 mm, no false turn");

	/* same hump at the 30 mm/s cruise: 0.3 mm per 10 ms sample */
	imu_proc_init(&p, &cal);
	hump_init(&h);
	last.completed = FALSE;
	for (yaw = -1000; yaw < 4000; yaw += 3) {		/* 0.1 mm units */
		W	pc = (yaw < 0 || yaw > 3000) ? 0 : s4_hump_pitch_cdeg(40, 300, yaw / 10);

		make_pose(&s, pc, 0);
		(void)imu_proc_step(&p, &s, 10);
		hump_step(&h, p.pitch_cdeg, 300, TRUE, &ev);
		if (ev.completed) {
			last = ev;
		}
	}
	TEST_CHECK("S4-U43", last.completed && iabs(last.height_um - 40000) <= 4000 && h.count == 1,
		   "no gyro: 40 mm hump at 30 mm/s (0.3 mm per sample) within +-4 mm");
	imu_proc_init(&p, &cal);
	for (k = 0; k < 100U; k++) {
		make_in_nogyro(&s, 0, 0, 1000, 0, 400, -300);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U50", !p.handling, "level 1g remains normal; gentle level lifting cannot be inferred");
	for (k = 0; k < 10U; k++) {
		make_in_nogyro(&s, 0, 0, 1600, 0, 400, -300);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U51", p.handling, "sustained vertical pickup jolt triggers handling stop");
	imu_proc_init(&p, &cal);
	for (k = 0; k < 8U; k++) {
		make_in_nogyro(&s, 0, 0, 200, 0, 400, -300);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U52", p.handling, "sustained low-g/free-fall triggers handling stop");
	imu_proc_init(&p, &cal);
	for (k = 0; k < 60U; k++) {
		make_in_nogyro(&s, -1000, 0, 0, 0, 400, -300);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U53", p.handling && p.tilt, "robot held on its side triggers severe-tilt handling stop");
	imu_proc_init(&p, &cal);
	for (k = 0; k < 60U; k++) {
		make_in_nogyro(&s, -342, 0, 940, 0, 400, -300);
		(void)imu_proc_step(&p, &s, 10);
	}
	TEST_CHECK("S4-U54", !p.handling, "ordinary 20-degree hump tilt does not trigger severe handling stop");
	/* Production path: whole-mm odometry, published every 20 ms, read
	   by a 10 ms IMU task. At 30 mm/s most increments are zero. */
	imu_proc_init(&p, &cal);
	hump_init(&h);
	last.completed = FALSE;
	{
		W previous_mm = -100, published_mm = -100;
		UINT tick = 0;
		for (yaw = -1000; yaw < 4000; yaw += 3, tick++) {
			W pc = (yaw < 0 || yaw > 3000) ? 0 : s4_hump_pitch_cdeg(40, 300, yaw / 10);
			W delta_mm;
			if ((tick & 1U) == 0U) published_mm = yaw / 10;
			delta_mm = published_mm - previous_mm;
			previous_mm = published_mm;
			make_pose(&s, pc, 0);
			(void)imu_proc_step(&p, &s, 10);
			hump_step(&h, p.pitch_cdeg, delta_mm * 1000, TRUE, &ev);
			if (ev.completed) last = ev;
		}
	}
	TEST_CHECK("S4-U55", last.completed && h.count == 1 && iabs(last.height_um - 40000) <= 4000,
		   "30 mm/s hump survives real 20 ms whole-mm odometry sampled at 10 ms");
	imu_proc_init(&p, &cal);
	hump_init(&h);
	last.completed = FALSE;
	for (k = 0; k < 240U; k++) {
		S4_HUMP_SAMPLE packed, unpacked;
		W roll = k < 40U ? 0 : k < 80U ? (W)(k - 40U) * 50
			: k < 140U ? 2000 : k < 180U ? (W)(180U - k) * 50 : 0;
		make_in_nogyro(&s, -1000 * imath_sin_q15(roll) / 32768, 0,
			1000 * imath_sin_q15(roll + 9000) / 32768, 0, 400, -300);
		(void)imu_proc_step(&p, &s, 10);
		packed.tilt_ddeg = (H)(p.roll_cdeg / 10);
		packed.ds_100um = (k % 4U == 0U) ? 10U : 0U;
		packed.valid = 1;
		packed.seq = (UB)(k & 15U);
		s4_sample_unpack(s4_sample_pack(&packed), &unpacked);
		hump_roll_step(&h, (W)unpacked.tilt_ddeg * 10,
			(W)unpacked.ds_100um * 100, unpacked.valid, &ev);
		if (ev.completed) last = ev;
	}
	TEST_CHECK("S4-U64", last.completed && h.count == 1 && iabs(last.height_um - 37622) < 1500,
		   "LSM303 roll filter and packed queue measure a one-wheel hump at slow speed");
	TEST_SUMMARY("s4 no gyro");
	return test_fail_;
}

LOCAL INT test_hump(void)
{
	HUMP		h;
	HUMP_EVT	ev, e2;
	W		s;
	BOOL		any = FALSE, sat;
	UINT		k;

	TEST_INIT();
	hump_init(&h);
	s4_drive_hump(&h, 40, 300, &ev);
	TEST_CHECK("S4-U19", ev.completed && iabs(ev.height_um - 40000) <= 4000 && h.count == 1,
		   "40 mm hump measured within +-4 mm (Tribunal 1 success criterion)");

	hump_init(&h);
	s4_drive_hump(&h, 20, 250, &ev);
	s4_drive_hump(&h, 40, 300, &ev);
	s4_drive_hump(&h, 30, 300, &e2);
	TEST_CHECK("S4-U20", h.count == 3 && iabs(h.max_height_um - 40000) <= 4000
		   && iabs(h.last_height_um - 30000) <= 4000,
		   "highest hump of the run tracked at full resolution");

	hump_init(&h);
	for (s = 0; s < 3000; s += 2) {
		W	noise = ((s / 2) % 7 - 3) * 40;		/* +-1.2 deg vibration */

		hump_step(&h, noise, 2000, TRUE, &ev);
		any = any || ev.completed;
	}
	TEST_CHECK("S4-U21", !any && h.count == 0, "flat track with vibration: no hump");

	hump_init(&h);
	any = FALSE;
	for (s = 0; s < 1600; s += 2) {
		hump_step(&h, 500, 2000, TRUE, &ev);		/* 5 deg ramp */
		any = any || ev.completed;
	}
	TEST_CHECK("S4-U22", !any && h.count == 0, "long ramp (> 1 m) is not reported as a hump");

	hump_init(&h);
	any = FALSE;
	for (s = 0; s < 400; s += 2) {
		W	p = (s > 300) ? 0 : s4_hump_pitch_cdeg(40, 300, s);

		hump_step(&h, p, 2000, (s != 150) ? TRUE : FALSE, &ev);
		any = any || ev.completed;
	}
	TEST_CHECK("S4-U23", !any, "invalid sample mid-hump: measurement discarded");

	TEST_CHECK("S4-U24", hump_code(0, &sat) == 0 && hump_code(999, &sat) == 0
		   && hump_code(1000, &sat) == 1 && hump_code(100000, &sat) == HUMP_CODE_MAX && !sat
		   && hump_code(101000, &sat) == HUMP_CODE_MAX && sat
		   && hump_code(-1, &sat) == HUMP_CODE_NONE
		   && hump_code_to_um(50) == 100000 && hump_code_to_um(HUMP_CODE_NONE) == -1,
		   "6-bit hump code: 2 mm steps, saturation flag, 63 = none");
	sat = FALSE;
	for (k = 0; k <= 100000; k += 250) {
		if (iabs(hump_code_to_um(hump_code((W)k, &sat)) - (W)k) > 1000) {
			sat = TRUE;
			break;
		}
		sat = FALSE;
	}
	TEST_CHECK("S4-U25", !sat, "quantisation error <= 1 mm over 0..100 mm");

	/* A shallow positive road grade before a candidate must not be added
	   to that candidate's height. */
	hump_init(&h);
	for (s = 0; s < 1000; s += 2) {
		hump_step(&h, 200, 2000, TRUE, &ev);	/* 2 deg for one metre */
	}
	hump_step(&h, 300, 2000, TRUE, &ev);
	hump_step(&h, 300, 2000, TRUE, &ev);
	TEST_CHECK("S4-U44", h.state == HS_CLIMB && h.elev_um < 1000 && h.travel_um == 4000,
		   "shallow road grade is not accumulated into a later hump candidate");

	/* Reversing changes the traversal path, so a partly integrated hump
	   must be discarded instead of being resumed later. */
	hump_step(&h, 300, -1000, TRUE, &ev);
	TEST_CHECK("S4-U45", ev.rejected && !hump_active(&h),
		   "reverse motion rejects an active forward hump measurement");
	hump_init(&h);
	for (k = 0; k < 400U; k++) hump_step(&h, 800, 0, TRUE, &ev);
	TEST_CHECK("S4-U56", !hump_active(&h) && h.count == 0,
		   "sustained uphill tilt without wheel travel cannot start a hump");
	hump_step(&h, 800, 1000, TRUE, &ev);
	TEST_CHECK("S4-U57", hump_active(&h) && h.elev_um > 0 && h.travel_um == 1000,
		   "confirmed uphill tilt starts with actual forward travel after zero increments");
	{
		W sign;
		BOOL both_sides = TRUE;
		for (sign = -1; sign <= 1; sign += 2) {
			hump_init(&h);
			any = FALSE;
			for (k = 0; k < 180U; k++) {
				W roll = k < 20U ? 0 : k < 60U ? (W)(k - 20U) * 50
					: k < 80U ? 2000 : k < 120U ? (W)(120U - k) * 50 : 0;
				hump_roll_step(&h, sign * roll, (k % 4U == 0U) ? 1000 : 0, TRUE, &ev);
				any = any || ev.completed;
			}
			both_sides = both_sides && any && h.count == 1 && iabs(h.last_height_um - 37622) < 600;
		}
		TEST_CHECK("S4-U58", both_sides,
			   "one-wheel 20-degree roll on either side measures about 37.6 mm with sparse odometry");
	}
	hump_init(&h);
	for (k = 0; k < 100U; k++) hump_roll_step(&h, 2000, 0, TRUE, &ev);
	TEST_CHECK("S4-U59", !hump_active(&h) && h.count == 0,
		   "stationary sideways tilt cannot start a terrain measurement");
	for (k = 0; k < 5U; k++) hump_roll_step(&h, 2000, 1000, TRUE, &ev);
	TEST_CHECK("S4-U60", hump_active(&h) && h.count == 0 && h.peak_um > 37000,
		   "one-wheel height tracks peak but waits for level before publishing a completed hump");
	hump_roll_step(&h, 2000, 1000, FALSE, &ev);
	any = ev.rejected;
	for (k = 0; k < 10U; k++) hump_roll_step(&h, 2000, 1000, TRUE, &ev);
	for (k = 0; k < 5U; k++) hump_roll_step(&h, 0, 1000, TRUE, &ev);
	TEST_CHECK("S4-U61", any && h.count == 0 && !hump_active(&h),
		   "lost IMU sample discards the side hump and blocks partial remeasurement until level");
	hump_init(&h);
	for (k = 0; k < 10U; k++) hump_roll_step(&h, 2000, 1000, TRUE, &ev);
	hump_roll_step(&h, 9000, 1000, TRUE, &ev);
	TEST_CHECK("S4-U62", ev.rejected && h.count == 0 && !hump_active(&h),
		   "overturn/severe tilt is not reported as a hump");
	hump_init(&h);
	for (k = 0; k < 1200U; k++) hump_roll_step(&h, 2000, 1000, TRUE, &ev);
	for (k = 0; k < 5U; k++) hump_roll_step(&h, 0, 1000, TRUE, &ev);
	TEST_CHECK("S4-U63", h.count == 0 && !hump_active(&h),
		   "long sideways slope is rejected without rearming before level");
	TEST_SUMMARY("s4 hump");
	return test_fail_;
}

LOCAL INT test_packing(void)
{
	S4_IMU_STATUS	a, b;
	S4_HUMP_STATUS	c, d;
	S4_HUMP_SAMPLE	e, f;
	UW		w[2];

	TEST_INIT();
	a.pitch_deg = -90;
	a.roll_deg = S4_ANGLE_INVALID;
	a.yaw_rate_dps = S4_RATE_INVALID;
	a.motion_class = MCLS_TURN_RIGHT;
	a.status = IMU_ST_FAULT;
	a.tilt = 1;
	a.collision = 1;
	a.valid = 0;
	a.heading_deg = 359;
	a.shock_half_g = 31;
	a.seq = 255;
	a.chip = 2;
	a.calibrated = 1;
	s4_imu_pack(&a, w);
	s4_imu_unpack(w, &b);
	TEST_CHECK("S4-U26", b.pitch_deg == -90 && b.roll_deg == S4_ANGLE_INVALID
		   && b.yaw_rate_dps == S4_RATE_INVALID && b.motion_class == MCLS_TURN_RIGHT
		   && b.status == IMU_ST_FAULT && b.tilt == 1 && b.collision == 1 && b.valid == 0
		   && b.heading_deg == 359 && b.shock_half_g == 31 && b.seq == 255
		   && b.chip == 2 && b.calibrated == 1,
		   "IMU status words round trip at limits and invalid codes");
	c.last_code = HUMP_CODE_NONE;
	c.max_code = 50;
	c.count = 200;
	c.active = 1;
	c.last_sat = 0;
	c.max_sat = 1;
	c.last_len_cm = 255;
	s4_hump_unpack(s4_hump_pack(&c), &d);
	TEST_CHECK("S4-U27", d.last_code == HUMP_CODE_NONE && d.max_code == 50 && d.count == 15
		   && d.active == 1 && d.max_sat == 1 && d.last_len_cm == 255,
		   "hump status word (count saturates at 15)");
	e.tilt_ddeg = -2000;
	e.ds_100um = 255;
	e.valid = 1;
	e.seq = 15;
	s4_sample_unpack(s4_sample_pack(&e), &f);
	TEST_CHECK("S4-U28", f.tilt_ddeg == -1023 && f.ds_100um == 255 && f.valid == 1 && f.seq == 15,
		   "IMU->hump queue message: pitch clamped to 11-bit range");
	TEST_CHECK("S4-U29", s4_deg8(-9049) == -90 && s4_deg8(9051) == 90 && s4_deg8(149) == 1
		   && s4_rate9(300000) == 255 && s4_rate9(-1499) == -1 && s4_shock5(20000) == 31
		   && s4_shock5(1250) == 3,
		   "compaction rounds to nearest and saturates");
	TEST_SUMMARY("s4 packing");
	return test_fail_;
}

EXPORT INT s4_run_unit_tests(void)
{
	return test_math() + test_calibration() + test_monitoring() + test_nogyro()
	     + test_hump() + test_packing();
}

#endif /* APP_TEST || HOST_TEST */
