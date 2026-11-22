/*
 *	task_hump.c
 *	Hump & Terrain Analysis task (Week 6: Medium, event-driven by the
 *	bounded queue mbf_imu_hump).  It never touches the IMU or the encoder
 *	hardware and never affects motion (Tribunal 1 interface map).
 *
 *	A gap in the sample sequence (queue overflow) or an invalid sample
 *	discards the hump in progress rather than measuring across missing
 *	data (Tribunal 1: "consumers ignore invalid or stale samples").
 */

#include "subsystem4.h"

EXPORT volatile UW	s4_hump_word = (UW)HUMP_CODE_NONE | ((UW)HUMP_CODE_NONE << 6);
EXPORT volatile UW	s4_max_hump = (UW)-1;

LOCAL HUMP	hump;

EXPORT void s4_get_hump_status(S4_HUMP_STATUS *s)
{
	s4_hump_unpack(app_load(&s4_hump_word), s);
}

EXPORT W s4_max_hump_um(void)
{
	return (W)app_load(&s4_max_hump);
}

LOCAL void publish(void)
{
	S4_HUMP_STATUS	s;

	hump_status(&hump, &s);
	app_store(&s4_hump_word, s4_hump_pack(&s));
	app_store(&s4_max_hump, (UW)hump.max_height_um);
}

EXPORT void hump_task(INT stacd, void *exinf)
{
	S4_HUMP_SAMPLE	s;
	HUMP_EVT	ev;
	UW		w;
	struct {
	 unsigned char expect : 4;
	 unsigned char first : 1;
	 unsigned char was_active : 1;
	} sequence = {0, TRUE, FALSE};

	(void)stacd;
	(void)exinf;
	hump_init(&hump);
	sys_task_ready(SYS_TASK_HUMP);
	publish();
	tm_printf((UB*)"hump: %s mode, physical wheel span %d mm\n",
		  HUMP_SIDEWAYS ? "sideways roll" : "forward pitch", HUMP_PHYSICAL_TRACK_MM);

	for (;;) {
		INT	n = tk_rcv_mbf(mbf_imu_hump, &w, TMO_FEVR);
		BOOL	valid;
		UB	previous_state;
		W	candidate_peak;

		if (n != (INT)sizeof(w)) {
			continue;
		}
		s4_sample_unpack(w, &s);
		valid = (s.valid != 0U) ? TRUE : FALSE;
		if (!sequence.first && s.seq != sequence.expect) {
			valid = FALSE;			/* samples were dropped */
		}
		sequence.first = FALSE;
		sequence.expect = (UB)((s.seq + 1U) & 0xFU);

		previous_state = hump.state;
		candidate_peak = hump.peak_um;
		if (HUMP_SIDEWAYS)
			hump_roll_step(&hump, (W)s.tilt_ddeg * 10, (W)s.ds_100um * 100, valid, &ev);
		else
			hump_step(&hump, (W)s.tilt_ddeg * 10, (W)s.ds_100um * 100, valid, &ev);
		if (previous_state == HS_IDLE && hump.state == HS_CLIMB) {
			tm_printf((UB*)"hump: %s climb started, tilt %d.%d deg\n",
				  HUMP_SIDEWAYS ? "sideways" : "forward",
				  (INT)(s.tilt_ddeg / 10),
				  (INT)((s.tilt_ddeg < 0 ? -s.tilt_ddeg : s.tilt_ddeg) % 10));
		} else if (previous_state == HS_CLIMB && hump.state == HS_DESCENT) {
			tm_printf((UB*)"hump: descending, peak %d.%d mm; waiting for level\n",
				  (INT)(hump.peak_um / 1000), (INT)((hump.peak_um % 1000) / 100));
		}
		if (ev.rejected) {
			tm_printf((UB*)"hump: discarded (%s), candidate peak %d.%d mm\n",
				  !valid ? "invalid/dropped IMU sample"
				  : (HUMP_SIDEWAYS && (s.tilt_ddeg >= IMU_HANDLE_TILT_CDEG / 10
				     || s.tilt_ddeg <= -IMU_HANDLE_TILT_CDEG / 10)) ? "severe tilt / handling"
				  : (candidate_peak < HUMP_MIN_HEIGHT_UM ? "below 3 mm" : "travel limit"),
				  (INT)(candidate_peak / 1000), (INT)((candidate_peak % 1000) / 100));
		}
		if (ev.completed) {
			tm_printf((UB*)"hump: height %d.%d mm, length %d mm, highest %d.%d mm\n",
				  (INT)(ev.height_um / 1000), (INT)((ev.height_um % 1000) / 100),
				  (INT)ev.length_mm,
				  (INT)(hump.max_height_um / 1000),
				  (INT)((hump.max_height_um % 1000) / 100));
		}
		if (ev.completed || ev.rejected || hump_active(&hump) != sequence.was_active) {
			sequence.was_active = hump_active(&hump);
			publish();
		}
	}
}
