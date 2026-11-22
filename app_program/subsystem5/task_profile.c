/*
 *	task_profile.c
 *	Obstacle Profile task (Week 6: Medium, queue-driven).  Collects the
 *	packed samples of one scan phase, builds the profile when the last
 *	sample arrives and sends it to the Avoidance & Recovery task.  A phase
 *	or scan-id change clears the buffer, so a late sample can
 *	never mix two obstacles (Tribunal 1: "scan identifiers, validity
 *	checks").
 */

#include "subsystem5.h"

EXPORT APP_SEQ	s5_prof_seq;
EXPORT UW	s5_prof_words[2];

LOCAL OP_SAMPLE	buf[OP_MAX_SAMPLES];

EXPORT void s5_get_profile(S5_PROFILE *p)
{
	UW	w[2] = {0, 0};
	UINT	tries;

	for (tries = 0; tries < APP_SEQ_READ_TRIES; tries++) {
		UW	q = app_seq_read_begin(&s5_prof_seq);

		w[0] = s5_prof_words[0];
		w[1] = s5_prof_words[1];
		if (!app_seq_read_retry(&s5_prof_seq, q)) {
			break;
		}
	}
	s5_profile_unpack(w, p);
}

EXPORT void profile_task(INT stacd, void *exinf)
{
	S5_SAMPLE	smp;
	S5_PROFILE	prof;
	UW		w, pw[2];
	S5_PROFILE_STATE current = {0, S5_PROFILE_ID_NONE, S5_PROFILE_PHASE_NONE, 0, FALSE};
	W		raw_width;

	(void)stacd;
	(void)exinf;
	sys_task_ready(SYS_TASK_PROFILE);
	for (;;) {
		if (tk_rcv_mbf(mbf_scan_samples, &w, TMO_FEVR) != (INT)sizeof(w)) {
			continue;
		}
		s5_sample_unpack(w, &smp);
		if (smp.scan_id != current.cur_id || smp.phase != current.cur_phase) {
			current.cur_id = smp.scan_id;		/* new obstacle or scan phase */
			current.cur_phase = smp.phase;
			current.n = 0;
			current.limit = 0;
			current.aborted = FALSE;
		}
		if (current.n < OP_MAX_SAMPLES) {
			buf[current.n].angle_deg = smp.angle_deg;
			buf[current.n].dist_mm = (H)((smp.status == SS_OK) ? s5_dist_mm(smp.dist_code) : -1);
			buf[current.n].status = smp.status;
			current.n++;
		}
		if (smp.status == SS_ABORTED) {
			current.aborted = TRUE;
		}
		if (smp.angle_deg > current.limit) {
			current.limit = smp.angle_deg;
		} else if (-smp.angle_deg > current.limit) {
			current.limit = -smp.angle_deg;
		}
		if (smp.last == 0U) {
			continue;
		}

		op_build(buf, current.n, current.limit, &prof);
		raw_width = prof.width_mm;
		if (current.aborted) {
			prof.status = PRF_INVALID;	/* sensor silent for the whole scan */
		} else {
			op_beam_compensate(&prof, S5_BEAM_COMP_DEG);
		}
		tm_printf((UB*)"profile: scan %d phase %d, samples %d, limit %d deg, "
			  "status %d, sensor %d mm, width %d mm (raw %d), "
			  "clear L %d R %d mm\n",
			  (INT)current.cur_id, (INT)current.cur_phase, (INT)current.n, current.limit,
			  (INT)prof.status, (INT)prof.dist_mm, (INT)prof.width_mm, (INT)raw_width,
			  (INT)prof.clear_left_mm, (INT)prof.clear_right_mm);
		prof.scan_id = current.cur_id;
		prof.phase = current.cur_phase;
		s5_profile_pack(&prof, pw);
		app_seq_write_begin(&s5_prof_seq);
		s5_prof_words[0] = pw[0];
		s5_prof_words[1] = pw[1];
		app_seq_write_end(&s5_prof_seq);
		if (tk_snd_mbf(mbf_profile, pw, (INT)sizeof(pw), TMO_POL) != E_OK) {
			sys_fault_set(SYS_FLT_TASK_COMM);
		}
	}
}
