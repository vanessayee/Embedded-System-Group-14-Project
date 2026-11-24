/*
 *	obstacle_profile.c
 *	Obstacle profile generation (see obstacle_profile.h).
 */

#include "obstacle_profile.h"

EXPORT W op_range_consensus_mm(const UW *ranges, UINT n)
{
	UW v[3], t, tolerance, best = 4001U;
	UINT i, j, pair = 0;
	BOOL found = FALSE;

	if (n < 2U || n > 3U) return -1;
	for (i = 0; i < n; i++) {
		if (ranges[i] < 20U || ranges[i] > 4000U) return -1;
		v[i] = ranges[i];
		for (j = i; j > 0U && v[j - 1U] > v[j]; j--) {
			t = v[j - 1U]; v[j - 1U] = v[j]; v[j] = t;
		}
	}
	for (i = 0; i + 1U < n; i++) {
		tolerance = (v[i] + v[i + 1U]) / 20U;
		if (tolerance < 10U) tolerance = 10U;
		if (v[i + 1U] - v[i] <= tolerance
		    && v[i + 1U] - v[i] < best) {
			best = v[i + 1U] - v[i]; pair = i; found = TRUE;
		}
	}
	if (!found) return -1;
	tolerance = (v[0] + v[n - 1U]) / 20U;
	if (tolerance < 10U) tolerance = 10U;
	if (n == 3U && v[2] - v[0] <= tolerance) return (W)v[1];
	return (W)((v[pair] + v[pair + 1U] + 1U) / 2U);
}

LOCAL W iabs(W v)
{
	return (v < 0) ? -v : v;
}

/* Bhaskara I approximation, integer degrees, |error| < 0.002 */
EXPORT W op_sin_q15(INT deg)
{
	W	x = deg % 360;
	W	sign = 1;
	D	p;

	if (x < 0) {
		x += 360;
	}
	if (x > 180) {
		x -= 180;
		sign = -1;
	}
	p = (D)x * (180 - x);
	return sign * (W)(((D)4 * p * 32768) / ((D)40500 - p));
}

EXPORT W op_cos_q15(INT deg)
{
	return op_sin_q15(90 - deg);
}

/* atan(y / x) in whole degrees, x > 0: (pi/4) z + 0.273 z (1 - z) */
EXPORT INT op_atan_deg(W y, W x)
{
	W	ay = iabs(y);
	W	z, a;

	if (x <= 0) {
		return (y < 0) ? -90 : 90;
	}
	if (ay <= x) {
		z = (W)(((D)ay << 15) / x);
		a = (W)(((D)45 * z + (D)1564 * z / 100 * (32768 - z) / 32768) / 32768);
	} else {
		z = (W)(((D)x << 15) / ay);
		a = 90 - (W)(((D)45 * z + (D)1564 * z / 100 * (32768 - z) / 32768) / 32768);
	}
	return (INT)((y < 0) ? -a : a);
}

LOCAL W lateral(W d, INT deg)
{
	return (W)(((D)d * op_sin_q15(deg)) / 32768);
}

LOCAL W forward(W d, INT deg)
{
	return (W)(((D)d * op_cos_q15(deg) + 16384) / 32768);
}

/* Sort a copy by angle (insertion sort, n <= OP_MAX_SAMPLES) */
LOCAL UINT sorted_copy(const OP_SAMPLE *s, UINT n, OP_SAMPLE *o)
{
	UINT	i, j;

	if (n > OP_MAX_SAMPLES) {
		n = OP_MAX_SAMPLES;
	}
	for (i = 0; i < n; i++) {
		OP_SAMPLE	t = s[i];

		for (j = i; j > 0U && o[j - 1U].angle_deg > t.angle_deg; j--) {
			o[j] = o[j - 1U];
		}
		o[j] = t;
	}
	return n;
}

LOCAL void init_out(S5_PROFILE *out, UB status)
{
	out->status = status;
	out->dist_mm = -1;
	out->bearing_deg = 0;
	out->width_mm = -1;
	out->left_mm = 0;
	out->right_mm = 0;
	out->clear_left_mm = -1;
	out->clear_right_mm = -1;
}

/* Frozen servo: identical returns across a wide sweep, with no open rays. */
LOCAL BOOL looks_frozen(const OP_SAMPLE *o, UINT n)
{
	W	dmin = 0x7FFFFFFF, dmax = -1;
	INT	amin = 1000, amax = -1000;
	UINT	i, valid = 0;

	for (i = 0; i < n; i++) {
		/* A hit-to-no-echo transition is evidence of a changing scene.
		   Narrow targets can otherwise have <10 mm range spread and be
		   misclassified as a stuck servo despite clear edges. */
		if (o[i].status == SS_NO_ECHO) return FALSE;
		if (o[i].status != SS_OK) {
			continue;
		}
		valid++;
		if (o[i].dist_mm < dmin) dmin = o[i].dist_mm;
		if (o[i].dist_mm > dmax) dmax = o[i].dist_mm;
		if (o[i].angle_deg < amin) amin = o[i].angle_deg;
		if (o[i].angle_deg > amax) amax = o[i].angle_deg;
	}
	return (valid >= OP_MIN_VALID && amax - amin >= OP_FROZEN_SPAN_DEG
		&& dmax - dmin <= OP_FROZEN_SPREAD_MM) ? TRUE : FALSE;
}

/* Lateral position, on the obstacle's front plane y, of bearing deg */
LOCAL W planar(W y, INT deg)
{
	W	c = op_cos_q15(deg);

	return (c > 0) ? (W)(((D)y * op_sin_q15(deg)) / c) : 0;
}

EXPORT void op_beam_compensate(S5_PROFILE *p, INT half_angle_deg)
{
	INT left_a, right_a, raw_span, correction;
	W left, right;

	/* Unknown edges cannot be corrected into known free space. */
	if (p->status != PRF_VALID || p->dist_mm <= 0 || half_angle_deg <= 0
	    || p->clear_left_mm < 0 || p->clear_right_mm < 0) return;
	left_a = op_atan_deg(p->left_mm, p->dist_mm);
	right_a = op_atan_deg(p->right_mm, p->dist_mm);
	raw_span = right_a - left_a;
	if (raw_span <= OP_EDGE_RESOLUTION_DEG) return;

	/* A single-object calibration must not erase most of another object's
	   measured span. Limit any future allowance to one scan cell per edge,
	   retaining at least one cell of the observed obstacle. */
	correction = half_angle_deg;
	if (correction > OP_EDGE_RESOLUTION_DEG) correction = OP_EDGE_RESOLUTION_DEG;
	if (correction > (raw_span - OP_EDGE_RESOLUTION_DEG) / 2)
		correction = (raw_span - OP_EDGE_RESOLUTION_DEG) / 2;
	if (correction <= 0) return;
	left = planar(p->dist_mm, left_a + correction);
	right = planar(p->dist_mm, right_a - correction);
	if (left < p->left_mm || right > p->right_mm || right <= left) return;
	p->left_mm = left;
	p->right_mm = right;
	p->width_mm = right - left;
	p->bearing_deg = op_atan_deg((left + right) / 2, p->dist_mm);
	/* Correcting a model is not an additional observation of free space.
	   Keep both measured side clearances unchanged. */
}

LOCAL W depth_tolerance(W range)
{
	W tol = range / OP_DEPTH_RANGE_DIV;
	if (tol < OP_DEPTH_MIN_MM) tol = OP_DEPTH_MIN_MM;
	return (tol > OP_DEPTH_TOL_MM) ? OP_DEPTH_TOL_MM : tol;
}

LOCAL BOOL is_hit(const OP_SAMPLE *o, const H *y, INT n, INT i, W y_near)
{
	return (i >= 0 && i < n && o[i].status == SS_OK
		&& iabs((W)y[i] - y_near) <= depth_tolerance(y_near)) ? TRUE : FALSE;
}

/* A face-band echo that is really the obstacle corner heard off-axis:
   outside the direct beam, clearly farther than the face, and projecting
   nearer than the face plane (OP_CORNER_* in obstacle_profile.h). */
LOCAL BOOL corner_echo(const OP_SAMPLE *o, H y, W y_near)
{
	return (iabs(o->angle_deg) > OP_BEAM_DIRECT_DEG
		&& (W)o->dist_mm - y_near >= OP_CORNER_RANGE_MM
		&& (W)y < y_near - OP_CORNER_DEPTH_MM) ? TRUE : FALSE;
}

/*
 * Ultrasonic reflections can disappear at one servo angle and reappear at
 * the next (the 3-degree fine scan observed 325, 781, 301 mm).  Treat one
 * isolated miss between front-face hits as part of the same obstacle.  Two
 * consecutive misses still establish a real edge.
 */
LOCAL BOOL region_continues(const OP_SAMPLE *o, const H *y, INT n, INT i,
			    INT step, W y_near)
{
	INT	j;

	if (is_hit(o, y, n, i, y_near) || (i >= 0 && i < n && o[i].status == SS_INVALID)) {
		return TRUE;
	}
	/* Only a missing/far background reflection can be bridged. A valid
	   nearer return does not support continuity of this front plane. Bridging it merged separate echo groups in the 208 mm log. */
	if (i >= 0 && i < n && o[i].status == SS_OK
	    && (W)y[i] < y_near - depth_tolerance(y_near)) return FALSE;
	for (j = 1; j <= OP_MAX_HIT_GAP; j++) {
		if (is_hit(o, y, n, i + step * j, y_near)) {
			return TRUE;
		}
	}
	return FALSE;
}

/* Clearance on one side: from the obstacle edge to the nearest other
   object on that side, or to the lateral reach of the scan at the
   obstacle's plane if nothing else was seen. */
LOCAL W side_clearance(const OP_SAMPLE *o, const H *y, UINT n, INT from, INT step,
		       W edge_x, W y_near, INT scan_limit_deg, BOOL edge_known)
{
	W	best = -1;
	W	reach;
	INT	i;

	if (!edge_known) {
		return -1;			/* unknown, never "free" */
	}
	for (i = from; i >= 0 && i < (INT)n; i += step) {
		W	x;

		if (o[i].status != SS_OK
		    || iabs((W)y[i] - y_near) > depth_tolerance(y_near)) {
			continue;
		}
		x = lateral(o[i].dist_mm, o[i].angle_deg);
		if (best < 0 || iabs(x - edge_x) < best) {
			best = iabs(x - edge_x);
		}
	}
	reach = iabs(planar(y_near, (step > 0) ? scan_limit_deg : -scan_limit_deg) - edge_x);
	if (best < 0 || best > reach) {
		best = reach;			/* free as far as the scan can see */
	}
	return (best > OP_CLEAR_CAP_MM) ? OP_CLEAR_CAP_MM : best;
}

EXPORT void op_build(const OP_SAMPLE *s, UINT n, INT scan_limit_deg,
		     S5_PROFILE *out)
{
	OP_SAMPLE	o[OP_MAX_SAMPLES];
	H		y[OP_MAX_SAMPLES];	/* forward distance of each echo, mm */
	H		front_y[OP_MAX_SAMPLES];
	UINT		i, j, readings = 0, hits = 0, front_n = 0;
	INT		near_i = -1, li, ri, k;
	W		y_near = 0x7FFFFFFF;
	INT		edge_l, edge_r;
	BOOL		l_known, r_known;

	n = sorted_copy(s, n, o);
	init_out(out, PRF_INVALID);

	for (i = 0; i < n; i++) {
		y[i] = -1;
		if (o[i].status == SS_NO_ECHO) {
			readings++;			/* valid: nothing within 4 m */
		}
		if (o[i].status != SS_OK) {
			continue;
		}
		readings++;
		y[i] = (H)forward(o[i].dist_mm, o[i].angle_deg);	/* <= 4000 */
		if (iabs(o[i].angle_deg) <= OP_FRONT_ESTIMATE_DEG) {
			front_y[front_n++] = y[i];
		}
		if (iabs(o[i].angle_deg) <= OP_FORWARD_CONE_DEG && y[i] < y_near) {
			y_near = y[i];
			near_i = (INT)i;
		}
	}
	if (readings < OP_MIN_VALID) {
		return;					/* PRF_INVALID: sensor data missing */
	}
	if (looks_frozen(o, n)) {
		return;					/* PRF_INVALID: servo suspect */
	}
	/* The old minimum-of-+-30-deg estimate let one short/specular return
	   make the obstacle appear tens of millimetres closer after a reverse.
	   The robot was triggered by its centre ray, so use the median of the
	   filtered samples around that ray and retain the minimum as fallback. */
	if (front_n > 0U) {
		W	best_delta = 0x7FFFFFFF;

		for (i = 1; i < front_n; i++) {
			H v = front_y[i];

			j = i;
			while (j > 0U && front_y[j - 1U] > v) {
				front_y[j] = front_y[j - 1U];
				j--;
			}
			front_y[j] = v;
		}
		y_near = (front_n & 1U) ? front_y[front_n / 2U]
			 : ((W)front_y[front_n / 2U - 1U] + front_y[front_n / 2U]) / 2;
		near_i = -1;
		for (i = 0; i < n; i++) {
			W delta;

			if (y[i] < 0 || iabs(o[i].angle_deg) > OP_FRONT_ESTIMATE_DEG) {
				continue;
			}
			delta = iabs((W)y[i] - y_near);
			if (delta < best_delta) {
				best_delta = delta;
				near_i = (INT)i;
			}
		}
	}
	if (near_i < 0 || y_near > OP_DETECT_RANGE_MM) {
		init_out(out, PRF_NO_OBSTACLE);		/* path is clear */
		return;
	}

	/* An echo belongs to the obstacle when it lies on (or just behind)
	   the nearest front plane: forward distance, not range, so a flat
	   face or a wall is recognised as one surface. */
	li = near_i;
	while (li - 1 >= 0 && region_continues(o, y, (INT)n, li - 1, -1, y_near)) {
		li--;
	}
	ri = near_i;
	while (ri + 1 < (INT)n && region_continues(o, y, (INT)n, ri + 1, 1, y_near)) {
		ri++;
	}
	while (li < near_i && !is_hit(o, y, (INT)n, li, y_near)) {
		li++;				/* do not end the region on a gap */
	}
	while (ri > near_i && !is_hit(o, y, (INT)n, ri, y_near)) {
		ri--;
	}
	/* Drop at most ONE corner echo per side: bounds any underestimate of a
	   genuinely wider (e.g. angled) face to a single scan cell. */
	if (li < near_i && corner_echo(&o[li], y[li], y_near)) {
		li++;
		while (li < near_i && !is_hit(o, y, (INT)n, li, y_near)) li++;
	}
	if (ri > near_i && corner_echo(&o[ri], y[ri], y_near)) {
		ri--;
		while (ri > near_i && !is_hit(o, y, (INT)n, ri, y_near)) ri--;
	}
	for (k = li; k <= ri; k++) {
		if (is_hit(o, y, (INT)n, k, y_near)) {
			hits++;
		}
	}
	if (hits < OP_MIN_HITS) {
		return;					/* PRF_INVALID: one stray echo */
	}

	/* edge = midway between the last hit and the first miss */
	l_known = (li > 0) ? TRUE : FALSE;
	r_known = (ri < (INT)n - 1) ? TRUE : FALSE;
	edge_l = l_known ? (o[li].angle_deg + o[li - 1].angle_deg) / 2 : o[li].angle_deg;
	edge_r = r_known ? (o[ri].angle_deg + o[ri + 1].angle_deg) / 2 : o[ri].angle_deg;
	/* If a nearer mismatched ray separated another hit group, allow up
	   to that first ray for edge uncertainty, without absorbing the group.
	   This is bounded by the actual scan spacing, not a fixed object size. */
	if (l_known && o[li - 1].status == SS_OK
	    && (W)y[li - 1] < y_near - depth_tolerance(y_near)
	    && is_hit(o, y, (INT)n, li - 2, y_near)) edge_l = o[li - 1].angle_deg;
	if (r_known && o[ri + 1].status == SS_OK
	    && (W)y[ri + 1] < y_near - depth_tolerance(y_near)
	    && is_hit(o, y, (INT)n, ri + 2, y_near)) edge_r = o[ri + 1].angle_deg;

	out->dist_mm = y_near;
	out->left_mm = planar(y_near, edge_l);
	out->right_mm = planar(y_near, edge_r);
	out->width_mm = out->right_mm - out->left_mm;
	if (out->width_mm < 0) {
		out->width_mm = 0;
	}
	out->bearing_deg = (edge_l + edge_r) / 2;
	out->clear_left_mm = side_clearance(o, y, n, li - 1, -1, out->left_mm,
					    y_near, scan_limit_deg, l_known);
	out->clear_right_mm = side_clearance(o, y, n, ri + 1, 1, out->right_mm,
					     y_near, scan_limit_deg, r_known);
	out->status = (l_known || r_known) ? PRF_VALID : PRF_INSUFFICIENT;
}
