/*
 * Automatic camera extrinsic calibration - see rift-cam-calib.h
 *
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#include <math.h>
#include <string.h>

#include "rift-cam-calib.h"

/* Re-solve every this many accepted samples. The solve is a robust mean over
 * at most RIFT_CAM_CALIB_HISTORY entries, so it costs microseconds; this is
 * only to keep it off the per-exposure path. */
#define SOLVE_INTERVAL 16
/* Trim passes when re-solving. */
#define TRIM_PASSES 3
#define TRIM_SIGMA 3.0f

void rift_cam_calib_init(rift_cam_calib *c)
{
	memset(c, 0, sizeof(*c));
	oquatf_set(&c->mean_orient, 0.0f, 0.0f, 0.0f, 1.0f);
	c->state = RIFT_CAM_UNCALIBRATED;
	c->residual_px = -1.0f;
}

void rift_cam_calib_reset(rift_cam_calib *c)
{
	uint32_t resets = c->n_resets, seen = c->n_seen, solves = c->n_solves;

	rift_cam_calib_init(c);
	c->n_resets = resets;
	c->n_seen = seen;
	c->n_solves = solves;
}

void rift_cam_calib_seed(rift_cam_calib *c, const posef *rel)
{
	rift_cam_calib_init(c);
	c->mean_orient = rel->orient;
	oquatf_normalize_me(&c->mean_orient);
	c->mean_pos = rel->pos;
	c->have_estimate = true;
	/* Never CALIBRATED: a pose from disk has to be confirmed against live
	 * observation before it is trusted. This is the case the Oculus runtime
	 * calls out with "Recalibrating camera %d: wasCalibrated %d, ...". */
	c->state = RIFT_CAM_ESTIMATED;
	c->settled = false;
}

/* The single-frame estimate, and the whole geometric basis of this module:
 *
 *   cam_other -> cam_ref  =  (obj -> cam_ref) . (obj -> cam_other)^-1
 *
 * No fusion state is involved, so unlike the old servo-toward-fusion attempt
 * there is no feedback loop to random-walk, and unlike the online refiner it
 * needs no existing camera pose to start from. */
static void single_frame_relative(const posef *obj_cam_ref,
	const posef *obj_cam_other, posef *rel)
{
	posef inv = *obj_cam_other;
	oposef_inverse(&inv);
	oposef_apply(&inv, obj_cam_ref, rel);
}

static float quat_angle_between(const quatf *a, const quatf *b)
{
	quatf d;
	float w;

	oquatf_diff(a, b, &d);
	oquatf_normalize_me(&d);
	w = fabsf(d.w);
	if (w > 1.0f)
		w = 1.0f;
	return 2.0f * acosf(w);
}

/* Which viewpoint bucket an observation belongs to. Coarse on purpose: the
 * point is only that a headset in a materially different place lands in a
 * different bucket, so eviction cannot squeeze it out. Distance and bearing
 * from the reference camera are what per-camera PnP bias actually tracks. */
static uint16_t viewpoint_bin(const posef *obj_cam_ref)
{
	const vec3f *p = &obj_cam_ref->pos;
	float dist = ovec3f_get_length((vec3f *) p);
	int d, az, el;

	if (!(dist > 0.05f))
		return 0;

	/* 4 distance shells x 2 azimuth x 2 elevation = 16 */
	d = (int) (dist / 0.75f);
	if (d > 3)
		d = 3;
	az = p->x >= 0.0f ? 1 : 0;
	el = p->y >= 0.0f ? 1 : 0;

	return (uint16_t) ((d * 4) + (az * 2) + el);
}

/* Cross-camera residual of one sample under a candidate relative pose, in
 * pixels. `rel` should map this camera's solution onto the reference camera's;
 * how far it misses by is the extrinsic error, seen from the reference camera
 * at the distance the device actually was.
 *
 * Orientation error is folded in at the LED shell radius so a rotational
 * mismatch is charged the same way a positional one is - the runtime reports
 * the two separately ("reprojection err %.2f, tilt err %.2f"); combining them
 * keeps one number to gate on. */
static float sample_residual_px(const rift_cam_calib_sample *s, const posef *rel)
{
	posef pred;
	vec3f d;
	float dist, err_m, ang;

	oposef_apply(&s->obj_cam_other, rel, &pred);

	ovec3f_subtract(&pred.pos, &s->obj_cam_ref.pos, &d);
	err_m = ovec3f_get_length(&d);

	ang = quat_angle_between(&pred.orient, &s->obj_cam_ref.orient);
	err_m += ang * RIFT_CAM_CALIB_SHELL_M;

	dist = ovec3f_get_length((vec3f *) &s->obj_cam_ref.pos);
	if (dist < 0.10f)
		dist = 0.10f;

	return RIFT_CAM_CALIB_FOCAL_PX * err_m / dist;
}

float rift_cam_calib_residual_px(const rift_cam_calib *c, const posef *rel)
{
	double sum = 0.0;
	uint16_t i;

	if (c->n_hist == 0)
		return -1.0f;

	for (i = 0; i < c->n_hist; i++)
		sum += sample_residual_px(c->hist + i, rel);

	return (float) (sum / c->n_hist);
}

bool rift_cam_calib_camera_moved(const rift_cam_calib *c, const posef *rel)
{
	float r;

	/* Only a history with something in it is entitled to overrule anything,
	 * and it must span more than the single viewpoint whose bias it would
	 * otherwise be mistaking for a moved camera. */
	if (c->n_hist < RIFT_CAM_CALIB_MIN_SAMPLES)
		return false;

	r = rift_cam_calib_residual_px(c, rel);
	return r > RIFT_CAM_CALIB_MOVED_PX;
}

/* Robust mean over the stored history: the "solve". Trimmed rather than
 * plain-averaged so a mislabelled LED set cannot drag it. Every entry counts
 * once regardless of how long the headset sat still there, because the history
 * is what is stratified - see the header. */
static bool solve_from_history(rift_cam_calib *c, posef *out)
{
	bool keep[RIFT_CAM_CALIB_HISTORY];
	posef rels[RIFT_CAM_CALIB_HISTORY];
	quatf qm;
	vec3f pm;
	uint16_t i, pass, n_keep;

	if (c->n_hist == 0)
		return false;

	for (i = 0; i < c->n_hist; i++) {
		single_frame_relative(&c->hist[i].obj_cam_ref,
			&c->hist[i].obj_cam_other, rels + i);
		keep[i] = isfinite(rels[i].pos.x) && isfinite(rels[i].pos.y) &&
			isfinite(rels[i].pos.z) && isfinite(rels[i].orient.w);
	}

	for (pass = 0; pass < TRIM_PASSES; pass++) {
		double sx = 0, sy = 0, sz = 0;
		float dev_pos = 0.0f, dev_ang = 0.0f;
		uint16_t n = 0;

		/* chordal quaternion mean, hemisphere-aligned to the first kept */
		quatf ref = { .x = 0, .y = 0, .z = 0, .w = 1 };
		double qx = 0, qy = 0, qz = 0, qw = 0;
		bool have_ref = false;

		for (i = 0; i < c->n_hist; i++) {
			quatf q;
			if (!keep[i])
				continue;
			q = rels[i].orient;
			if (!have_ref) {
				ref = q;
				have_ref = true;
			}
			if (q.x * ref.x + q.y * ref.y + q.z * ref.z + q.w * ref.w < 0.0f) {
				q.x = -q.x; q.y = -q.y; q.z = -q.z; q.w = -q.w;
			}
			qx += q.x; qy += q.y; qz += q.z; qw += q.w;
			sx += rels[i].pos.x; sy += rels[i].pos.y; sz += rels[i].pos.z;
			n++;
		}
		if (n == 0)
			return false;

		ovec3f_set(&pm, (float) (sx / n), (float) (sy / n), (float) (sz / n));
		oquatf_set(&qm, (float) qx, (float) qy, (float) qz, (float) qw);
		oquatf_normalize_me(&qm);

		if (pass + 1 == TRIM_PASSES)
			break;

		/* dispersion, then drop anything past 3 sigma and go again */
		for (i = 0; i < c->n_hist; i++) {
			vec3f d;
			if (!keep[i])
				continue;
			ovec3f_subtract(&rels[i].pos, &pm, &d);
			dev_pos += ovec3f_get_length(&d);
			dev_ang += quat_angle_between(&rels[i].orient, &qm);
		}
		dev_pos /= n;
		dev_ang /= n;
		if (!(dev_pos > 0.0f) || !(dev_ang > 0.0f))
			break;

		n_keep = 0;
		for (i = 0; i < c->n_hist; i++) {
			vec3f d;
			float dp, da;
			if (!keep[i])
				continue;
			ovec3f_subtract(&rels[i].pos, &pm, &d);
			dp = ovec3f_get_length(&d);
			da = quat_angle_between(&rels[i].orient, &qm);
			if (dp > TRIM_SIGMA * dev_pos || da > TRIM_SIGMA * dev_ang)
				keep[i] = false;
			else
				n_keep++;
		}
		if (n_keep < RIFT_CAM_CALIB_MIN_SAMPLES / 2)
			break;   /* trimmed too hard - keep what the last pass gave */
	}

	out->orient = qm;
	out->pos = pm;
	return true;
}

/* Store one sample, evicting from the FULLEST viewpoint bucket when there is
 * no room. That is what keeps a stationary headset from crowding out the
 * viewpoints that actually condition the estimate. */
static void history_store(rift_cam_calib *c, const posef *ref, const posef *other)
{
	uint16_t bin = viewpoint_bin(ref);
	uint16_t idx;

	if (c->n_hist < RIFT_CAM_CALIB_HISTORY) {
		idx = c->n_hist++;
	} else {
		uint16_t fullest = 0, i, victim = 0;
		uint32_t oldest = 0;
		bool have_victim = false;

		for (i = 0; i < RIFT_CAM_CALIB_BINS; i++) {
			if (c->bin_count[i] > c->bin_count[fullest])
				fullest = i;
		}
		/* The OLDEST entry of that bucket, not the lowest-indexed one.
		 * Taking the first match by index means a freshly written sample -
		 * which lands in whichever low slot was just freed - is the first
		 * thing found next time and is immediately evicted again, so the
		 * window never turns over: measured 191 of 192 entries still stale
		 * after 1100 newer samples, leaving the calibration unable to follow
		 * a sensor that moved while running. */
		for (i = 0; i < c->n_hist; i++) {
			if (c->hist[i].bin != fullest)
				continue;
			if (!have_victim || c->hist[i].seq < oldest) {
				oldest = c->hist[i].seq;
				victim = i;
				have_victim = true;
			}
		}
		if (!have_victim)
			return;
		c->bin_count[fullest]--;
		idx = victim;
	}

	c->hist[idx].obj_cam_ref = *ref;
	c->hist[idx].obj_cam_other = *other;
	c->hist[idx].bin = bin;
	c->hist[idx].seq = c->seq++;
	if (c->bin_count[bin] == 0)
		c->bins_seen++;
	c->bin_count[bin]++;
}

bool rift_cam_calib_add(rift_cam_calib *c, const posef *obj_cam_ref,
	const posef *obj_cam_other)
{
	posef rel;

	single_frame_relative(obj_cam_ref, obj_cam_other, &rel);
	if (!isfinite(rel.pos.x) || !isfinite(rel.pos.y) || !isfinite(rel.pos.z) ||
	    !isfinite(rel.orient.w))
		return false;

	c->n_seen++;
	history_store(c, obj_cam_ref, obj_cam_other);

	/* First sample: adopt it outright. One exposure is geometrically
	 * sufficient, and this is what gets tracking running immediately. */
	if (!c->have_estimate) {
		c->mean_orient = rel.orient;
		oquatf_normalize_me(&c->mean_orient);
		c->mean_pos = rel.pos;
		c->have_estimate = true;
		c->state = RIFT_CAM_ESTIMATED;
		c->residual_px = rift_cam_calib_residual_px(c, &rel);
		return true;
	}

	if (c->n_hist < RIFT_CAM_CALIB_MIN_SAMPLES ||
	    (c->n_seen % SOLVE_INTERVAL) != 0)
		return false;

	{
		posef cand, cur;
		float r_cur, r_new;

		if (!solve_from_history(c, &cand))
			return false;
		c->n_solves++;

		cur.orient = c->mean_orient;
		cur.pos = c->mean_pos;
		r_cur = rift_cam_calib_residual_px(c, &cur);
		r_new = rift_cam_calib_residual_px(c, &cand);
		c->residual_px = r_cur;

		/* Hold the best fit to the history we have. No improvement gate
		 * here: the "Insufficient Multicam calibration improvement" bar
		 * (0.85) guards against CHURN, and churn is a property of adopting a
		 * calibration into the running tracker, not of computing one. It
		 * belongs at the point where the sensor pose is actually moved and
		 * the fusion disturbed - rift_tracker_cam_calib_apply() applies it
		 * there. Gating here instead froze the estimate at whatever the
		 * first viewpoint gave: re-fitting after the headset was set down
		 * elsewhere cut the residual over the pooled history from 1.51 px to
		 * 1.34 px, a real gain but only 11%, so a 15% bar rejected it and
		 * left the very overfitting this module exists to remove.
		 *
		 * There is nothing to oscillate: the solve is a deterministic
		 * trimmed mean over a history that changes by at most one entry per
		 * exposure. */
		if (r_new < r_cur) {
			c->mean_orient = cand.orient;
			c->mean_pos = cand.pos;
			c->residual_px = r_new;
		}

		/* "Camera Calibration Settled after calibration." Requires both a
		 * residual inside the 2 px bar AND enough viewpoint spread for that
		 * residual to mean something - a single-viewpoint fit scores well
		 * against its own viewpoint no matter how biased it is. */
		if (c->residual_px >= 0.0f &&
		    c->residual_px <= RIFT_CAM_CALIB_GOOD_PX &&
		    c->bins_seen >= RIFT_CAM_CALIB_MIN_BINS) {
			c->state = RIFT_CAM_CALIBRATED;
			c->settled = true;
		} else {
			c->state = RIFT_CAM_ESTIMATED;
			c->settled = false;
		}
		return true;
	}
}

bool rift_cam_calib_get(const rift_cam_calib *c, posef *rel_out)
{
	if (!c->have_estimate || c->state == RIFT_CAM_UNCALIBRATED)
		return false;
	rel_out->orient = c->mean_orient;
	rel_out->pos = c->mean_pos;
	return true;
}

rift_cam_calib_action rift_cam_calib_decide(const rift_cam_calib *c,
	const posef *rel_in_use, bool adopted, bool recovering,
	int stored_viewpoints, float *out_r_in_use, float *out_r_est)
{
	posef rel;
	float r_in_use = -1.0f, r_est = -1.0f;

	if (c->n_hist >= RIFT_CAM_CALIB_MIN_SAMPLES && rift_cam_calib_get(c, &rel)) {
		r_est = rift_cam_calib_residual_px(c, &rel);
		if (rel_in_use != NULL)
			r_in_use = rift_cam_calib_residual_px(c, rel_in_use);
	}
	if (out_r_in_use != NULL)
		*out_r_in_use = r_in_use;
	if (out_r_est != NULL)
		*out_r_est = r_est;

	if (r_est < 0.0f)
		return RIFT_CAM_CALIB_WAIT;

	/* Nothing to weigh against: the sensor has no pose at all, so any estimate
	 * beats none. */
	if (rel_in_use == NULL)
		return RIFT_CAM_CALIB_ADOPT;

	/* Recovering from a move the caller has already acted on. The history was
	 * rebuilt from observations taken AFTER it, so the two tests below are not
	 * merely unnecessary here, they are actively wrong:
	 *
	 *  - camera-moved would compare the fresh history against the same stale
	 *    pose that triggered the reset, exceed the threshold again, and reset
	 *    again - forever, at roughly one reset per MIN_SAMPLES exposures. The
	 *    sensor would keep its wrong pose and never be corrected.
	 *  - the viewpoint guard would weigh the rebuilt history against a stored
	 *    viewpoint count describing geometry that no longer exists.
	 */
	if (recovering)
		return RIFT_CAM_CALIB_ADOPT;

	/* A pose that cannot describe the observed history AT ALL means the camera
	 * itself moved, not that the headset did - the two differ by ~600x in this
	 * measure. The history straddles the move, so it is worthless. */
	if (rift_cam_calib_camera_moved(c, rel_in_use))
		return RIFT_CAM_CALIB_RESET;

	/* A calibration fitted over MORE viewpoints must not be replaced by one
	 * fitted over fewer, however good the newcomer looks. It looks good
	 * precisely because it is scored against the narrow history it came from:
	 * measured over three real headset positions, a one-viewpoint fit scores
	 * 0.2-0.6 px at its own spot and 2.3-3.8 px at the others, while the
	 * three-viewpoint fit stays under 2 px everywhere. */
	if ((int) c->bins_seen < stored_viewpoints)
		return RIFT_CAM_CALIB_KEEP;

	/* Otherwise only replace it for a real improvement, on the runtime's own
	 * 15% bar. Setting the headset down somewhere new does not clear this,
	 * which is the point. */
	if (adopted && !(r_est < r_in_use * RIFT_CAM_CALIB_IMPROVE_GATE))
		return RIFT_CAM_CALIB_KEEP;

	return RIFT_CAM_CALIB_ADOPT;
}

void rift_cam_calib_to_world(const posef *ref_world, const posef *rel,
	posef *out_world)
{
	oposef_apply(rel, ref_world, out_world);
}
