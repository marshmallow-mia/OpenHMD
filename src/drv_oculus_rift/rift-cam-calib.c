/*
 * Automatic camera extrinsic calibration - see rift-cam-calib.h
 *
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#include <math.h>
#include <string.h>

#include "rift-cam-calib.h"

/* Deviations are tracked as running means rather than true variances: one
 * pass, no history, and the ratio to the mean deviation is a good enough
 * outlier test for data this clean (the offline reference keeps 1004/1024). */
#define DEV_BLEND 0.02f

/* The deviation EMAs start at zero, so for the first few dozen samples they
 * read low - which would make the sigma gate far tighter than 3 sigma and let
 * the settle test fire before the dispersion is even known. Divide out the
 * remaining bias, (1 - blend)^n, as an exponential average normally is. */
void rift_cam_calib_dev(const rift_cam_calib *c, float *out_ang, float *out_pos)
{
	float scale = 1.0f - c->dev_warm;

	if (scale < 1e-6f) {
		if (out_ang != NULL)
			*out_ang = 0.0f;
		if (out_pos != NULL)
			*out_pos = 0.0f;
		return;
	}
	if (out_ang != NULL)
		*out_ang = c->dev_ang / scale;
	if (out_pos != NULL)
		*out_pos = c->dev_pos / scale;
}

void rift_cam_calib_init(rift_cam_calib *c)
{
	memset(c, 0, sizeof(*c));
	oquatf_set(&c->mean_orient, 0.0f, 0.0f, 0.0f, 1.0f);
	c->dev_warm = 1.0f;
	c->state = RIFT_CAM_UNCALIBRATED;
}

void rift_cam_calib_reset(rift_cam_calib *c)
{
	rift_cam_calib_init(c);
}

void rift_cam_calib_seed(rift_cam_calib *c, const posef *rel)
{
	rift_cam_calib_init(c);
	c->mean_orient = rel->orient;
	oquatf_normalize_me(&c->mean_orient);
	c->mean_pos = rel->pos;
	c->n = 1;
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

bool rift_cam_calib_add(rift_cam_calib *c, const posef *obj_cam_ref,
	const posef *obj_cam_other)
{
	posef rel;
	float d_ang, d_pos, dev_ang, dev_pos;
	vec3f dp;

	single_frame_relative(obj_cam_ref, obj_cam_other, &rel);

	if (!isfinite(rel.pos.x) || !isfinite(rel.pos.y) || !isfinite(rel.pos.z) ||
	    !isfinite(rel.orient.w))
		return false;

	if (c->n == 0) {
		c->mean_orient = rel.orient;
		oquatf_normalize_me(&c->mean_orient);
		c->mean_pos = rel.pos;
		c->n = 1;
		c->state = RIFT_CAM_ESTIMATED;
		return true;
	}

	d_ang = quat_angle_between(&rel.orient, &c->mean_orient);
	ovec3f_subtract(&rel.pos, &c->mean_pos, &dp);
	d_pos = ovec3f_get_length(&dp);

	/* Reject once there is enough history for the deviation to mean
	 * something. A bumped sensor produces a sustained RUN of "outliers"
	 * rather than scattered ones, which is how the two are told apart
	 * below — otherwise the estimate would defend its stale mean forever. */
	rift_cam_calib_dev(c, &dev_ang, &dev_pos);
	if (c->n >= RIFT_CAM_CALIB_MIN_SAMPLES && dev_ang > 0.0f && dev_pos > 0.0f) {
		if (d_ang > RIFT_CAM_CALIB_OUTLIER_SIGMA * dev_ang ||
		    d_pos > RIFT_CAM_CALIB_OUTLIER_SIGMA * dev_pos) {
			c->n_rejected++;
			if (++c->n_consec_rejects < RIFT_CAM_CALIB_BUMP_RUN)
				return false;

			/* Every recent sample disagrees with the mean, so it is the
			 * mean that is stale: the sensor was moved. Start again from
			 * THIS sample rather than discarding it too, which would leave
			 * nothing to rebuild from. */
			{
				uint32_t rejected = c->n_rejected, resets = c->n_resets;
				rift_cam_calib_init(c);
				c->n_rejected = rejected;
				c->n_resets = resets + 1;
			}
			c->mean_orient = rel.orient;
			oquatf_normalize_me(&c->mean_orient);
			c->mean_pos = rel.pos;
			c->n = 1;
			c->state = RIFT_CAM_ESTIMATED;
			return true;
		}
	}
	c->n_consec_rejects = 0;

	c->n++;

	/* incremental means: position arithmetic, orientation by slerp toward
	 * the new sample with weight 1/n (the running chordal mean) */
	{
		const float w = 1.0f / (float) c->n;
		vec3f step;
		quatf blended;

		ovec3f_multiply_scalar(&dp, w, &step);
		ovec3f_add(&c->mean_pos, &step, &c->mean_pos);

		oquatf_slerp(w, &c->mean_orient, &rel.orient, true, &blended);
		oquatf_normalize_me(&blended);
		c->mean_orient = blended;
	}

	c->dev_ang += (d_ang - c->dev_ang) * DEV_BLEND;
	c->dev_pos += (d_pos - c->dev_pos) * DEV_BLEND;
	c->dev_warm *= (1.0f - DEV_BLEND);
	rift_cam_calib_dev(c, &dev_ang, &dev_pos);

	if (c->n >= RIFT_CAM_CALIB_MIN_SAMPLES &&
	    dev_ang <= RIFT_CAM_CALIB_SETTLE_ANG &&
	    dev_pos <= RIFT_CAM_CALIB_SETTLE_POS) {
		c->state = RIFT_CAM_CALIBRATED;
		c->settled = true;
	} else if (c->state == RIFT_CAM_UNCALIBRATED) {
		c->state = RIFT_CAM_ESTIMATED;
	}

	return true;
}

bool rift_cam_calib_get(const rift_cam_calib *c, posef *rel_out)
{
	if (c->n == 0 || c->state == RIFT_CAM_UNCALIBRATED)
		return false;
	rel_out->orient = c->mean_orient;
	rel_out->pos = c->mean_pos;
	return true;
}

void rift_cam_calib_compare(const rift_cam_calib *c, const posef *rel,
	float *out_ang, float *out_pos)
{
	vec3f d;

	if (out_ang != NULL)
		*out_ang = quat_angle_between(&rel->orient, &c->mean_orient);
	if (out_pos != NULL) {
		ovec3f_subtract(&rel->pos, &c->mean_pos, &d);
		*out_pos = ovec3f_get_length(&d);
	}
}

bool rift_cam_calib_rejects(const rift_cam_calib *c, const posef *rel)
{
	float ang, pos;

	/* Only a converged estimate is entitled to overrule anything. */
	if (c->n < RIFT_CAM_CALIB_MIN_SAMPLES)
		return false;

	rift_cam_calib_compare(c, rel, &ang, &pos);
	return ang > RIFT_CAM_CALIB_REJECT_ANG || pos > RIFT_CAM_CALIB_REJECT_POS;
}

void rift_cam_calib_to_world(const posef *ref_world, const posef *rel,
	posef *out_world)
{
	oposef_apply(rel, ref_world, out_world);
}
