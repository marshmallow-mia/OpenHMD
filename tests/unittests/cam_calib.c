/*
 * Tests for automatic camera extrinsic calibration.
 *
 * The premise under test is that NO user interaction is needed: one exposure
 * seen by two cameras determines the transform between them exactly, and
 * averaging many only reduces noise.
 *
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

#include "tests.h"
#include "drv_oculus_rift/rift-cam-calib.h"

/* oposef_apply(A, B, out) composes out = B . A — verified against the
 * driver's own use in rift-sensor-pose-search.c (obj->cam then cam->world
 * gives obj->world). */

static void pose_make(posef *p, float px, float py, float pz,
	float ax, float ay, float az, float angle)
{
	vec3f axis = {{ ax, ay, az }};
	ovec3f_normalize_me(&axis);
	oquatf_init_axis(&p->orient, &axis, angle);
	ovec3f_set(&p->pos, px, py, pz);
}

/* the device's pose as a camera at `cam_world` would solve it */
static void obj_in_cam(const posef *obj_world, const posef *cam_world, posef *out)
{
	posef inv = *cam_world;
	oposef_inverse(&inv);
	oposef_apply(obj_world, &inv, out);
}

/* truth: cam_other -> cam_ref */
static void relative_truth(const posef *cam_ref, const posef *cam_other, posef *out)
{
	posef inv = *cam_ref;
	oposef_inverse(&inv);
	oposef_apply(cam_other, &inv, out);
}

static float pose_ang_err(const posef *a, const posef *b)
{
	quatf d;
	float w;
	oquatf_diff(&a->orient, &b->orient, &d);
	oquatf_normalize_me(&d);
	w = fabsf(d.w);
	if (w > 1.0f)
		w = 1.0f;
	return 2.0f * acosf(w);
}

static float pose_pos_err(const posef *a, const posef *b)
{
	vec3f d;
	ovec3f_subtract(&a->pos, &b->pos, &d);
	return ovec3f_get_length(&d);
}

/* ONE co-observed exposure must determine the full 6-DoF transform between
 * the cameras. This is the claim that makes a calibration walk unnecessary. */
void test_rift_cam_calib_single_exposure_is_enough()
{
	posef cam_ref, cam_other, obj, ocr, oco, truth, got;
	rift_cam_calib c;

	pose_make(&cam_ref, 0.49f, 1.56f, -1.62f, 0.0f, 1.0f, 0.0f, 0.10f);
	pose_make(&cam_other, -0.74f, 1.84f, -1.54f, 0.0f, 1.0f, 0.0f, -0.40f);
	pose_make(&obj, 0.05f, 1.10f, -0.30f, 0.2f, 1.0f, 0.1f, 0.6f);

	obj_in_cam(&obj, &cam_ref, &ocr);
	obj_in_cam(&obj, &cam_other, &oco);
	relative_truth(&cam_ref, &cam_other, &truth);

	rift_cam_calib_init(&c);
	TAssert(!rift_cam_calib_get(&c, &got));        /* nothing known yet */

	TAssert(rift_cam_calib_add(&c, &ocr, &oco));   /* a single exposure */
	TAssert(rift_cam_calib_get(&c, &got));

	TAssert(pose_ang_err(&got, &truth) < 1e-4f);
	TAssert(pose_pos_err(&got, &truth) < 1e-4f);

	/* and it composes back to the camera's world pose */
	posef world;
	rift_cam_calib_to_world(&cam_ref, &got, &world);
	TAssert(pose_ang_err(&world, &cam_other) < 1e-4f);
	TAssert(pose_pos_err(&world, &cam_other) < 1e-4f);
}

/* Deterministic pseudo-noise on a solved pose, standing in for per-camera
 * PnP error. */
static void perturb(posef *p, int i, float pos_amp, float ang_amp)
{
	vec3f axis = {{ (float)((i * 37) % 13) - 6.0f,
	                (float)((i * 53) % 11) - 5.0f,
	                (float)((i * 71) % 17) - 8.0f }};
	quatf dq, tmp;

	ovec3f_normalize_me(&axis);
	oquatf_init_axis(&dq, &axis, ang_amp * (((i * 29) % 7) / 3.0f - 1.0f));
	oquatf_mult(&dq, &p->orient, &tmp);
	oquatf_normalize_me(&tmp);
	p->orient = tmp;

	p->pos.x += pos_amp * (((i * 41) % 9) / 4.0f - 1.0f);
	p->pos.y += pos_amp * (((i * 17) % 7) / 3.0f - 1.0f);
	p->pos.z += pos_amp * (((i * 61) % 11) / 5.0f - 1.0f);
}

/* Averaging many noisy exposures must beat a single noisy one, and the
 * estimate must settle. The device does NOT move between exposures. */
void test_rift_cam_calib_averages_and_settles()
{
	posef cam_ref, cam_other, obj, truth, got, first;
	rift_cam_calib c;
	int i;

	pose_make(&cam_ref, 0.49f, 1.56f, -1.62f, 0.0f, 1.0f, 0.0f, 0.10f);
	pose_make(&cam_other, -0.74f, 1.84f, -1.54f, 0.0f, 1.0f, 0.0f, -0.40f);
	pose_make(&obj, 0.0f, 1.05f, -0.35f, 0.1f, 1.0f, 0.0f, 0.3f);
	relative_truth(&cam_ref, &cam_other, &truth);

	rift_cam_calib_init(&c);

	for (i = 0; i < 400; i++) {
		posef ocr, oco;
		obj_in_cam(&obj, &cam_ref, &ocr);
		obj_in_cam(&obj, &cam_other, &oco);
		perturb(&ocr, i, 0.002f, 0.002f);
		perturb(&oco, i + 7, 0.002f, 0.002f);
		rift_cam_calib_add(&c, &ocr, &oco);
		if (i == 0)
			rift_cam_calib_get(&c, &first);
	}

	TAssert(rift_cam_calib_get(&c, &got));
	TAssert(c.state == RIFT_CAM_CALIBRATED);
	TAssert(c.settled);

	/* the reported dispersion is bias-corrected, so it reflects the real
	 * per-sample scatter rather than an EMA still climbing out of zero */
	{
		float dev_ang, dev_pos;
		rift_cam_calib_dev(&c, &dev_ang, &dev_pos);
		TAssert(dev_pos > 0.0005f);   /* the noise is not invisible... */
		TAssert(dev_pos < 0.010f);    /* ...and it is inside the settle bar */
		TAssert(dev_ang > 0.0f);
		TAssert(dev_ang < 0.0087f);
	}

	/* the averaged estimate beats the single-shot one it started from */
	TAssert(pose_pos_err(&got, &truth) < pose_pos_err(&first, &truth));
	TAssert(pose_pos_err(&got, &truth) < 0.005f);
	TAssert(pose_ang_err(&got, &truth) < 0.01f);
}

/* The failure that motivated all of this: a stored calibration that is 9 deg
 * and 214 mm wrong because a sensor was moved. It must be REJECTED, not
 * refined — the driver used to trust it absolutely. */
void test_rift_cam_calib_rejects_stale_stored_calibration()
{
	posef cam_ref, cam_other, obj, truth, stale, got;
	rift_cam_calib c;
	int i;

	pose_make(&cam_ref, 0.49f, 1.56f, -1.62f, 0.0f, 1.0f, 0.0f, 0.10f);
	pose_make(&cam_other, -0.74f, 1.84f, -1.54f, 0.0f, 1.0f, 0.0f, -0.40f);
	pose_make(&obj, 0.0f, 1.05f, -0.35f, 0.1f, 1.0f, 0.0f, 0.3f);
	relative_truth(&cam_ref, &cam_other, &truth);

	/* what the file claims: 9 deg out and 214 mm out */
	stale = truth;
	{
		vec3f axis = {{ 0.3f, 1.0f, 0.2f }};
		quatf dq, tmp;
		ovec3f_normalize_me(&axis);
		oquatf_init_axis(&dq, &axis, 9.0f / 57.2957795f);
		oquatf_mult(&dq, &stale.orient, &tmp);
		oquatf_normalize_me(&tmp);
		stale.orient = tmp;
		stale.pos.x += 0.214f;
	}

	rift_cam_calib_init(&c);
	rift_cam_calib_seed(&c, &stale);
	/* seeded from disk is ESTIMATED, never CALIBRATED */
	TAssert(c.state == RIFT_CAM_ESTIMATED);
	TAssert(!c.settled);
	/* and with no live evidence yet it cannot overrule anything */
	TAssert(!rift_cam_calib_rejects(&c, &stale));

	/* now observe reality */
	rift_cam_calib_reset(&c);
	for (i = 0; i < 200; i++) {
		posef ocr, oco;
		obj_in_cam(&obj, &cam_ref, &ocr);
		obj_in_cam(&obj, &cam_other, &oco);
		perturb(&ocr, i, 0.002f, 0.002f);
		perturb(&oco, i + 3, 0.002f, 0.002f);
		rift_cam_calib_add(&c, &ocr, &oco);
	}

	TAssert(c.state == RIFT_CAM_CALIBRATED);
	TAssert(rift_cam_calib_rejects(&c, &stale));   /* the file is wrong */
	TAssert(!rift_cam_calib_rejects(&c, &truth));  /* reality is not */

	rift_cam_calib_get(&c, &got);
	TAssert(pose_ang_err(&got, &truth) < 0.01f);
}

/* A wild outlier must not drag the estimate. */
void test_rift_cam_calib_rejects_outliers()
{
    posef cam_ref, cam_other, obj, truth, before, after;
	rift_cam_calib c;
	int i;

	pose_make(&cam_ref, 0.49f, 1.56f, -1.62f, 0.0f, 1.0f, 0.0f, 0.10f);
	pose_make(&cam_other, -0.74f, 1.84f, -1.54f, 0.0f, 1.0f, 0.0f, -0.40f);
	pose_make(&obj, 0.0f, 1.05f, -0.35f, 0.0f, 1.0f, 0.0f, 0.2f);
	relative_truth(&cam_ref, &cam_other, &truth);

	rift_cam_calib_init(&c);
	for (i = 0; i < 200; i++) {
		posef ocr, oco;
		obj_in_cam(&obj, &cam_ref, &ocr);
		obj_in_cam(&obj, &cam_other, &oco);
		perturb(&ocr, i, 0.001f, 0.001f);
		perturb(&oco, i + 5, 0.001f, 0.001f);
		rift_cam_calib_add(&c, &ocr, &oco);
	}
	rift_cam_calib_get(&c, &before);

	{   /* a solve that landed half a metre away - a mislabelled LED set */
		posef ocr, oco;
		/* the noisy stream will already have tripped the gate a few times,
		 * which is the gate working; what matters is that THIS one does */
		uint32_t rejected_before = c.n_rejected;
		uint32_t n_before = c.n;

		obj_in_cam(&obj, &cam_ref, &ocr);
		obj_in_cam(&obj, &cam_other, &oco);
		oco.pos.x += 0.5f;
		TAssert(!rift_cam_calib_add(&c, &ocr, &oco));
		TAssert(c.n_rejected == rejected_before + 1);
		TAssert(c.n == n_before);              /* not folded into the mean */
	}

	rift_cam_calib_get(&c, &after);
	TAssert(pose_pos_err(&before, &after) < 1e-6f);   /* untouched */
	TAssert(pose_pos_err(&after, &truth) < 0.005f);
}
