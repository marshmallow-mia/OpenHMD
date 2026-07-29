/*
 * Tests for automatic camera extrinsic calibration.
 *
 * Two claims are under test. First, that NO user interaction is needed: one
 * exposure seen by two cameras determines the transform between them exactly.
 * Second — the harder one, and the reason this module was rewritten — that
 * moving the HEADSET is never mistaken for moving a CAMERA.
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

/* Deterministic pseudo-noise on a solved pose, standing in for blob noise. */
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

/* Four places a headset might sit, far enough apart to land in different
 * viewpoint buckets (rift-cam-calib.c viewpoint_bin: distance shells of
 * 0.75 m, split by the sign of x and y in the reference camera's frame). */
static const float VIEWPOINTS[4][3] = {
	{  0.10f,  0.30f, 1.20f },
	{ -0.20f,  0.40f, 2.00f },
	{  0.30f, -0.20f, 2.80f },
	{  0.50f, -0.30f, 1.50f },
};

/* One co-observed exposure with the headset at viewpoint `v`.
 *
 * `bias_m` models per-camera PnP bias: the second camera's solution is offset
 * by an amount that DEPENDS ON THE VIEWPOINT, which is exactly what makes a
 * single-viewpoint calibration overfit. Measured on real hardware at ~7 mm for
 * 28 cm of headset movement (windows-vs-linux-tracking.md §5b). */
static void exposure_at(int v, int i, float bias_m, float noise_m,
	posef *ocr, posef *oco)
{
	posef cam_ref, cam_other, obj, at;

	pose_make(&cam_ref, 0.49f, 1.56f, -1.62f, 0.0f, 1.0f, 0.0f, 0.10f);
	pose_make(&cam_other, -0.74f, 1.84f, -1.54f, 0.0f, 1.0f, 0.0f, -0.40f);

	/* place the object so that, seen from cam_ref, it sits at VIEWPOINTS[v] */
	pose_make(&at, VIEWPOINTS[v][0], VIEWPOINTS[v][1], VIEWPOINTS[v][2],
		0.1f, 1.0f, 0.0f, 0.3f + 0.2f * (float) v);
	oposef_apply(&at, &cam_ref, &obj);

	obj_in_cam(&obj, &cam_ref, ocr);
	obj_in_cam(&obj, &cam_other, oco);

	/* viewpoint-dependent bias, in the second camera only */
	oco->pos.x += bias_m * (float)(v - 1);
	oco->pos.z += bias_m * 0.5f * (float)((v & 1) ? 1 : -1);

	perturb(ocr, i, noise_m, noise_m);
	perturb(oco, i + 7, noise_m, noise_m);
}

/* Averaging noise down must work, and settling must require the estimate to be
 * conditioned by more than one viewpoint — a fit scored against the single
 * viewpoint it came from flatters itself no matter how biased it is. */
void test_rift_cam_calib_averages_and_settles()
{
	rift_cam_calib c;
	posef ocr, oco, got;
	int i, v;

	/* one viewpoint only: it can converge, but it must NOT claim to be
	 * settled, because nothing has tested it against another viewpoint */
	rift_cam_calib_init(&c);
	for (i = 0; i < 300; i++) {
		exposure_at(0, i, 0.0f, 0.002f, &ocr, &oco);
		rift_cam_calib_add(&c, &ocr, &oco);
	}
	TAssert(rift_cam_calib_get(&c, &got));
	TAssert(c.bins_seen == 1);
	TAssert(!c.settled);
	TAssert(c.state == RIFT_CAM_ESTIMATED);

	/* now let the headset be used normally - several places */
	for (v = 1; v < 4; v++) {
		for (i = 0; i < 300; i++) {
			exposure_at(v, i, 0.0f, 0.002f, &ocr, &oco);
			rift_cam_calib_add(&c, &ocr, &oco);
		}
	}
	TAssert(c.bins_seen >= RIFT_CAM_CALIB_MIN_BINS);
	TAssert(c.state == RIFT_CAM_CALIBRATED);
	TAssert(c.settled);

	/* with no bias, the answer is the truth and the residual is tiny */
	{
		posef cam_ref, cam_other, truth;
		pose_make(&cam_ref, 0.49f, 1.56f, -1.62f, 0.0f, 1.0f, 0.0f, 0.10f);
		pose_make(&cam_other, -0.74f, 1.84f, -1.54f, 0.0f, 1.0f, 0.0f, -0.40f);
		relative_truth(&cam_ref, &cam_other, &truth);
		rift_cam_calib_get(&c, &got);
		TAssert(pose_pos_err(&got, &truth) < 0.005f);
		TAssert(c.residual_px >= 0.0f);
		TAssert(c.residual_px <= RIFT_CAM_CALIB_GOOD_PX);
	}
}

/* THE REGRESSION THIS MODULE WAS REWRITTEN FOR.
 *
 * Setting the headset down somewhere new shifts every single-frame estimate,
 * because per-camera PnP bias is viewpoint-dependent. The previous design
 * tested that shift against the scatter of a converged running mean, which
 * meant an ordinary headset move (~13 mm) sat a factor of 1.14 from the
 * trigger (~15 mm) and tripped a false "the camera was moved" rebuild. It must
 * not, and a real camera move must still be caught. */
void test_rift_cam_calib_headset_move_is_not_a_camera_move()
{
	rift_cam_calib c;
	posef ocr, oco, adopted;
	int i;

	/* converge with the headset sitting in one place */
	rift_cam_calib_init(&c);
	for (i = 0; i < 400; i++) {
		exposure_at(0, i, 0.007f, 0.002f, &ocr, &oco);
		rift_cam_calib_add(&c, &ocr, &oco);
	}
	TAssert(rift_cam_calib_get(&c, &adopted));
	TAssert(c.n_resets == 0);

	/* the headset is picked up and set down somewhere else, for a long time */
	for (i = 0; i < 800; i++) {
		exposure_at(1, i, 0.007f, 0.002f, &ocr, &oco);
		rift_cam_calib_add(&c, &ocr, &oco);
		/* at no point may the calibration in use look like a moved camera */
		TAssert(!rift_cam_calib_camera_moved(&c, &adopted));
	}
	TAssert(c.n_resets == 0);

	/* but a camera actually knocked 226 mm - what happened on real hardware -
	 * has to be caught */
	{
		posef moved = adopted;
		moved.pos.x += 0.226f;
		TAssert(rift_cam_calib_camera_moved(&c, &moved));
	}
}

/* A history spanning several viewpoints must generalise better than one built
 * from a single viewpoint, judged at a viewpoint neither was fitted on. This
 * is the reason the history is stratified rather than a plain ring. */
void test_rift_cam_calib_history_spans_viewpoints()
{
	rift_cam_calib narrow, wide, heldout;
	posef ocr, oco, rel_narrow, rel_wide;
	float r_narrow, r_wide;
	const float BIAS = 0.007f;
	int i, v;

	/* fitted at one place only */
	rift_cam_calib_init(&narrow);
	for (i = 0; i < 600; i++) {
		exposure_at(0, i, BIAS, 0.002f, &ocr, &oco);
		rift_cam_calib_add(&narrow, &ocr, &oco);
	}
	TAssert(rift_cam_calib_get(&narrow, &rel_narrow));

	/* fitted across three, with the same total number of exposures */
	rift_cam_calib_init(&wide);
	for (i = 0; i < 200; i++) {
		for (v = 0; v < 3; v++) {
			exposure_at(v, i, BIAS, 0.002f, &ocr, &oco);
			rift_cam_calib_add(&wide, &ocr, &oco);
		}
	}
	TAssert(rift_cam_calib_get(&wide, &rel_wide));
	TAssert(wide.bins_seen > narrow.bins_seen);

	/* score both at a FOURTH place, which neither was fitted on */
	rift_cam_calib_init(&heldout);
	for (i = 0; i < 200; i++) {
		exposure_at(3, i, BIAS, 0.002f, &ocr, &oco);
		rift_cam_calib_add(&heldout, &ocr, &oco);
	}
	r_narrow = rift_cam_calib_residual_px(&heldout, &rel_narrow);
	r_wide = rift_cam_calib_residual_px(&heldout, &rel_wide);

	TAssert(r_narrow > 0.0f && r_wide > 0.0f);
	TAssert(r_wide < r_narrow);   /* diversity is what conditions the fit */
}

/* A stored calibration that is 9 deg / 214 mm wrong because a sensor was moved
 * must be recognisable as such from live observation alone. */
void test_rift_cam_calib_rejects_stale_stored_calibration()
{
	rift_cam_calib c;
	posef cam_ref, cam_other, truth, stale, ocr, oco;
	int i, v;

	pose_make(&cam_ref, 0.49f, 1.56f, -1.62f, 0.0f, 1.0f, 0.0f, 0.10f);
	pose_make(&cam_other, -0.74f, 1.84f, -1.54f, 0.0f, 1.0f, 0.0f, -0.40f);
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
	TAssert(!rift_cam_calib_camera_moved(&c, &stale));

	/* now observe reality */
	rift_cam_calib_reset(&c);
	for (v = 0; v < 3; v++) {
		for (i = 0; i < 200; i++) {
			exposure_at(v, i, 0.0f, 0.002f, &ocr, &oco);
			rift_cam_calib_add(&c, &ocr, &oco);
		}
	}

	TAssert(rift_cam_calib_camera_moved(&c, &stale));   /* the file is wrong */
	TAssert(!rift_cam_calib_camera_moved(&c, &truth));  /* reality is not */
	TAssert(rift_cam_calib_residual_px(&c, &stale) >
	        50.0f * rift_cam_calib_residual_px(&c, &truth));
}

/* A wild outlier must not drag the estimate — the solve is a trimmed mean. */
void test_rift_cam_calib_rejects_outliers()
{
	rift_cam_calib c;
	posef ocr, oco, before, after;
	int i;

	rift_cam_calib_init(&c);
	for (i = 0; i < 200; i++) {
		exposure_at(0, i, 0.0f, 0.001f, &ocr, &oco);
		rift_cam_calib_add(&c, &ocr, &oco);
	}
	rift_cam_calib_get(&c, &before);

	/* a handful of solves that landed half a metre away - mislabelled LEDs */
	for (i = 0; i < 5; i++) {
		exposure_at(0, 500 + i, 0.0f, 0.001f, &ocr, &oco);
		oco.pos.x += 0.5f;
		rift_cam_calib_add(&c, &ocr, &oco);
	}
	for (i = 200; i < 260; i++) {   /* let it re-solve */
		exposure_at(0, i, 0.0f, 0.001f, &ocr, &oco);
		rift_cam_calib_add(&c, &ocr, &oco);
	}

	rift_cam_calib_get(&c, &after);
	TAssert(pose_pos_err(&before, &after) < 0.005f);
}
