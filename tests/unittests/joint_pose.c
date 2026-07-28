/*
 * Unit tests for the joint multi-camera pose reconstruction.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

#include <stdlib.h>
#include <string.h>

#include "tests.h"
#include "drv_oculus_rift/rift-joint-pose.h"

/* A Touch-sized LED ring: 24 points on a partial sphere */
#define N_LEDS 24

static void make_leds(vec3f leds[N_LEDS])
{
	for (int i = 0; i < N_LEDS; i++) {
		float a = (float)i * 2.0f * (float)M_PI / N_LEDS;
		float z = -0.03f + 0.06f * ((i % 5) / 4.0f);
		leds[i].x = 0.04f * cosf(a);
		leds[i].y = 0.04f * sinf(a);
		leds[i].z = z;
	}
}

static void pose_from(posef *p, float px, float py, float pz,
	float ax, float ay, float az, float angle)
{
	vec3f axis = {{ ax, ay, az }};
	ovec3f_normalize_me(&axis);
	oquatf_init_axis(&p->orient, &axis, angle);
	ovec3f_set(&p->pos, px, py, pz);
}

/* Place a camera at `eye` looking at `target`: camera +Z is the view axis. */
static void look_at(posef *cam, float ex, float ey, float ez,
	const vec3f *target)
{
	vec3f fwd, eye = {{ ex, ey, ez }};
	const vec3f z = {{ 0.0f, 0.0f, 1.0f }};

	ovec3f_set(&cam->pos, ex, ey, ez);
	ovec3f_subtract(target, &eye, &fwd);
	ovec3f_normalize_me(&fwd);
	oquatf_from_vectors(&cam->orient, &z, &fwd);
}

/* Fill a view with the exact rays a camera would see for `truth`, keeping only
 * LEDs that land in front of the camera. */
static void project_view(rift_joint_view *view, const vec3f *leds, int num_leds,
	const posef *truth, float noise)
{
	quatf inv = view->camera_pose.orient;
	oquatf_inverse(&inv);

	view->n_points = 0;
	for (int i = 0; i < num_leds; i++) {
		vec3f rotated, world, rel, cam;
		oquatf_get_rotated(&truth->orient, &leds[i], &rotated);
		ovec3f_add(&rotated, &truth->pos, &world);
		ovec3f_subtract(&world, &view->camera_pose.pos, &rel);
		oquatf_get_rotated(&inv, &rel, &cam);
		if (cam.z < 0.2f)
			continue;

		rift_joint_point *pt = view->points + view->n_points++;
		pt->led_index = (uint8_t)i;
		pt->ray[0] = cam.x / cam.z;
		pt->ray[1] = cam.y / cam.z;
		if (noise > 0.0f) {
			/* deterministic pseudo-noise, +/- noise (normalised units) */
			pt->ray[0] += noise * (((i * 37) % 17) / 8.0f - 1.0f);
			pt->ray[1] += noise * (((i * 53) % 13) / 6.0f - 1.0f);
		}
	}
}

/* Exact data, perturbed start: the solve must recover the truth. */
void test_rift_joint_pose_exact()
{
	vec3f leds[N_LEDS];
	make_leds(leds);

	posef truth;
	pose_from(&truth, 0.15f, 1.10f, -1.30f, 0.3f, 1.0f, 0.2f, 0.6f);

	rift_joint_view views[2];
	memset(views, 0, sizeof(views));
	look_at(&views[0].camera_pose, 0.5f, 1.6f, -1.6f, &truth.pos);
	look_at(&views[1].camera_pose, -0.7f, 1.8f, -1.5f, &truth.pos);
	views[0].focal_px = views[1].focal_px = 716.9f;
	project_view(&views[0], leds, N_LEDS, &truth, 0.0f);
	project_view(&views[1], leds, N_LEDS, &truth, 0.0f);

	/* start 4 cm and ~6 degrees away */
	posef init = truth;
	init.pos.x += 0.04f;
	init.pos.z -= 0.02f;
	quatf perturb;
	vec3f axis = {{ 0.0f, 1.0f, 0.3f }};
	ovec3f_normalize_me(&axis);
	oquatf_init_axis(&perturb, &axis, 0.10f);
	quatf tmp;
	oquatf_mult(&perturb, &init.orient, &tmp);
	init.orient = tmp;

	posef out;
	rift_joint_result res;
	TAssert(rift_joint_pose_solve(leds, N_LEDS, views, 2, &init, &out, &res));

	TAssert(vec3f_eq(out.pos, truth.pos, 1e-4f));
	TAssert(quatf_eq(out.orient, truth.orient, 1e-4f));
	TAssert(res.worst_view_px < 0.01f);
	TAssert(res.n_views == 2);
}

/* Two cameras must beat one. With a single steeply-viewed camera the depth is
 * weakly constrained, so a pose that fits that camera can be centimetres off;
 * adding the second view has to fix it. This is the whole premise of the
 * joint solve (windows-vs-linux-tracking.md section 2). */
void test_rift_joint_pose_two_cameras_beat_one()
{
	vec3f leds[N_LEDS];
	make_leds(leds);

	posef truth;
	pose_from(&truth, 0.0f, 0.90f, -1.20f, 0.1f, 1.0f, 0.0f, 0.35f);

	rift_joint_view views[2];
	memset(views, 0, sizeof(views));
	look_at(&views[0].camera_pose, 0.6f, 1.7f, -1.7f, &truth.pos);
	look_at(&views[1].camera_pose, -0.8f, 1.7f, -1.4f, &truth.pos);
	views[0].focal_px = views[1].focal_px = 716.9f;

	/* modest measurement noise, same in both views */
	project_view(&views[0], leds, N_LEDS, &truth, 2.0e-4f);
	project_view(&views[1], leds, N_LEDS, &truth, 2.0e-4f);

	posef init = truth;
	init.pos.y += 0.03f;

	posef one, two;
	rift_joint_result r_one, r_two;
	TAssert(rift_joint_pose_solve(leds, N_LEDS, views, 1, &init, &one, &r_one));
	TAssert(rift_joint_pose_solve(leds, N_LEDS, views, 2, &init, &two, &r_two));

	vec3f err_one, err_two;
	ovec3f_subtract(&one.pos, &truth.pos, &err_one);
	ovec3f_subtract(&two.pos, &truth.pos, &err_two);

	/* the joint solve must be at least as good, and in this geometry clearly
	 * better, than the single-camera solve */
	TAssert(ovec3f_get_length(&err_two) <= ovec3f_get_length(&err_one));
	TAssert(ovec3f_get_length(&err_two) < 0.005f);
	TAssert(r_two.n_views == 2);
}

/* A pose that satisfies one camera but not the other must NOT be reported as
 * a good joint solve - that is the extrinsic-quality signal the runtime uses
 * ("Bad calibration for camera %d ... reprojection err %.2f"). */
void test_rift_joint_pose_detects_bad_extrinsics()
{
	vec3f leds[N_LEDS];
	make_leds(leds);

	posef truth;
	pose_from(&truth, 0.0f, 1.00f, -1.25f, 0.0f, 1.0f, 0.0f, 0.2f);

	rift_joint_view views[2];
	memset(views, 0, sizeof(views));
	look_at(&views[0].camera_pose, 0.5f, 1.6f, -1.6f, &truth.pos);
	look_at(&views[1].camera_pose, -0.7f, 1.8f, -1.5f, &truth.pos);
	views[0].focal_px = views[1].focal_px = 716.9f;
	project_view(&views[0], leds, N_LEDS, &truth, 0.0f);
	project_view(&views[1], leds, N_LEDS, &truth, 0.0f);

	/* control: with correct extrinsics the truth pose fits both cameras */
	posef clean;
	rift_joint_result clean_res;
	TAssert(rift_joint_pose_solve(leds, N_LEDS, views, 2, &truth, &clean, &clean_res));
	TAssert(clean_res.worst_view_px < 0.05f);

	/* Now rotate camera 1 by 1 degree, as a bumped sensor would.
	 * Measured sensitivity of this residual (see rift-joint-pose.c notes):
	 *   ~4.3 px per degree of camera rotation error
	 *   ~1 px per 16 mm of camera translation error
	 * so Oculus's 2 px acceptance corresponds to roughly 0.47 deg / 32 mm of
	 * extrinsic error. A pure translation is ~77% absorbed by moving the
	 * object, which is why rotation is much the stronger signal. */
	vec3f axis = {{ 0.0f, 1.0f, 0.0f }};
	quatf bump, rotated;
	oquatf_init_axis(&bump, &axis, 1.0f / 57.2957795f);
	oquatf_mult(&bump, &views[1].camera_pose.orient, &rotated);
	views[1].camera_pose.orient = rotated;

	posef out;
	rift_joint_result res;
	TAssert(rift_joint_pose_solve(leds, N_LEDS, views, 2, &truth, &out, &res));

	/* no single pose can satisfy both cameras any more */
	TAssert(res.worst_view_px > 2.0f);
}

/* Not enough correspondences must fail cleanly and leave the pose alone. */
void test_rift_joint_pose_rejects_thin_data()
{
	vec3f leds[N_LEDS];
	make_leds(leds);

	posef truth;
	pose_from(&truth, 0.0f, 1.0f, -1.2f, 0.0f, 1.0f, 0.0f, 0.0f);

	rift_joint_view view;
	memset(&view, 0, sizeof(view));
	look_at(&view.camera_pose, 0.5f, 1.6f, -1.6f, &truth.pos);
	view.focal_px = 716.9f;
	project_view(&view, leds, N_LEDS, &truth, 0.0f);
	view.n_points = 2;   /* below MIN_POINTS_TOTAL */

	posef out;
	memset(&out, 0xAB, sizeof(out));
	posef untouched = out;
	rift_joint_result res;
	TAssert(!rift_joint_pose_solve(leds, N_LEDS, &view, 1, &truth, &out, &res));
	TAssert(memcmp(&out, &untouched, sizeof(out)) == 0);
}
