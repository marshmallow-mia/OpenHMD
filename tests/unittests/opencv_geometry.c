/*
 * Regression tests for the OpenCV-backed geometry the constellation tracker
 * depends on: the fisheye projection / undistortion pair and the PnP solve.
 *
 * These exist because the OpenCV 4 -> 5 upgrade (2026-07-26) moved the headers
 * and silently changed which branch of rift-sensor-opencv.cpp compiled. A
 * major-version bump of OpenCV must not change the geometry, and without
 * hardware these round-trips are the only way to know.
 *
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

#include <string.h>
#include <stdlib.h>

#include "tests.h"
#include "drv_oculus_rift/rift-sensor-opencv.h"

#if HAVE_OPENCV

/* Real CV1 sensor intrinsics, from captures/lin/2026-07-12/*.jsonl
 * (sensor 0, WMTD306Q701DEK) */
static void cv1_calib(rift_sensor_camera_params *calib)
{
	memset(calib, 0, sizeof(*calib));
	calib->is_cv1 = true;
	calib->dist_fisheye = true;
	calib->width = 1280;
	calib->height = 960;
	calib->camera_matrix.m[0] = 716.934998; calib->camera_matrix.m[1] = 0.0;
	calib->camera_matrix.m[2] = 661.864014;
	calib->camera_matrix.m[3] = 0.0; calib->camera_matrix.m[4] = 716.934998;
	calib->camera_matrix.m[5] = 469.516998;
	calib->camera_matrix.m[6] = 0.0; calib->camera_matrix.m[7] = 0.0;
	calib->camera_matrix.m[8] = 1.0;
	calib->dist_coeffs[0] = 0.0708194003;
	calib->dist_coeffs[1] = -0.0232641995;
	calib->dist_coeffs[2] = 0.00510769989;
	calib->dist_coeffs[3] = -0.000523074996;
	calib->dist_coeffs[4] = 0.0;
}

#define N_TEST_LEDS 20

/* A Touch-sized LED ring in front of the camera */
static void make_leds(rift_led *leds)
{
	for (int i = 0; i < N_TEST_LEDS; i++) {
		float a = (float)i * 2.0f * (float)M_PI / N_TEST_LEDS;
		leds[i].id = i;
		leds[i].pos.x = 0.04f * cosf(a);
		leds[i].pos.y = 0.04f * sinf(a);
		leds[i].pos.z = -0.02f + 0.04f * ((i % 4) / 3.0f);
		leds[i].dir.x = 0.0f; leds[i].dir.y = 0.0f; leds[i].dir.z = -1.0f;
		leds[i].pattern = 0xff;
	}
}

/* project (fisheye, distorted pixels) then undistort must return the true
 * normalised ray. This pins the fisheye model across OpenCV versions. */
void test_opencv_fisheye_roundtrip()
{
	rift_sensor_camera_params calib;
	rift_led leds[N_TEST_LEDS];
	cv1_calib(&calib);
	make_leds(leds);

	posef pose;
	vec3f axis = {{ 0.2f, 1.0f, 0.1f }};
	ovec3f_normalize_me(&axis);
	oquatf_init_axis(&pose.orient, &axis, 0.4f);
	ovec3f_set(&pose.pos, 0.05f, -0.03f, 1.40f);   /* in front of the camera */

	vec3f projected[N_TEST_LEDS];
	rift_project_points(leds, N_TEST_LEDS, &calib, &pose, projected);

	/* every point must land inside the sensor */
	for (int i = 0; i < N_TEST_LEDS; i++) {
		TAssert(projected[i].x > 0.0f && projected[i].x < (float)calib.width);
		TAssert(projected[i].y > 0.0f && projected[i].y < (float)calib.height);
	}

	struct blob blobs[N_TEST_LEDS];
	memset(blobs, 0, sizeof(blobs));
	for (int i = 0; i < N_TEST_LEDS; i++) {
		blobs[i].x = projected[i].x;
		blobs[i].y = projected[i].y;
	}

	vec3f rays[N_TEST_LEDS];
	undistort_points(blobs, N_TEST_LEDS, rays, &calib);

	/* the recovered ray must match the true camera-frame direction */
	for (int i = 0; i < N_TEST_LEDS; i++) {
		vec3f rotated, cam;
		oquatf_get_rotated(&pose.orient, &leds[i].pos, &rotated);
		ovec3f_add(&rotated, &pose.pos, &cam);
		TAssert(cam.z > 0.1f);
		TAssert(float_eq(rays[i].x, cam.x / cam.z, 1e-4f));
		TAssert(float_eq(rays[i].y, cam.y / cam.z, 1e-4f));
	}
}

/* The PnP solve must recover a known pose from exact fisheye projections. */
void test_opencv_estimate_initial_pose()
{
	rift_sensor_camera_params calib;
	rift_led leds[N_TEST_LEDS];
	cv1_calib(&calib);
	make_leds(leds);

	posef truth;
	vec3f axis = {{ 0.1f, 1.0f, 0.2f }};
	ovec3f_normalize_me(&axis);
	oquatf_init_axis(&truth.orient, &axis, 0.3f);
	ovec3f_set(&truth.pos, -0.02f, 0.04f, 1.25f);

	vec3f projected[N_TEST_LEDS];
	rift_project_points(leds, N_TEST_LEDS, &calib, &truth, projected);

	struct blob blobs[N_TEST_LEDS];
	memset(blobs, 0, sizeof(blobs));
	for (int i = 0; i < N_TEST_LEDS; i++) {
		blobs[i].x = projected[i].x;
		blobs[i].y = projected[i].y;
		blobs[i].led_id = LED_MAKE_ID(1, i);   /* device 1, led i */
	}

	posef out;
	memset(&out, 0, sizeof(out));
	oquatf_set(&out.orient, 0, 0, 0, 1);
	int num_leds_out = 0, num_inliers = 0;
	TAssert(estimate_initial_pose(blobs, N_TEST_LEDS, 1, leds, N_TEST_LEDS,
		&calib, &out, &num_leds_out, &num_inliers, false));

	TAssert(num_leds_out == N_TEST_LEDS);
	TAssert(vec3f_eq(out.pos, truth.pos, 1e-3f));
	TAssert(quatf_eq(out.orient, truth.orient, 1e-3f));
}

#else /* !HAVE_OPENCV */

void test_opencv_fisheye_roundtrip() {}
void test_opencv_estimate_initial_pose() {}

#endif
