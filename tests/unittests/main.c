/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 * Copyright (C) 2013 Fredrik Hultin.
 * Copyright (C) 2013 Jakob Bornecrantz.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

/* Unit Tests - Main */

#include <string.h>
#include "tests.h"

bool float_eq(float a, float b, float t)
{
	return fabsf(a - b) < t;
}

#define Test(_t) printf("   "#_t); _t(); printf("%*sok\n", 50 - (int)strlen(#_t), "");

int main()
{
	printf("vec3f tests\n");
	Test(test_ovec3f_normalize_me);
	Test(test_ovec3f_get_length);
	Test(test_ovec3f_get_angle);
	Test(test_ovec3f_get_dot);
	Test(test_ovec3f_inverse);
	printf("\n");
	
	printf("quatf tests\n");
	Test(test_oquatf_init_axis);
	Test(test_oquatf_get_rotated);
	Test(test_oquatf_get_dot);
	Test(test_oquatf_inverse);
	Test(test_oquatf_diff);
	Test(test_oquatf_decompose_swing_twist);
	printf("\n");

	printf("pose tests\n");
	Test(test_oposef_init);
	Test(test_oposef_inverse);
	Test(test_oposef_apply);

	printf("\n");

	printf("joint pose tests\n");
	Test(test_rift_joint_pose_exact);
	Test(test_rift_joint_pose_two_cameras_beat_one);
	Test(test_rift_joint_pose_detects_bad_extrinsics);
	Test(test_rift_joint_pose_rejects_thin_data);
	printf("\n");

	printf("kalman 6dof tests\n");
	Test(test_rift_kalman_stationary);
	Test(test_rift_kalman_five_delay_slots);
	Test(test_rift_kalman_two_sample_rates);
	Test(test_rift_kalman_delayed_position_update);
	Test(test_rift_fusion_ovr_saturated_accel_ignored);
	Test(test_rift_kalman_rejects_non_finite_imu);
	Test(test_rift_fusion_ovr_camera_moved);
	printf("\n");

	printf("camera calibration tests\n");
	Test(test_rift_cam_calib_single_exposure_is_enough);
	Test(test_rift_cam_calib_averages_and_settles);
	Test(test_rift_cam_calib_rejects_stale_stored_calibration);
	Test(test_rift_cam_calib_rejects_outliers);
	Test(test_rift_cam_calib_headset_move_is_not_a_camera_move);
	Test(test_rift_cam_calib_history_spans_viewpoints);
	printf("\n");

	printf("sync monitor tests\n");
	Test(test_rift_sync_monitor_clean_stream);
	Test(test_rift_sync_monitor_repeated_exposure);
	Test(test_rift_sync_monitor_dropped_exposure);
	Test(test_rift_sync_monitor_latency_outlier);
	Test(test_rift_sync_monitor_late_pose);
	printf("\n");

	printf("opencv geometry tests\n");
	Test(test_opencv_fisheye_roundtrip);
	Test(test_opencv_estimate_initial_pose);
	printf("\n");

	printf("high level tests\n");
	Test(test_highlevel_open_close_device);
	Test(test_highlevel_open_close_many_devices);
	printf("\n");

	printf("all a-ok\n");
	return 0;
}
