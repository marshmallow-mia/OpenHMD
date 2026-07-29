// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - positional tracking interface */

#include "rift.h"
#include "rift-kalman-6dof.h"
#include "rift-tracker-common.h"
#include "rift-sensor.h"
#include "rift-sensor-pose-helper.h"
#include "rift-joint-pose.h"

#ifndef __RIFT_TRACKER_H__
#define __RIFT_TRACKER_H__

rift_tracker_ctx *rift_tracker_new (ohmd_context* ohmd_ctx, const uint8_t radio_id[5]);

rift_tracked_device *rift_tracker_add_device (rift_tracker_ctx *ctx, int device_id, posef *imu_pose, posef *model_pose, rift_leds *leds, rift_tracked_device_imu_calibration *calib);
void rift_tracker_on_new_exposure (rift_tracker_ctx *ctx, uint32_t hmd_ts, uint16_t exposure_count, uint32_t exposure_hmd_ts, uint8_t led_pattern_phase);
uint8_t rift_tracker_get_device_list(rift_tracker_ctx *tracker_ctx, rift_tracked_device **dev_list);

bool rift_tracker_frame_captured (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *info, const char *source);
void rift_tracker_frame_release (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_local_ts, rift_tracker_exposure_info *info, const char *source);

void rift_tracker_free (rift_tracker_ctx *ctx);

/* Per-sample IMU health. A saturated axis is not a measurement - the true
 * value is somewhere beyond the rail - so the fusion must stop trusting it
 * rather than integrate the clipped number. Oculus tracks the same two
 * conditions and inflates the corresponding sigma while they last
 * ("Begin Gyro saturation: %.4f, orient sigma %.2f", "Acc saturation: %d
 * samples, pos sigma %.1f"). */
typedef enum {
	RIFT_IMU_SAMPLE_OK = 0,
	RIFT_IMU_ACCEL_SATURATED = (1 << 0),
	RIFT_IMU_GYRO_SATURATED = (1 << 1),
} rift_imu_sample_flags;

void rift_tracked_device_imu_update(rift_tracked_device *dev, uint64_t local_ts, uint32_t device_ts, float dt, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field, rift_imu_sample_flags flags);
void rift_tracked_device_get_view_pose(rift_tracked_device *dev, posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel);
uint64_t rift_tracked_device_get_pose_age_ns(rift_tracked_device *dev, uint64_t now_local_ts);

bool rift_tracked_device_get_latest_exposure_info_pose (rift_tracked_device *dev, rift_tracked_device_exposure_info *dev_info);

/* `view` is this sensor's LED correspondences for the exposure (may be NULL).
 * When two or more sensors supply one for the same exposure, the tracker
 * reconstructs a single pose from all of them instead of averaging their
 * separate solutions - see rift-joint-pose.h. */
/* Accelerometer-derived gravity direction in the device MODEL frame, for
 * gravity alignment. Vision plays no part in it, so it is an independent
 * reference for how the room frame relates to true up. False if unavailable
 * (not warmed up, or a fusion backend that does not track it). */
bool rift_tracked_device_get_gravity_model(rift_tracked_device *dev_base, vec3f *out);

bool rift_tracked_device_model_pose_update(rift_tracked_device *dev_base, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *exposure_info, rift_pose_metrics *score, posef *pose, const rift_joint_view *view, const char *source);
void rift_tracked_device_frame_release (rift_tracked_device *dev_base, rift_tracker_exposure_info *info);

/* Automatic camera extrinsic calibration (rift-cam-calib.h). add_calib_obs
 * takes the device's pose in ONE sensor's own frame; apply places a sensor
 * once its relative pose to the anchor has converged. Both are called from
 * the sensor's own analysis thread. */
void rift_tracker_add_calib_obs(rift_tracker_ctx *ctx, rift_sensor_ctx *sensor,
	rift_tracked_device *dev, rift_tracker_exposure_info *exposure_info,
	const posef *obj_cam_pose);
void rift_tracker_cam_calib_apply(rift_tracker_ctx *ctx, rift_sensor_ctx *sensor);

void rift_tracker_update_sensor_pose(rift_tracker_ctx *tracker_ctx, rift_sensor_ctx *sensor, posef *new_pose);
void rift_tracker_extrinsic_refine_apply(rift_tracker_ctx *ctx, rift_sensor_ctx *sensor);

#endif
