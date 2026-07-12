// Copyright 2020 Jan Schmidt
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */
#ifndef __POSE_SEARCH_H__
#define __POSE_SEARCH_H__

#include "rift.h"
#include "rift-tracker-common.h"
#include "rift-sensor.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-blobwatch.h"

#include "correspondence_search.h"

typedef struct rift_pose_finder rift_pose_finder;

typedef bool (*rift_pose_finder_cb) (void *cb_data,
	rift_tracked_device *dev, rift_sensor_analysis_frame *frame,
	posef *obj_world_pose, rift_pose_metrics *score);

struct rift_pose_finder {
	int sensor_id;
	rift_sensor_camera_params *calib;

	/* Updated from fast_analysis_thread */
	bool have_camera_pose;
	bool camera_pose_changed;
	posef camera_pose;
	vec3f cam_gravity_vector;

	/* Brute force search */
	correspondence_search_t *cs;

	rift_pose_finder_cb pose_cb;
	void *pose_cb_data;
};

void rift_pose_finder_init(rift_pose_finder *pf, rift_sensor_camera_params *calib,
		rift_pose_finder_cb pose_cb, void *pose_cb_data);
void rift_pose_finder_clear(rift_pose_finder *pf);

void rift_pose_finder_process_blobs_fast(rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
	rift_tracked_device **devs);
void rift_pose_finder_process_blobs_long(rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
	rift_tracked_device **devs);
void rift_pose_finder_exp_info_to_dev_state (rift_pose_finder *pf,
  const rift_tracked_device_exposure_info *exp_dev_info, rift_sensor_frame_device_state *dev_state);

/* Offline-calibration raw observation capture. Enabled by setting
 * OHMD_RIFT_CAL_CAPTURE to an output path; JSON lines are appended there.
 * All functions are no-ops when the env var is unset. */
void rift_cal_capture_register_sensor(int sensor_id, const char *serial,
	const rift_sensor_camera_params *calib);
void rift_cal_capture_register_leds(int device_id, const rift_leds *leds);
void rift_cal_capture_obs(rift_pose_finder *pf, rift_sensor_analysis_frame *frame,
	rift_tracked_device *dev, rift_sensor_frame_device_state *dev_state,
	rift_tracked_device_exposure_info *exp_dev_info, const posef *obj_cam_pose);

#endif
