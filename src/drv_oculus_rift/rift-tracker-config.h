// Copyright 2021, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 */

/* Oculus Rift driver - tracking config store */

#include "rift-tracker-common.h"
#include "rift-sensor.h"

#ifndef __RIFT_TRACKER_CONFIG_H__
#define __RIFT_TRACKER_CONFIG_H__

typedef struct rift_tracker_config_s rift_tracker_config;
typedef struct rift_tracker_sensor_config_s rift_tracker_sensor_config;

struct rift_tracker_sensor_config_s {
	char serial_no[RIFT_SENSOR_SERIAL_LEN+1];
	posef pose;
	/* How many distinct viewpoints the calibration that produced this pose
	 * was fitted over. A fit from one headset position is superb at that
	 * position and poor elsewhere, so without this a fresh session with the
	 * headset sitting still would happily replace a well-conditioned
	 * calibration with an overfit one - it beats the incumbent on its own
	 * narrow history every time. 0 for configs written before this existed,
	 * or by the offline solver. */
	int viewpoints;
};

struct rift_tracker_config_s {
	bool modified;

	/* Room configuration */
	vec3f room_center_offset;
	float room_yaw_offset; /* Radians */

	int n_sensors;
	rift_tracker_sensor_config sensors[RIFT_MAX_SENSORS];
};

void rift_tracker_config_init(rift_tracker_config *config);
void rift_tracker_config_load(ohmd_context *ctx, rift_tracker_config *config);
void rift_tracker_config_save(ohmd_context *ctx, rift_tracker_config *config);

void rift_tracker_config_get_room_pose_offset(rift_tracker_config *config, posef *room_pose_offset);

void rift_tracker_config_set_sensor_pose(rift_tracker_config *config, const char *serial_no, posef *pose);
bool rift_tracker_config_get_sensor_pose(rift_tracker_config *config, const char *serial_no, posef *pose);

/* Viewpoint coverage of the stored calibration; 0 if unknown. */
void rift_tracker_config_set_sensor_viewpoints(rift_tracker_config *config, const char *serial_no, int viewpoints);
int rift_tracker_config_get_sensor_viewpoints(rift_tracker_config *config, const char *serial_no);

#endif
