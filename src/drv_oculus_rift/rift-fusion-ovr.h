/*
 * Complementary-filter fusion for the Rift, ported from the algorithm
 * Oculus shipped as open source in SDK 0.3.2 (LibOVR OVR_SensorFusion.cpp)
 * and documented in LaValle et al., "Head Tracking for the Oculus Rift"
 * (ICRA 2014). This is the direct ancestor of the CV1 runtime's fusion:
 * 1000 Hz gyro dead-reckoning; gravity corrects tilt and vision corrects
 * only yaw and position, each as a small per-sample fraction of the error
 * (never a visible step; snap only above 0.1 rad / 0.1 m).
 *
 * Presents the same interface shape as rift-kalman-6dof so the tracker
 * can switch between backends at runtime.
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#ifndef RIFT_FUSION_OVR_H
#define RIFT_FUSION_OVR_H

#include <stdint.h>
#include <stdbool.h>

#include "../omath.h"

#define RIFT_FUSION_OVR_MAX_SLOTS 5

typedef struct {
	bool used;
	uint64_t time_ns;
	posef pose;      /* fusion state snapshot at exposure time */
	vec3f lin_vel;
} rift_fusion_ovr_slot;

typedef struct {
	/* WorldFromImu state */
	posef pose;
	vec3f lin_vel;      /* world frame */
	vec3f accel_offset; /* world-frame accelerometer bias integral */
	vec3f ang_vel;      /* last gyro sample, body frame */
	vec3f lin_accel;    /* last world-frame acceleration (gravity removed) */

	uint64_t time_ns;
	bool have_time;
	uint32_t stage;

	/* Body-frame low-pass gravity estimate (SensorFilterBodyFrame port)
	 * plus running world-frame statistics for the confidence gate */
	vec3f grav_filter;    /* filtered accel, current body frame */
	bool grav_init;
	vec3f grav_mean;      /* EMA of world-frame filtered accel */
	float grav_var;       /* EMA variance (m/s^2)^2 */
	float grav_warmup;    /* 0..1 */

	/* Vision correction state. The error is world-frame:
	 * orient error e satisfies vision = e * recorded;
	 * pos error = vision - recorded. Corrections applied to the live
	 * state are simultaneously removed from the error (the SDK applies
	 * them to the exposure records; algebraically identical). */
	posef vision_error;
	bool have_vision_error;
	uint64_t last_vision_ns;
	bool snap_pending;    /* first fix / reacquire: apply in full */

	int num_slots;
	rift_fusion_ovr_slot slots[RIFT_FUSION_OVR_MAX_SLOTS];
} rift_fusion_ovr;

void rift_fusion_ovr_init(rift_fusion_ovr *f, const posef *init_pose, int num_delay_slots);
void rift_fusion_ovr_clear(rift_fusion_ovr *f);

void rift_fusion_ovr_prepare_delay_slot(rift_fusion_ovr *f, uint64_t time, int delay_slot);
void rift_fusion_ovr_release_delay_slot(rift_fusion_ovr *f, int delay_slot);

/* accel_saturated: the accelerometer clipped, so its DIRECTION is wrong and it
 * must not be used to correct tilt this sample. */
void rift_fusion_ovr_imu_update(rift_fusion_ovr *f, uint64_t time,
	const vec3f *ang_vel, const vec3f *accel, const vec3f *mag, bool accel_saturated);

/* replace_pending: this fix is a merged re-statement of the current
 * exposure's earlier fix(es) — it must overwrite the pending vision error
 * instead of blending with it */
void rift_fusion_ovr_pose_update(rift_fusion_ovr *f, uint64_t time,
	posef *pose, int delay_slot, float obs_scale, bool replace_pending);
void rift_fusion_ovr_position_update(rift_fusion_ovr *f, uint64_t time,
	vec3f *position, int delay_slot, float obs_scale, bool replace_pending);

void rift_fusion_ovr_get_delay_slot_pose_at(rift_fusion_ovr *f, uint64_t time, int delay_slot,
	posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error);
/* The accelerometer-derived gravity direction in the CURRENT BODY FRAME, as
 * low-passed by the tilt-correction filter. This is pure accelerometer - no
 * vision - which is what makes it usable as an independent reference for
 * gravity alignment. Returns false before the filter has warmed up. */
bool rift_fusion_ovr_get_gravity_body(rift_fusion_ovr *f, vec3f *out);

void rift_fusion_ovr_get_pose_at(rift_fusion_ovr *f, uint64_t time,
	posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error);

#endif /* RIFT_FUSION_OVR_H */
