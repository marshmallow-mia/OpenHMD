/*
 * Joint multi-camera pose reconstruction for the Rift constellation tracker.
 *
 * Solves ONE device pose from the LED correspondences of every camera that saw
 * the device in the same exposure, with the camera extrinsics held fixed. This
 * is the shape of the Oculus runtime's own reconstruction: Rift.dll calls its
 * reconstruction routine with a camera index of -1 meaning "all cameras", and
 * only re-calls it with a concrete index as a single-camera fallback
 * ("Object %d using fallback camera %d, cameras %d"). See
 * rift-cv1-center/windows-vs-linux-tracking.md section 2.
 *
 * Why this matters: solving each camera separately and averaging the resulting
 * poses cannot remove per-camera PnP bias, because each solution is already
 * optimal in its own image and wrong in the other's. Measured offline on
 * recorded captures, the per-camera solutions fit their own camera to 0.09 px
 * while reprojecting into the other camera at 5.6 px; the averaged pose is
 * wrong in both (2.8 px). One joint solve reaches 0.66 px in the worst camera
 * and cuts frame-to-frame jitter 11x.
 *
 * Residuals are in undistorted normalised camera rays (x/z, y/z), matching the
 * runtime's own normalised acceptance threshold of 2/715.
 *
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#ifndef RIFT_JOINT_POSE_H
#define RIFT_JOINT_POSE_H

#include <stdbool.h>
#include <stdint.h>

#include "../omath.h"

/* CV1 HMD carries 34 LEDs, Touch 24. */
#define RIFT_JOINT_MAX_POINTS 48
/* Matches RIFT_MAX_SENSORS; kept local so this module stands alone. */
#define RIFT_MAX_SENSORS_JOINT 4

typedef struct {
	uint8_t led_index;
	float ray[2];  /* undistorted normalised camera ray: (x/z, y/z) */
} rift_joint_point;

/* One camera's view of the device in a single exposure */
typedef struct {
	posef camera_pose;   /* camera -> world */
	float focal_px;      /* to express residuals in pixels */
	int n_points;
	rift_joint_point points[RIFT_JOINT_MAX_POINTS];
} rift_joint_view;

typedef struct {
	float rms_px;          /* over all views */
	float worst_view_px;   /* worst per-view RMS - the acceptance metric */
	int n_points;          /* correspondences actually used */
	int n_views;
	int iterations;
} rift_joint_result;

/*
 * leds / num_leds: the device's LED model, positions in the device frame.
 * views / n_views: per-camera correspondence sets for ONE exposure.
 * init_pose: starting estimate, device -> world (e.g. the per-camera solve).
 * out_pose: refined device -> world pose.
 *
 * Returns false if there is not enough data to solve, or the solve diverged;
 * out_pose is then untouched. A caller should fall back to the single-camera
 * result in that case, as the runtime does.
 */
bool rift_joint_pose_solve(const vec3f *leds, int num_leds,
	const rift_joint_view *views, int n_views,
	const posef *init_pose, posef *out_pose, rift_joint_result *out_result);

/* Reprojection RMS in pixels of a given pose, per view. Exposed for scoring
 * and for the extrinsic-quality check: a joint residual that cannot be driven
 * below a few px means the extrinsics are wrong, not the pose. */
float rift_joint_pose_rms_px(const vec3f *leds, int num_leds,
	const rift_joint_view *view, const posef *pose);

#endif /* RIFT_JOINT_POSE_H */
