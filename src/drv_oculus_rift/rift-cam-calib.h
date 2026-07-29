/*
 * Automatic camera extrinsic calibration for the Rift constellation tracker.
 *
 * The headset is only trackable if the driver knows where the sensors are
 * relative to each other. This works that out from tracking data alone, with
 * no user step — which is what the Oculus runtime does. Its own log shows the
 * whole progression:
 *
 *   "Waiting for initial calibration"
 *   "Single frame calibration, camera %d, oneChanged %d"
 *   "Estimated calibration: camera %d"
 *   "FastCalibrate: camera %d, fixed %d, poses %d, residual %.3f"
 *   "Camera Calibration Settled after calibration."
 *   "Recalibrating camera %d: wasCalibrated %d, wasEstimated %d, wasAligned %d"
 *
 * The geometry is simple and needs no movement whatsoever. Each camera solves
 * the device's full 6-DoF pose in its OWN frame, so one exposure seen by two
 * cameras already determines the transform between them:
 *
 *     cam_b -> cam_ref  =  (obj -> cam_ref) . (obj -> cam_b)^-1
 *
 * Measured on a live capture with the headset sitting still: 0.107 deg /
 * 4.49 mm of scatter per single exposure, and robust-averaging 1024 of them
 * put the joint reconstruction at 0.097 px worst-camera reprojection with
 * 100% of exposures inside Oculus's 2 px bar — against 119 px and 0% for the
 * stored calibration file, which had gone 9 deg stale because a sensor was
 * moved. Motion only averages out per-camera PnP bias; it is a refinement,
 * not a precondition.
 *
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#ifndef RIFT_CAM_CALIB_H
#define RIFT_CAM_CALIB_H

#include <stdbool.h>
#include <stdint.h>

#include "../omath.h"

/* Enough constraints to believe an estimate at all. One exposure is
 * geometrically sufficient; this is purely about averaging down blob noise. */
#define RIFT_CAM_CALIB_MIN_SAMPLES 30
/* Below this dispersion the estimate has converged and can be adopted. */
#define RIFT_CAM_CALIB_SETTLE_ANG 0.0087f   /* 0.5 deg, in radians */
#define RIFT_CAM_CALIB_SETTLE_POS 0.010f    /* 10 mm */
/* A sample this far from the running mean is an outlier, not information.
 * The Python reference rejects at 3 sigma and keeps 1004/1024 on live data. */
#define RIFT_CAM_CALIB_OUTLIER_SIGMA 3.0f
/* A stored calibration disagreeing with live observation by more than this is
 * wrong, not merely drifted, and must be re-estimated rather than trusted.
 * The failure that motivated this module was 9 deg / 214 mm. */
#define RIFT_CAM_CALIB_REJECT_ANG 0.035f    /* 2 deg */
#define RIFT_CAM_CALIB_REJECT_POS 0.050f    /* 50 mm */

typedef enum {
	/* nothing known — the sensor cannot contribute to tracking */
	RIFT_CAM_UNCALIBRATED = 0,
	/* a pose is available but not yet trustworthy (few or scattered samples,
	 * or loaded from disk and not yet checked against live data) */
	RIFT_CAM_ESTIMATED,
	/* converged and verified */
	RIFT_CAM_CALIBRATED,
} rift_cam_calib_state;

typedef struct {
	rift_cam_calib_state state;
	bool settled;

	/* running robust mean of (this camera -> reference camera) */
	quatf mean_orient;
	vec3f mean_pos;
	uint32_t n;              /* samples folded into the mean */
	uint32_t n_rejected;     /* outliers dropped */

	/* dispersion about the mean, as running means of the per-sample
	 * deviation — cheap, and enough for a settle test and a sigma gate */
	float dev_ang;           /* radians */
	float dev_pos;           /* metres */
} rift_cam_calib;

void rift_cam_calib_init(rift_cam_calib *c);

/* Seed from an existing (e.g. stored) relative pose. The result is ESTIMATED,
 * never CALIBRATED: a calibration off disk has to earn trust against live
 * observations before it is believed. */
void rift_cam_calib_seed(rift_cam_calib *c, const posef *rel);

/* Fold in one co-observed exposure. `obj_cam_ref` and `obj_cam_other` are the
 * device's pose as solved by the reference camera and by this camera, each in
 * its own camera frame. Returns true if the sample was used, false if it was
 * rejected as an outlier. */
bool rift_cam_calib_add(rift_cam_calib *c, const posef *obj_cam_ref,
	const posef *obj_cam_other);

/* Current estimate of (this camera -> reference camera). False if there is
 * nothing usable yet. */
bool rift_cam_calib_get(const rift_cam_calib *c, posef *rel_out);

/* How far a candidate pose is from the running estimate. Used both to decide
 * whether a stored calibration is still valid and to detect a bumped sensor. */
void rift_cam_calib_compare(const rift_cam_calib *c, const posef *rel,
	float *out_ang, float *out_pos);

/* True when the estimate disagrees with `rel` badly enough that `rel` should be
 * thrown away rather than refined. */
bool rift_cam_calib_rejects(const rift_cam_calib *c, const posef *rel);

/* Throw away the accumulated history — a sensor moved, or the estimate went
 * bad ("Invalid calibration: resetting history"). */
void rift_cam_calib_reset(rift_cam_calib *c);

/* Compose a camera's world pose from the reference camera's world pose and a
 * relative pose. */
void rift_cam_calib_to_world(const posef *ref_world, const posef *rel,
	posef *out_world);

#endif /* RIFT_CAM_CALIB_H */
