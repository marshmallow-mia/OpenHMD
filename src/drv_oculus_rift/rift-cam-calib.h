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
 *   "FastCalibrateFromHistory time: %.2f msec"
 *   "FastCalibrate: Camera moved"
 *   "Good/Bad calibration for camera %d, object %d: reprojection err %.2f, tilt err %.2f"
 *   "Insufficient Multicam calibration improvement %.1f%%: %.3f/%.3f > %.3f"
 *   "Invalid calibration: resetting history"
 *   "Camera Calibration Settled after calibration."
 *
 * Note what that sequence is: a single-frame estimate to get started, then a
 * solve over a stored HISTORY of poses, judged by a reprojection residual. It
 * is not a statistical test on a running average, and the difference matters —
 * see "Why the residual" below.
 *
 * The geometry needs no movement whatsoever. Each camera solves the device's
 * full 6-DoF pose in its OWN frame, so one exposure seen by two cameras
 * already determines the transform between them:
 *
 *     cam_b -> cam_ref  =  (obj -> cam_ref) . (obj -> cam_b)^-1
 *
 * That is `single_frame_relative()`, and it is what makes tracking work within
 * a second of plugging in, with nothing asked of the user.
 *
 * WHY A HISTORY, AND WHY IT IS STRATIFIED BY VIEWPOINT
 *
 * One exposure is geometrically sufficient but not well CONDITIONED. Per-camera
 * PnP bias depends on where the device is and which way it faces, so an
 * estimate built from one headset position bakes in that viewpoint's bias.
 * Measured on this hardware (windows-vs-linux-tracking.md §5b), with the
 * headset simply set down 28 cm away and turned 7 degrees:
 *
 *     calibration fitted at position A, scored at A     0.2 px cross-camera
 *     the same calibration, scored at position B        2.0 px
 *     fitted over A and B together, worst of the two    1.0 px
 *
 * So pooling viewpoints halves the worst case. Crucially the estimator itself
 * barely matters — a full scipy bundle adjustment over the same data scored no
 * better than the robust mean here (5.2 mm vs 4.5 mm worst case). Conditioning
 * is the whole game, which is why the history is stratified: entries are
 * bucketed by viewpoint and eviction takes from the FULLEST bucket, so a
 * headset sitting still for an hour cannot crowd out the one minute it spent
 * somewhere else. Oculus log the same idea as four per-bucket counts,
 * "sample counts: %d, %d, %d %d".
 *
 * WHY THE RESIDUAL, AND NOT A SIGMA GATE
 *
 * Telling "the headset moved" from "a camera moved" is the hard part, and a
 * sigma gate on a running mean cannot do it. Measured separation:
 *
 *     headset set down elsewhere      13.2 mm   ~2 px
 *     a camera actually knocked      226.0 mm  ~119 px
 *
 * Against a converged mean the sigma gate fires at ~15 mm — a factor of 1.14
 * away from an ordinary headset move, i.e. no separation at all. The
 * cross-camera residual separates the same two cases by ~600x. So a camera
 * move is declared from the residual, exactly as the runtime's
 * "FastCalibrate: Camera moved" is a result of its solve rather than a
 * threshold on scatter.
 *
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#ifndef RIFT_CAM_CALIB_H
#define RIFT_CAM_CALIB_H

#include <stdbool.h>
#include <stdint.h>

#include "../omath.h"

/* Co-observed exposures kept for re-solving. Each is two poses, so the whole
 * history is a few KB per sensor. */
#define RIFT_CAM_CALIB_HISTORY 192
/* Viewpoint buckets the history is stratified across. */
#define RIFT_CAM_CALIB_BINS 16
/* Enough constraints to believe an estimate at all. One exposure is
 * geometrically sufficient; this is about averaging down blob noise. */
#define RIFT_CAM_CALIB_MIN_SAMPLES 30
/* Distinct viewpoint buckets needed before the estimate is well enough
 * conditioned to call CALIBRATED rather than merely ESTIMATED. */
#define RIFT_CAM_CALIB_MIN_BINS 3
/* Residuals are expressed in pixels the way the runtime does it, against its
 * own focal length constant of 715 px (fcn.18017f370). Our CV1 sensors read
 * 716.9 and 716.7 from EEPROM, so this is within 0.3%. */
#define RIFT_CAM_CALIB_FOCAL_PX 715.0f
/* Device LED shell radius, to fold an orientation error into the same pixel
 * measure as a position error. */
#define RIFT_CAM_CALIB_SHELL_M 0.08f

/* Oculus's acceptance for a reconstruction is 2/715 normalised = 2 px
 * (fcn.18017f370); calibration is held to the same bar. */
#define RIFT_CAM_CALIB_GOOD_PX 2.0f
/* A new solve must cut the residual by at least 15% or it is rejected as
 * "Insufficient Multicam calibration improvement" (0.85, fcn.18018f9b0). */
#define RIFT_CAM_CALIB_IMPROVE_GATE 0.85f
/* Beyond this the calibration is not merely stale, it is describing a camera
 * that is no longer there: "FastCalibrate: Camera moved". Sits ~8x above the
 * ~2 px an ordinary headset move produces and ~7x below the ~119 px a real
 * 226 mm knock produced on this hardware. */
#define RIFT_CAM_CALIB_MOVED_PX 16.0f

typedef enum {
	/* nothing known — the sensor cannot contribute to tracking */
	RIFT_CAM_UNCALIBRATED = 0,
	/* a pose is available but not yet trustworthy (few samples, or all from
	 * one viewpoint, or loaded from disk and not yet checked) */
	RIFT_CAM_ESTIMATED,
	/* converged over a viewpoint-diverse history and inside the 2 px bar */
	RIFT_CAM_CALIBRATED,
} rift_cam_calib_state;

typedef struct {
	posef obj_cam_ref;    /* device pose in the reference camera's frame */
	posef obj_cam_other;  /* the same exposure, in this camera's frame */
	uint16_t bin;         /* viewpoint bucket, for stratified eviction */
} rift_cam_calib_sample;

typedef struct {
	rift_cam_calib_state state;
	bool settled;

	/* current estimate of (this camera -> reference camera) */
	quatf mean_orient;
	vec3f mean_pos;
	bool have_estimate;

	/* viewpoint-stratified history */
	rift_cam_calib_sample hist[RIFT_CAM_CALIB_HISTORY];
	uint16_t n_hist;
	uint16_t bin_count[RIFT_CAM_CALIB_BINS];
	uint16_t bins_seen;      /* distinct viewpoint buckets represented */

	uint32_t n_seen;         /* co-observed exposures ever fed in */
	uint32_t n_solves;
	uint32_t n_resets;       /* histories dropped after a camera move */

	/* residual of the current estimate over the history, in pixels */
	float residual_px;
} rift_cam_calib;

void rift_cam_calib_init(rift_cam_calib *c);

/* Seed from an existing (e.g. stored) relative pose. The result is ESTIMATED,
 * never CALIBRATED: a calibration off disk has to earn trust against live
 * observations before it is believed. */
void rift_cam_calib_seed(rift_cam_calib *c, const posef *rel);

/* Fold in one co-observed exposure. `obj_cam_ref` and `obj_cam_other` are the
 * device's pose as solved by the reference camera and by this camera, each in
 * its own camera frame. Returns true if the estimate changed. */
bool rift_cam_calib_add(rift_cam_calib *c, const posef *obj_cam_ref,
	const posef *obj_cam_other);

/* Current estimate of (this camera -> reference camera). False if there is
 * nothing usable yet. */
bool rift_cam_calib_get(const rift_cam_calib *c, posef *rel_out);

/* Mean cross-camera residual of a candidate relative pose over the stored
 * history, in pixels — the runtime's "reprojection err %.2f". This is the
 * quality measure for everything else: acceptance, settling, and telling a
 * moved camera from a moved headset. Returns -1 with an empty history. */
float rift_cam_calib_residual_px(const rift_cam_calib *c, const posef *rel);

/* True when `rel` cannot describe the observed history at all, i.e. the camera
 * itself has moved. Not a scatter test — see the header comment. */
bool rift_cam_calib_camera_moved(const rift_cam_calib *c, const posef *rel);

/* Throw away the accumulated history ("Invalid calibration: resetting
 * history"). */
void rift_cam_calib_reset(rift_cam_calib *c);

/* Compose a camera's world pose from the reference camera's world pose and a
 * relative pose. */
void rift_cam_calib_to_world(const posef *ref_world, const posef *rel,
	posef *out_world);

#endif /* RIFT_CAM_CALIB_H */
