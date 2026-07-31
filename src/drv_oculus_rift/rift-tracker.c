/*
 * Rift position tracking
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019 Jan Schmidt
 * SPDX-License-Identifier: BSL-1.0
 */

#define _GNU_SOURCE

#include <libusb.h>
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include <inttypes.h>
#include <stdarg.h>
#include <time.h>
#include <math.h>

#include "ohmd-video.h"

#include "../exponential-filter.h"
#include "rift-fusion-ovr.h"
#include "rift-tracker.h"
#include "rift-tracker-config.h"
#include "rift-sync-monitor.h"
#include "rift-cam-calib.h"
#include "rift-sensor.h"
#include "rift-sensor-usb.h"

#include "rift-sensor.h"
#include "rift-sensor-maths.h"
#include "rift-sensor-opencv.h"
#include "rift-sensor-pose-helper.h"
#include "rift-sensor-pose-search.h"

#include "rift-debug-draw.h"

#include "ohmd-pipewire.h"

#define ASSERT_MSG(_v, label, ...) if(!(_v)){ fprintf(stderr, __VA_ARGS__); goto label; }

/* Number of IMU observations we accumulate before output */
#define RIFT_MAX_PENDING_IMU_OBSERVATIONS 1000

/* Number of state slots to use for quat/position updates. Each slot holds one
 * exposure open until every sensor that saw it has reported, so this is also
 * how long a slow sensor has to deliver before its observation is dropped
 * entirely -- and a dropped observation costs the joint reconstruction its
 * second view, not just one correction. At 3 that was ~58 ms at the 52 Hz
 * exposure rate, and the live test measured a reacquiring sensor at ~100 ms.
 * Both fusion backends already size their arrays for 5 (MAX_DELAY_SLOTS,
 * RIFT_FUSION_OVR_MAX_SLOTS), so the headroom is free. */
#define NUM_POSE_DELAY_SLOTS 5

/* Number of exposure history slots to keep */
#define NUM_EXPOSURE_HISTORY 3

/* Length of time (milliseconds) we will interpolate position before declaring
 * tracking lost */
#define POSE_LOST_THRESHOLD 500

/* Length of time (milliseconds) we can ignore orientation from cameras before
 * we force an update */
#define POSE_LOST_ORIENT_THRESHOLD 100

/* Length of time (milliseconds) that we are allowed to lose tracking because
 * there's no free delay slots before we'll warn about it. 60ms is ~3 frames
 */
#define NO_FREE_DELAY_SLOT_THRESHOLD 60

/* Output correction bleeding: each discrete optical Kalman update is absorbed
 * into an offset between the fusion state and the displayed pose, which then
 * bleeds away smoothly (never a visible step). Bleed speeds up while the head
 * is moving, when the eye can't detect it. Offsets larger than the clamps
 * (bad prior / re-acquisition) step through immediately. */
#define OUT_CORR_TAU 0.3f                    /* bleed time constant, seconds */
#define OUT_CORR_ANG_REF 1.0f                /* rad/s of head rotation that doubles the bleed rate */
#define OUT_CORR_LIN_REF 0.5f                /* m/s of head motion that doubles the bleed rate */
#define OUT_CORR_MIN_ANG_RATE DEG_TO_RAD(0.5f) /* minimum bleed, rad/s */
#define OUT_CORR_MIN_LIN_RATE 0.005f         /* minimum bleed, m/s */
#define OUT_CORR_MAX_ANG DEG_TO_RAD(3.0f)    /* clamp: larger corrections step */
#define OUT_CORR_MAX_LIN 0.05f               /* clamp: larger corrections step, m */

#define MIN_ROT_ERROR DEG_TO_RAD(25)
#define MIN_POS_ERROR 0.1

typedef struct rift_tracked_device_priv rift_tracked_device_priv;
typedef struct rift_tracker_pose_report rift_tracker_pose_report;
typedef struct rift_tracker_pose_delay_slot rift_tracker_pose_delay_slot;

typedef struct rift_tracked_device_imu_observation rift_tracked_device_imu_observation;

struct rift_tracked_device_imu_observation {
	uint64_t local_ts;
	uint64_t device_ts;
	float dt;

	vec3f ang_vel;
	vec3f accel;
	vec3f mag;
};

struct rift_tracker_pose_report {
		bool report_used; /* TRUE if this report has been integrated */
		bool orient_used; /* TRUE if the report's orientation was applied */
		const char *source; /* serial of the sensor that produced it */
		posef pose;
		rift_pose_metrics score;
		float obs_scale; /* confidence tier the report was integrated with */
		/* This sensor's LED correspondences for the exposure, so a later
		 * report can reconstruct one pose across all of them */
		bool have_view;
		rift_joint_view view;
};

struct rift_tracker_pose_delay_slot {
	int slot_id;		/* Index of the slot */
	bool valid;			/* true if the exposure info was set */
	int use_count;	/* Number of frames using this slot */

	uint64_t device_time_ns; /* Device time this slot is currently tracking */

	/* rift_tracked_device_model_pose_update stores the observed poses here */
	int n_pose_reports;
	rift_tracker_pose_report pose_reports[RIFT_MAX_SENSORS];
	/* Number of reports we used from the supplied ones */
	int n_used_reports;
};

/* Internal full tracked device struct */
struct rift_tracked_device_priv {
	rift_tracked_device base;

	int index; /* Index of this entry in the devices array for the tracker and exposures */
	rift_tracker_ctx *tracker; /* owning tracker (for extrinsic refinement) */

	/* LED positions in the model frame, unpacked from base.leds so the joint
	 * solver can take a plain vec3f array. Owned by this struct. */
	vec3f *led_pos;
	int n_led_pos;
	/* Joint reconstruction telemetry */
	uint32_t joint_solved, joint_rejected, joint_single;
	/* Observations that arrived after their exposure's delay slot was
	 * recycled. These are invisible in the joint counters -- the exposure
	 * simply looks single-camera -- so count them separately, otherwise a
	 * timing problem is indistinguishable from a sensor not seeing the
	 * device. */
	uint32_t reports_dropped_late;
	uint32_t imu_saturated_samples;
	rift_sync_monitor sync;

	ohmd_mutex *device_lock;

	/* 6DOF Kalman Filter */
	rift_kalman_6dof_filter ukf_fusion;
	/* OVR-SDK-style complementary filter (default backend) */
	rift_fusion_ovr ovr_fusion;

	/* Account keeping for UKF fusion slots */
	int n_delay_slots;
	int delay_slot_index;
	rift_tracker_pose_delay_slot delay_slots[NUM_POSE_DELAY_SLOTS];
	/* Track the time we last started having no free delay slots (or 0 if never) */
	uint64_t last_no_free_delay_slot;

	/* The pose of the device relative to the IMU 3D space */
	posef device_from_fusion;

	/* The pose of the IMU relative to the LED model space */
	posef fusion_from_model;
	posef model_from_fusion;

	uint32_t last_device_ts;
	uint64_t device_time_ns;

	/* Host-clock arrival time of the most recent IMU sample. The fused pose is
	 * a state estimate at this instant, so (now - last_imu_local_ts) is the age
	 * of the pose we hand out — which is what SteamVR's DriverPose_t
	 * poseTimeOffset wants, so it can predict forward from the right epoch. */
	uint64_t last_imu_local_ts;

	uint64_t last_observed_orient_ts;
	uint64_t last_observed_pose_ts;
	posef last_observed_pose;

	/* same-exposure merge telemetry */
	int merge_count;
	double merge_shift_accum;

	uint64_t last_acquired_pose_lock_ts;

	/* Reported view pose (to the user) and model pose (for the tracking) respectively */
	uint64_t last_reported_pose;
	posef reported_pose;
	vec3f reported_ang_vel;
	vec3f reported_lin_vel;
	vec3f reported_lin_accel;

	/* EMA state for the exported-velocity low-pass (see get_view_pose) */
	vec3f vel_filt;
	vec3f ang_vel_filt;
	uint64_t vel_filt_ts;

	posef model_pose;

	exp_filter_pose pose_output_filter;

	/* Output correction offset (global frame): absorbs discrete optical-update
	 * jumps so the displayed pose stays continuous; bled off in
	 * rift_tracked_device_get_view_pose */
	quatf out_corr_orient;
	vec3f out_corr_pos;

	int num_pending_imu_observations;
	rift_tracked_device_imu_observation pending_imu_observations[RIFT_MAX_PENDING_IMU_OBSERVATIONS];

	ohmd_pw_debug_stream *debug_metadata;
	FILE *debug_file;
};

/* Online extrinsic refinement (Oculus-runtime style). Whenever two cameras
 * verify the SAME exposure of the HMD, their relative geometry error is a
 * direct optical measurement M = pose_anchor o inv(pose_other) — the fused
 * prior is NOT in the loop, so unlike the old servo-toward-fusion attempt
 * this cannot random-walk. Sensor 0 stays fixed (anchors the world); the
 * others are corrected toward agreement with it, slowly, and only when the
 * headset has moved through enough space that per-camera PnP bias (which
 * looks like a large phantom mismatch at static steep views) averages out. */
typedef struct rift_extrinsic_refine {
	int n_meas;
	vec3f mean_dpos;    /* incremental mean of M position */
	quatf mean_dorient; /* incremental slerp mean of M orientation */
	vec3f span_min, span_max; /* HMD positions covered by this window */
	vec3f fwd_sum;      /* sum of HMD floor-plane forward vectors: view
	                     * diversity gate — per-camera PnP bias is view-
	                     * dependent and only cancels across gaze angles */
	uint64_t last_apply_ts;
	uint64_t last_save_ts;
	posef start_pose;   /* camera pose at first apply (net-drift telemetry) */
	bool have_start;
	/* Calibration settle state: unsettled while refinement is still moving
	 * this sensor, settled after enough consecutive quiet evaluations. */
	bool settled;
	int quiet_rounds;
	double applied_pos_total; /* sum of applied step sizes (telemetry) */
	double applied_ang_total;
} rift_extrinsic_refine;

/* One exposure's worth of per-sensor object->camera poses, waiting to be
 * paired. Sensors deliver ms apart in racing order, so a couple of exposures
 * are kept in flight; the exposure counter is the key both sensors agree on. */
#define RIFT_CALIB_EXP_SLOTS 4

typedef struct {
	bool valid;
	uint16_t count;              /* exposure counter */
	bool have[RIFT_MAX_SENSORS];
	posef obj_cam[RIFT_MAX_SENSORS];
} rift_calib_exposure;

struct rift_tracker_ctx_s
{
	ohmd_context* ohmd_ctx;
	libusb_context *usb_ctx;
	ohmd_mutex *tracker_lock;

	/* exposure/frame timing health (rift-sync-monitor.h) */
	rift_sync_monitor sync;

	ohmd_mutex *refine_lock;
	rift_extrinsic_refine refine[RIFT_MAX_SENSORS];

	/* Automatic extrinsic calibration (rift-cam-calib.h): every sensor's
	 * pose of the HMD in its own frame, paired per exposure against the
	 * anchor sensor's. */
	ohmd_mutex *calib_lock;
	rift_calib_exposure calib_exp[RIFT_CALIB_EXP_SLOTS];
	int calib_exp_next;
	rift_cam_calib cam_calib[RIFT_MAX_SENSORS];
	bool cam_calib_adopted[RIFT_MAX_SENSORS];
	/* set when a camera move was detected and the history dropped; cleared
	 * once the rebuilt estimate has been adopted */
	bool cam_calib_recovering[RIFT_MAX_SENSORS];
	uint32_t cam_calib_pairs;
	uint32_t cam_calib_obs;   /* observations that reached the feed at all */

	ohmd_thread* usb_thread;
	int usb_completed;

	int exposure_history_index;
	int exposure_history_size;
	rift_tracker_exposure_info exposure_history[NUM_EXPOSURE_HISTORY];

	rift_tracker_config config;

	rift_sensor_ctx *sensors[RIFT_MAX_SENSORS];
	uint8_t n_sensors;

	rift_tracked_device_priv devices[RIFT_MAX_TRACKED_DEVICES];
	uint8_t n_devices;
};

/* Fusion backend selection. Default is the UKF (rift-kalman-6dof.c);
 * OHMD_RIFT_FUSION=ovr selects the OVR-SDK-style complementary filter ported
 * from Oculus SDK 0.3.2 (rift-fusion-ovr.c).
 *
 * The default was the OVR filter from the commit that introduced it (b665454)
 * until 2026-07-31, and it is why the headset "overshoots and settles" after
 * every movement. Bisected against pristine upstream: the artifact is absent at
 * b665454^ and present from b665454 on, and switching this one selector at
 * current HEAD removes it while leaving everything else in place.
 *
 * The mechanism is visible in the gains. rift-fusion-ovr.c applies
 * corr_{pos,vel,accel} = err * GAIN * dt, a third-order observer with
 * characteristic polynomial s^3 + Kp s^2 + Kv s + Ka. With the shipped
 * GAIN_POS/GAIN_VEL/GAIN_ACCEL that factors into a well-damped pair at ~1 Hz
 * AND a real pole at -0.559, i.e. a mode with a 1.8 s time constant. Ka is low
 * relative to Kp and Kv (placing all three poles together at Kp=10 wants
 * Kv=33.3, Ka=36.9 against the shipped 50/25), and that is what strands the
 * slow root. The UKF has no such observer.
 *
 * This is a default change, not a deletion: the OVR filter was ported for
 * reasons that still stand, and which of the two is better on latency, rest
 * jitter and dropout recovery is NOT yet measured - only the overshoot is.
 * Score both with tools/settle_profile.py in the rift-cv1-center repo against
 * the Oculus runtime's 14.21 mm / tau 0.85 s before treating this as settled. */
static bool use_ovr_fusion(void)
{
	static int use = -1;
	if (use == -1) {
		const char *e = getenv("OHMD_RIFT_FUSION");
		use = (e && strcmp(e, "ovr") == 0);
	}
	return use;
}

/* Tell every tracked device that a sensor's extrinsics moved. Each fix taken
 * through that sensor was computed against a world that has since shifted, so
 * the fusion must stop treating its pending error as current. Oculus does the
 * same and says so: "Ekf CameraPoseChange: dt ... dr ..." followed by
 * "Ekf Reset on CameraPoseChange". */
static void notify_camera_moved(rift_tracker_ctx *ctx, const vec3f *dpos, float dang)
{
	/* how wrong the pose could now be, as a variance */
	const double pos_var = (double)ovec3f_get_length(dpos) * ovec3f_get_length(dpos);
	const double rot_var = (double)dang * dang;
	int i;

	for (i = 0; i < ctx->n_devices; i++) {
		rift_tracked_device_priv *dev = ctx->devices + i;

		ohmd_lock_mutex(dev->device_lock);
		if (use_ovr_fusion())
			rift_fusion_ovr_notify_camera_moved(&dev->ovr_fusion);
		else
			rift_kalman_6dof_notify_camera_moved(&dev->ukf_fusion, pos_var, rot_var);
		ohmd_unlock_mutex(dev->device_lock);
	}
}

static void fusion_imu_update(rift_tracked_device_priv *dev, uint64_t time,
	const vec3f *ang_vel, const vec3f *accel, const vec3f *mag, bool accel_saturated)
{
	if (use_ovr_fusion())
		rift_fusion_ovr_imu_update(&dev->ovr_fusion, time, ang_vel, accel, mag, accel_saturated);
	else
		rift_kalman_6dof_imu_update(&dev->ukf_fusion, time, ang_vel, accel, mag, accel_saturated);
}

static void fusion_pose_update(rift_tracked_device_priv *dev, uint64_t time,
	posef *pose, int delay_slot, float obs_scale, bool replace_pending)
{
	if (use_ovr_fusion())
		rift_fusion_ovr_pose_update(&dev->ovr_fusion, time, pose, delay_slot, obs_scale, replace_pending);
	else
		rift_kalman_6dof_pose_update(&dev->ukf_fusion, time, pose, delay_slot, obs_scale);
}

static void fusion_position_update(rift_tracked_device_priv *dev, uint64_t time,
	vec3f *position, int delay_slot, float obs_scale, bool replace_pending)
{
	if (use_ovr_fusion())
		rift_fusion_ovr_position_update(&dev->ovr_fusion, time, position, delay_slot, obs_scale, replace_pending);
	else
		rift_kalman_6dof_position_update(&dev->ukf_fusion, time, position, delay_slot, obs_scale);
}

static void fusion_prepare_delay_slot(rift_tracked_device_priv *dev, uint64_t time, int delay_slot)
{
	if (use_ovr_fusion())
		rift_fusion_ovr_prepare_delay_slot(&dev->ovr_fusion, time, delay_slot);
	else
		rift_kalman_6dof_prepare_delay_slot(&dev->ukf_fusion, time, delay_slot);
}

static void fusion_get_pose_at(rift_tracked_device_priv *dev, uint64_t time, posef *pose,
	vec3f *vel, vec3f *accel, vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error)
{
	if (use_ovr_fusion())
		rift_fusion_ovr_get_pose_at(&dev->ovr_fusion, time, pose, vel, accel, ang_vel, pos_error, rot_error);
	else
		rift_kalman_6dof_get_pose_at(&dev->ukf_fusion, time, pose, vel, accel, ang_vel, pos_error, rot_error);
}

static void fusion_get_delay_slot_pose_at(rift_tracked_device_priv *dev, uint64_t time, int delay_slot,
	posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error)
{
	if (use_ovr_fusion())
		rift_fusion_ovr_get_delay_slot_pose_at(&dev->ovr_fusion, time, delay_slot, pose, vel, accel, ang_vel, pos_error, rot_error);
	else
		rift_kalman_6dof_get_delay_slot_pose_at(&dev->ukf_fusion, time, delay_slot, pose, vel, accel, ang_vel, pos_error, rot_error);
}

/* Set OHMD_RIFT_NO_OBS_MERGE=1 to disable merging of same-exposure
 * observations from multiple sensors (for A/B testing). Without merging,
 * each sensor's fix is applied sequentially and the fused pose lands near
 * whichever sensor's correction arrived last; the arrival order races and
 * flips quasi-periodically, oscillating the output between the two cameras'
 * solutions (measured: 7.5 Hz line, amplitude = cross-camera disagreement —
 * see rift-cv1-center/rest-wander-analysis.md). */
static bool obs_merge_enabled(void)
{
	static int enabled = -1;
	if (enabled == -1) {
		const char *e = getenv("OHMD_RIFT_NO_OBS_MERGE");
		enabled = !(e && e[0] == '1');
	}
	return enabled;
}

/* Joint multi-camera reconstruction: solve ONE pose from every sensor's LED
 * correspondences for an exposure, instead of averaging the per-sensor poses.
 * Each per-camera solution is already optimal in its own image and wrong in the
 * other's, so their average is wrong in both - measured offline on recorded
 * captures, the per-camera solutions fit their own camera to 0.09 px but
 * reproject into the other at 5.6 px, and the merged pose sits at 2.8 px. The
 * joint solve reaches 0.66 px in the worst camera and cuts frame-to-frame
 * jitter 11x (rift-cv1-center/windows-vs-linux-tracking.md section 2).
 * OHMD_RIFT_NO_JOINT_SOLVE=1 falls back to the weighted merge for A/B. */
static bool joint_solve_enabled(void)
{
	static int enabled = -1;
	if (enabled == -1) {
		const char *e = getenv("OHMD_RIFT_NO_JOINT_SOLVE");
		enabled = !(e && e[0] == '1');
	}
	return enabled;
}

/* Oculus accepts a reconstruction at 2 px reprojection (their 2/715
 * normalised, Rift.dll fcn.18017f370). Beyond that no single pose explains
 * every camera's image, which means the extrinsics are wrong rather than the
 * pose - measured sensitivity ~4.3 px per degree of camera rotation error. */
#define JOINT_ACCEPT_PX 2.0f

static bool extrinsic_refine_enabled(void)
{
	static int enabled = -1;
	if (enabled == -1) {
		const char *e = getenv("OHMD_RIFT_NO_EXTRINSIC_REFINE");
		enabled = !(e && e[0] == '1');
	}
	return enabled;
}

/* Calibration settle state, mirroring the runtime's own
 * "Camera Calibration Settled/Unsettled" and its is_sensor_settled /
 * are_sensors_settled telemetry. A sensor is unsettled while refinement is
 * still moving it, and settles once this many consecutive evaluations produce
 * a step small enough to be inside the deadband. */
#define EXTRINSIC_SETTLE_QUIET_ROUNDS 3

#define EXTRINSIC_REFINE_MIN_MEAS 100
#define EXTRINSIC_REFINE_INTERVAL_NS 5000000000ULL
#define EXTRINSIC_REFINE_MIN_SPAN_M 0.10f
#define EXTRINSIC_REFINE_GAIN 0.15f
#define EXTRINSIC_REFINE_MAX_POS_STEP 0.03f
#define EXTRINSIC_REFINE_MAX_ANG_STEP DEG_TO_RAD(2.0f)
/* dead-band: residual PnP-bias mismatch is < ~8 mm on this hardware while a
 * real bumped sensor measures 10-100x that — don't chase the noise floor */
#define EXTRINSIC_REFINE_DEADBAND_POS 0.008f
#define EXTRINSIC_REFINE_DEADBAND_ANG DEG_TO_RAD(0.3f)
/* |mean forward|/n <= cos(45deg/2): the window must span >= ~45 deg of gaze
 * so view-dependent bias averages out instead of being chased */
#define EXTRINSIC_REFINE_MAX_FWD_COHERENCE 0.92f

/* Called with the device lock held, right after a new used pose report was
 * stored for this exposure. If the exposure now carries strong LED-verified
 * HMD observations from the anchor sensor (sensors[0]) AND another sensor,
 * record their relative mismatch into that sensor's refinement window. */
static void extrinsic_refine_measure(rift_tracked_device_priv *dev,
	rift_tracker_pose_delay_slot *slot, rift_tracker_pose_report *newr)
{
	rift_tracker_ctx *ctx = dev->tracker;
	int i;

	if (!extrinsic_refine_enabled() || ctx == NULL || ctx->n_sensors < 2)
		return;
	if (dev->base.id != 0)
		return; /* only the HMD constellation is rich enough to trust */
	if (!POSE_HAS_FLAGS(&newr->score, RIFT_POSE_MATCH_STRONG | RIFT_POSE_MATCH_LED_IDS) ||
	    newr->score.matched_blobs < 10 || newr->source == NULL)
		return;

	const char *anchor_serial = rift_sensor_serial_no(ctx->sensors[0]);

	for (i = 0; i < slot->n_pose_reports; i++) {
		rift_tracker_pose_report *other = slot->pose_reports + i;
		rift_tracker_pose_report *anchor, *target;
		int tidx = -1, s, k;

		if (other == newr || !other->report_used || other->source == NULL)
			continue;
		if (strcmp(other->source, newr->source) == 0)
			continue;
		if (!POSE_HAS_FLAGS(&other->score, RIFT_POSE_MATCH_STRONG | RIFT_POSE_MATCH_LED_IDS) ||
		    other->score.matched_blobs < 10)
			continue;

		/* exactly one of the pair must be the anchor sensor */
		if (strcmp(newr->source, anchor_serial) == 0) {
			anchor = newr;
			target = other;
		} else if (strcmp(other->source, anchor_serial) == 0) {
			anchor = other;
			target = newr;
		} else
			continue;

		for (s = 1; s < ctx->n_sensors; s++) {
			if (strcmp(rift_sensor_serial_no(ctx->sensors[s]), target->source) == 0) {
				tidx = s;
				break;
			}
		}
		if (tidx < 0)
			continue;

		/* M = anchor_pose o inv(target_pose): the world-pose premultiplier
		 * that would bring the target sensor into agreement */
		posef M, inv = target->pose;
		oposef_inverse(&inv);
		oposef_apply(&inv, &anchor->pose, &M);

		/* floor-plane gaze direction (LED-model +Z), for view diversity */
		vec3f fwd_axis = {{ 0.0f, 0.0f, 1.0f }}, fwd;
		oquatf_get_rotated(&anchor->pose.orient, &fwd_axis, &fwd);
		fwd.y = 0.0f;
		float fn = ovec3f_get_length(&fwd);
		if (fn > 0.3f)
			ovec3f_multiply_scalar(&fwd, 1.0f / fn, &fwd);
		else
			ovec3f_set(&fwd, 0, 0, 0); /* looking up/down: no yaw info */

		ohmd_lock_mutex(ctx->refine_lock);
		rift_extrinsic_refine *r = ctx->refine + tidx;
		if (r->n_meas == 0) {
			r->mean_dpos = M.pos;
			r->mean_dorient = M.orient;
			r->span_min = r->span_max = anchor->pose.pos;
			r->fwd_sum = fwd;
			r->n_meas = 1;
		} else {
			vec3f delta;
			r->n_meas++;
			ovec3f_subtract(&M.pos, &r->mean_dpos, &delta);
			ovec3f_multiply_scalar(&delta, 1.0f / r->n_meas, &delta);
			ovec3f_add(&r->mean_dpos, &delta, &r->mean_dpos);
			oquatf_slerp(1.0f / r->n_meas, &r->mean_dorient, &M.orient, true, &r->mean_dorient);
			oquatf_normalize_me(&r->mean_dorient);
			ovec3f_add(&r->fwd_sum, &fwd, &r->fwd_sum);
			for (k = 0; k < 3; k++) {
				if (anchor->pose.pos.arr[k] < r->span_min.arr[k])
					r->span_min.arr[k] = anchor->pose.pos.arr[k];
				if (anchor->pose.pos.arr[k] > r->span_max.arr[k])
					r->span_max.arr[k] = anchor->pose.pos.arr[k];
			}
		}
		ohmd_unlock_mutex(ctx->refine_lock);
		break;
	}
}

/* Optical corrections are applied to the displayed pose directly. They used to
 * be bled in over OUT_CORR_TAU instead, to hide the step each one made, and
 * that is now OFF by default -- set OHMD_RIFT_BLEED=1 to restore it.
 *
 * Bleeding was worth it when corrections were large and noisy. It is not any
 * more: with the joint reconstruction and automatic calibration in place they
 * are small and trustworthy, so smearing one over ~1 s only delays a correct
 * measurement, and does so *while the head is moving* since the rate scales
 * with velocity. Reported in the headset as movement that "translates weirdly"
 * while never vibrating - which is exactly the shape of the trade.
 *
 * Measured, stationary, on the output pose: turning it off costs 0.023 -> 0.029
 * mm of sample-to-sample step (max 0.17 -> 0.35 mm) and 0.184 -> 0.236 mm rms
 * shake. All far below anything visible, against up to OUT_CORR_MAX_LIN = 5 cm
 * of positional lag while moving. */
static bool out_corr_enabled(void)
{
	static int enabled = -1;
	if (enabled == -1) {
		const char *e = getenv("OHMD_RIFT_BLEED");
		enabled = (e && e[0] == '1');
		LOGI("output correction bleeding %s%s",
			enabled ? "ON" : "OFF",
			enabled ? " (OHMD_RIFT_BLEED=1)" : " - set OHMD_RIFT_BLEED=1 to restore");
	}
	return enabled;
}

static void rift_tracked_device_send_imu_debug(rift_tracked_device_priv *dev);
static void rift_tracked_device_send_debug_printf(rift_tracked_device_priv *dev, uint64_t local_ts, const char *fmt, ...);

static void rift_tracked_device_on_new_exposure (rift_tracked_device_priv *dev, uint64_t exposure_local_ts, rift_tracked_device_exposure_info *dev_info);
static int rift_tracked_device_exposure_claim(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info);
static void rift_tracked_device_exposure_release_locked(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info);

rift_tracked_device *
rift_tracker_add_device (rift_tracker_ctx *ctx, int device_id, posef *imu_pose, posef *model_pose, rift_leds *leds,
      rift_tracked_device_imu_calibration *imu_calib)
{
	int i, s;
	rift_tracked_device_priv *next_dev;
	char device_name[64];
	/* Rotate our initial pose 180 deg to point along the -Z axis */
	posef init_pose = { .pos = {{ 0.0, 0.0, 0.0 }}, .orient = {{ 0.0, 1.0, 0.0, 0.0 }}};

	snprintf(device_name,64,"openhmd-rift-device-%d", device_id);
	device_name[63] = 0;

	assert (ctx->n_devices < RIFT_MAX_TRACKED_DEVICES);

	ohmd_lock_mutex (ctx->tracker_lock);
	next_dev = ctx->devices + ctx->n_devices;

	next_dev->base.id = device_id;
	next_dev->tracker = ctx;
	next_dev->n_delay_slots = ctx->n_sensors != 0 ? NUM_POSE_DELAY_SLOTS : 0;
	rift_kalman_6dof_init(&next_dev->ukf_fusion, &init_pose, next_dev->n_delay_slots);
	rift_fusion_ovr_init(&next_dev->ovr_fusion, &init_pose, next_dev->n_delay_slots);
	LOGI("Device %d using %s fusion backend", device_id,
		use_ovr_fusion() ? "OVR-SDK complementary" : "UKF");
	next_dev->last_acquired_pose_lock_ts = next_dev->last_reported_pose = next_dev->last_observed_orient_ts = next_dev->last_observed_pose_ts = next_dev->device_time_ns = 0;

	exp_filter_pose_init(&next_dev->pose_output_filter);

	oquatf_set(&next_dev->out_corr_orient, 0, 0, 0, 1);
	ovec3f_set(&next_dev->out_corr_pos, 0, 0, 0);

	/* Init delay slot bookkeeping */
	for (s = 0; s < next_dev->n_delay_slots; s++) {
		rift_tracker_pose_delay_slot *slot = next_dev->delay_slots + s;

		slot->slot_id = s;
		slot->valid = false;
	}

	/* Compute the device->IMU conversion from the imu->device pose passed */
	next_dev->device_from_fusion = *imu_pose;
	oposef_inverse(&next_dev->device_from_fusion);

	/* Compute the IMU->model transform by composing imu->device->model */
	oposef_apply(imu_pose, model_pose, &next_dev->fusion_from_model);
	/* And the inverse fusion->model conversion */
	next_dev->model_from_fusion = next_dev->fusion_from_model;
	oposef_inverse(&next_dev->model_from_fusion);

	next_dev->debug_metadata = ohmd_pw_debug_stream_new (device_name, "Rift Device");

	uint64_t now = ohmd_monotonic_get(ctx->ohmd_ctx);
	rift_tracked_device_send_debug_printf (next_dev, now, "{ \"type\": \"device\", "
		 "\"device-id\": %d,"
		"\"imu-calibration\": { \"accel-offset\": [ %f, %f, %f ], "
		"\"accel-matrix\": [ %f, %f, %f, %f, %f, %f, %f, %f, %f ], "
		"\"gyro_offset\": [ %f, %f, %f ], "
		"\"gyro-matrix\": [ %f, %f, %f, %f, %f, %f, %f, %f, %f ] } },", device_id,
		imu_calib->accel_offset.x, imu_calib->accel_offset.y, imu_calib->accel_offset.z,
		imu_calib->accel_matrix[0], imu_calib->accel_matrix[1], imu_calib->accel_matrix[2],
		imu_calib->accel_matrix[3], imu_calib->accel_matrix[4], imu_calib->accel_matrix[5],
		imu_calib->accel_matrix[6], imu_calib->accel_matrix[7], imu_calib->accel_matrix[8],
		imu_calib->gyro_offset.x, imu_calib->gyro_offset.y, imu_calib->gyro_offset.z,
		imu_calib->gyro_matrix[0], imu_calib->gyro_matrix[1], imu_calib->gyro_matrix[2],
		imu_calib->gyro_matrix[3], imu_calib->gyro_matrix[4], imu_calib->gyro_matrix[5],
		imu_calib->gyro_matrix[6], imu_calib->gyro_matrix[7], imu_calib->gyro_matrix[8]);

	next_dev->base.leds = leds;
	next_dev->base.led_search = led_search_model_new (leds);

	/* Unpack the LED model into a plain vec3f array for the joint solver */
	next_dev->led_pos = calloc(leds->num_points, sizeof(vec3f));
	if (next_dev->led_pos != NULL) {
		for (i = 0; i < leds->num_points; i++)
			next_dev->led_pos[i] = leds->points[i].pos;
		next_dev->n_led_pos = leds->num_points;
	}
	rift_cal_capture_register_leds (device_id, leds);
	ctx->n_devices++;
	ohmd_unlock_mutex (ctx->tracker_lock);

	/* Tell the sensors about the new device */
	for (i = 0; i < ctx->n_sensors; i++) {
		rift_sensor_ctx *sensor_ctx = ctx->sensors[i];
		if (!rift_sensor_add_device (sensor_ctx, (rift_tracked_device *) next_dev)) {
			LOGE("Failed to configure object tracking for device %d\n", device_id);
		}
	}

	printf("device %d online. Now tracking.\n", device_id);
	return (rift_tracked_device *) next_dev;
}

static unsigned int uvc_handle_events(void *arg)
{
	rift_tracker_ctx *tracker_ctx = arg;

	while (!tracker_ctx->usb_completed) {
		struct timeval timeout;

		timeout.tv_sec = 0;
		timeout.tv_usec = 100000;

		libusb_handle_events_timeout_completed(tracker_ctx->usb_ctx, &timeout, &tracker_ctx->usb_completed);
	}

	return 0;
}

rift_tracker_ctx *
rift_tracker_new (ohmd_context* ohmd_ctx,
		const uint8_t radio_id[5])
{
	rift_tracker_ctx *tracker_ctx = NULL;
	int ret, i;
	libusb_device **devs;
	posef room_pose_offset;

	tracker_ctx = ohmd_alloc(ohmd_ctx, sizeof (rift_tracker_ctx));
	tracker_ctx->ohmd_ctx = ohmd_ctx;
	tracker_ctx->tracker_lock = ohmd_create_mutex(ohmd_ctx);
	tracker_ctx->refine_lock = ohmd_create_mutex(ohmd_ctx);
	tracker_ctx->calib_lock = ohmd_create_mutex(ohmd_ctx);
	for (i = 0; i < RIFT_MAX_SENSORS; i++)
		rift_cam_calib_init(tracker_ctx->cam_calib + i);

	rift_tracker_config_init(&tracker_ctx->config);
	rift_tracker_config_load(ohmd_ctx, &tracker_ctx->config);
	rift_tracker_config_get_room_pose_offset(&tracker_ctx->config, &room_pose_offset);

	for (i = 0; i < RIFT_MAX_TRACKED_DEVICES; i++) {
		rift_tracked_device_priv *dev = tracker_ctx->devices + i;
		dev->index = i;
		dev->device_lock = ohmd_create_mutex(ohmd_ctx);
	}

	ret = libusb_init(&tracker_ctx->usb_ctx);
	ASSERT_MSG(ret >= 0, fail, "could not initialize libusb\n");

	ret = libusb_get_device_list(tracker_ctx->usb_ctx, &devs);
	ASSERT_MSG(ret >= 0, fail, "Could not get USB device list\n");

	/* Start USB event thread */
	tracker_ctx->usb_completed = false;
	tracker_ctx->usb_thread = ohmd_create_thread (ohmd_ctx, uvc_handle_events, tracker_ctx);

	for (i = 0; devs[i]; ++i) {
		struct libusb_device_descriptor desc;
		libusb_device_handle *usb_devh;
		unsigned char serial[RIFT_SENSOR_SERIAL_LEN+1];

		ret = libusb_get_device_descriptor(devs[i], &desc);
		if (ret < 0)
			continue; /* Can't access this device */
		if (desc.idVendor != 0x2833 || (desc.idProduct != CV1_PID && desc.idProduct != DK2_PID))
			continue;

		ret = libusb_open(devs[i], &usb_devh);
		if (ret) {
			fprintf (stderr, "Failed to open Rift Sensor device. Check permissions\n");
			continue;
		}

		sprintf ((char *) serial, "UNKNOWN");
		serial[RIFT_SENSOR_SERIAL_LEN] = '\0';

		if (desc.iSerialNumber) {
			ret = libusb_get_string_descriptor_ascii(usb_devh, desc.iSerialNumber, serial, 32);
			if (ret < 0)
				fprintf (stderr, "Failed to read the Rift Sensor Serial number.\n");
		}

		rift_sensor_device *sensor_device = rift_sensor_usb_new (ohmd_ctx, tracker_ctx->n_sensors, (char *) serial, tracker_ctx->usb_ctx, usb_devh, radio_id);

		if (sensor_device == NULL) {
			LOGW("Failed to open Rift Sensor USB device %s\n", serial);
			continue;
		}

		rift_sensor_ctx *sensor_ctx = rift_sensor_new (ohmd_ctx, tracker_ctx->n_sensors, (char *) serial, sensor_device, tracker_ctx);
		if (sensor_ctx == NULL) {
			LOGW("Failed to open Rift Sensor analyser for %s\n", serial);
			continue;
		}

		posef camera_pose;
		if (rift_tracker_config_get_sensor_pose(&tracker_ctx->config, (char *) serial, &camera_pose)) {
			LOGI("Loaded pose for sensor %s from room store: "
			    "\"pos\" : [ %f, %f, %f ], \"orient\" : [ %f, %f, %f, %f ]",
			    serial,
			    camera_pose.pos.x, camera_pose.pos.y,
			    camera_pose.pos.z, camera_pose.orient.x,
			    camera_pose.orient.y, camera_pose.orient.z,
			    camera_pose.orient.w);

			/* Add the room offset to the camera pose we give the sensor */
			oposef_apply(&camera_pose, &room_pose_offset, &camera_pose);
			rift_sensor_set_pose(sensor_ctx, &camera_pose);
		}

		tracker_ctx->sensors[tracker_ctx->n_sensors] = sensor_ctx;
		tracker_ctx->n_sensors++;
		if (tracker_ctx->n_sensors == RIFT_MAX_SENSORS) {
			LOGI("Found the maximum number of supported sensors: %d.\n", RIFT_MAX_SENSORS);
			break;
		}
	}
	libusb_free_device_list(devs, 1);

	printf ("Opened %u Rift Sensor cameras\n", tracker_ctx->n_sensors);

	/* Loop over the sensors we found and start the video flowing */
	for (i = 0; i < tracker_ctx->n_sensors; i++) {
		rift_sensor_ctx *sensor = tracker_ctx->sensors[i];

		if (!rift_sensor_start (sensor)) {
			LOGW("Failed to start video stream for sensor %s\n", rift_sensor_serial_no (sensor));
		}
	}

	return tracker_ctx;

fail:
	if (tracker_ctx)
		rift_tracker_free (tracker_ctx);
	return NULL;
}

/* Called from the rift IMU / packet handling loop
 * when processing an IMU update from the HMD. If the
 * packet signalled a new camera exposure, we take
 * a snapshot of the predicted state of each device
 * into a lagged fusion slot */
void rift_tracker_on_new_exposure (rift_tracker_ctx *ctx, uint32_t hmd_ts, uint16_t exposure_count, uint32_t exposure_hmd_ts, uint8_t led_pattern_phase)
{
	rift_tracker_exposure_info *info;
	bool is_new_exposure = false;
	int i;

	ohmd_lock_mutex (ctx->tracker_lock);

	if (ctx->exposure_history_size > 0) {
		if (ctx->exposure_history[ctx->exposure_history_index].count != exposure_count) {
			is_new_exposure = true;

			ctx->exposure_history_index = (ctx->exposure_history_index + 1) % NUM_EXPOSURE_HISTORY;
			info = ctx->exposure_history + ctx->exposure_history_index;

			if (ctx->exposure_history_size < NUM_EXPOSURE_HISTORY)
				ctx->exposure_history_size++;
		}
	}
	else {
		info = ctx->exposure_history;
		ctx->exposure_history_index = 0;
		ctx->exposure_history_size = 1;
		is_new_exposure = true;
	}

	if (!is_new_exposure)
		goto done;

	if (info->led_pattern_phase != led_pattern_phase) {
		LOGD ("%f LED pattern phase changed to %d",
			(double) (ohmd_monotonic_get(ctx->ohmd_ctx)) / 1000000.0, led_pattern_phase);
		info->led_pattern_phase = led_pattern_phase;
	}

	uint64_t now = ohmd_monotonic_get(ctx->ohmd_ctx);

	/* Timing health: the HMD re-announcing an exposure, or a gap where one
	 * should have been, both mean the vision timebase is stuttering. */
	{
		int64_t delta_ns = 0;
		if (rift_sync_monitor_exposure(&ctx->sync, exposure_count, now, &delta_ns)) {
			if ((ctx->sync.repeated_exposures % 100) == 1)
				LOGW("Repeated exposure time: count %u seen again (%u so far)",
					exposure_count, ctx->sync.repeated_exposures);
		} else if (rift_sync_monitor_exposure_gap(delta_ns)) {
			if ((ctx->sync.dropped_exposures % 100) == 1)
				LOGW("Exposure stream out of sync: gap of %.1f ms (nominal %.1f ms), "
					"%u so far", delta_ns / 1000000.0,
					RIFT_SYNC_NOMINAL_EXPOSURE_NS / 1000000.0,
					ctx->sync.dropped_exposures);
		}
	}

	info->local_ts = now;
	info->count = exposure_count;
	info->hmd_ts = exposure_hmd_ts;
	info->led_pattern_phase = led_pattern_phase;

	LOGD ("%f Have new exposure TS %u count %u LED pattern phase %d",
		(double) (now) / 1000000.0, exposure_count, exposure_hmd_ts, led_pattern_phase);

	if ((int32_t)(exposure_hmd_ts - hmd_ts) < -1500) {
		LOGW("Exposure timestamp %u was more than 1.5 IMU samples earlier than IMU ts %u by %u µS",
				exposure_hmd_ts, hmd_ts, hmd_ts - exposure_hmd_ts);
	}

	info->n_devices = ctx->n_devices;

	for (i = 0; i < ctx->n_devices; i++) {
		rift_tracked_device_priv *dev = ctx->devices + i;
		rift_tracked_device_exposure_info *dev_info = info->devices + i;

		dev_info->device_index = dev->index;

		ohmd_lock_mutex (dev->device_lock);
		rift_tracked_device_on_new_exposure(dev, now, dev_info);

		rift_tracked_device_send_imu_debug(dev);

		rift_tracked_device_send_debug_printf(dev, now,
				",\n{ \"type\": \"exposure\", \"local-ts\": %llu, "
				"\"hmd-ts\": %u, \"exposure-ts\": %u, \"count\": %u, \"device-ts\": %llu, "
				"\"delay-slot\": %d	}",
				(unsigned long long) now,
				hmd_ts, exposure_hmd_ts, exposure_count,
				(unsigned long long) dev_info->device_time_ns, dev_info->fusion_slot);
		ohmd_unlock_mutex (dev->device_lock);
	}
	/* Clear the info for non-existent devices */
	for (; i < RIFT_MAX_TRACKED_DEVICES; i++) {
		rift_tracked_device_exposure_info *dev_info = info->devices + i;
		dev_info->fusion_slot = -1;
	}

done:
	ohmd_unlock_mutex (ctx->tracker_lock);
}

/* Called from a sensor device when a video frame has been captured.
 * Iterate the exposure history and find the exposure that matches
 * the arrival time of the frame within +/- 10ms
 */
bool
rift_tracker_frame_captured (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *out_info, const char *source)
{
	int i;
	ohmd_lock_mutex (ctx->tracker_lock);

	/* Find and populate the exposure info and return true if found.
	 * Exposures are only ~19.2ms apart, so with a +/-10ms acceptance
	 * window adjacent exposures overlap - always pick the CLOSEST one,
	 * not the last one that falls inside the window. */
	bool have_exposure_info = false;
	int64_t best_abs_ns = INT64_MAX;
	int64_t best_diff_ns = 0;
	int64_t matched_latency_ns = 0;

	for (i = 0; i < ctx->exposure_history_size; i++) {
		rift_tracker_exposure_info *info = ctx->exposure_history + i;
		int64_t time_diff_ns = frame_start_local_ts - info->local_ts;
		int64_t abs_ns = time_diff_ns < 0 ? -time_diff_ns : time_diff_ns;
		if (abs_ns < best_abs_ns) {
			best_abs_ns = abs_ns;
			best_diff_ns = time_diff_ns;
			if (abs_ns < 10000000) {
				have_exposure_info = true;
				matched_latency_ns = time_diff_ns;
				*out_info = *info;
			}
		}
	}

	/* Timing health: how long after its exposure this frame turned up. A
	 * frame that lands far from what the stream has been doing means the
	 * camera and the HMD have drifted apart - the runtime blames the sync
	 * cable in so many words. */
	if (have_exposure_info) {
		int64_t predicted_ns = 0;
		if (rift_sync_monitor_frame(&ctx->sync, matched_latency_ns, &predicted_ns) &&
		    (ctx->sync.latency_outliers % 100) == 1) {
			LOGW("Sensor %s: predicted exposure-to-frame latency of %.1f ms differed "
				"from measured %.1f ms (%u so far) - check the sensor sync cable",
				source, predicted_ns / 1000000.0, matched_latency_ns / 1000000.0,
				ctx->sync.latency_outliers);
		}
	}

	/* Temporary telemetry: per-sensor frame-vs-exposure timing statistics */
	{
		#define FC_MAX_SOURCES 4
		static struct {
			const char *src;
			int n, missed;
			double sum_ms, min_ms, max_ms;
		} fc_stats[FC_MAX_SOURCES];
		int s;
		double diff_ms = (double)best_diff_ns / 1e6;
		for (s = 0; s < FC_MAX_SOURCES; s++) {
			if (fc_stats[s].src == source)
				break;
			if (fc_stats[s].src == NULL) {
				fc_stats[s].src = source;
				fc_stats[s].min_ms = 1e9;
				fc_stats[s].max_ms = -1e9;
				break;
			}
		}
		if (s < FC_MAX_SOURCES) {
			fc_stats[s].n++;
			if (!have_exposure_info)
				fc_stats[s].missed++;
			fc_stats[s].sum_ms += diff_ms;
			if (diff_ms < fc_stats[s].min_ms) fc_stats[s].min_ms = diff_ms;
			if (diff_ms > fc_stats[s].max_ms) fc_stats[s].max_ms = diff_ms;
			if (fc_stats[s].n >= 300) {
				LOGI("frame-timing %s: %d frames, %d missed exposure, diff avg %.2f min %.2f max %.2f ms",
				    source, fc_stats[s].n, fc_stats[s].missed,
				    fc_stats[s].sum_ms / fc_stats[s].n, fc_stats[s].min_ms, fc_stats[s].max_ms);
				fc_stats[s].n = fc_stats[s].missed = 0;
				fc_stats[s].sum_ms = 0;
				fc_stats[s].min_ms = 1e9;
				fc_stats[s].max_ms = -1e9;
			}
		}
	}

	if (!have_exposure_info)
		goto done;

	for (i = 0; i < ctx->n_devices; i++) {
		rift_tracked_device_priv *dev = ctx->devices + i;

		ohmd_lock_mutex (dev->device_lock);

		if (i < out_info->n_devices) {
			rift_tracked_device_exposure_info *dev_info = out_info->devices + i;

#if LOGLEVEL == 0
			LOGD("Frame capture - ts %llu, delay slot %d for dev %d",
				(unsigned long long) dev_info->device_time_ns, dev_info->fusion_slot, dev->base.id);
#endif
			dev_info->fusion_slot = rift_tracked_device_exposure_claim(dev, dev_info);
		}

		rift_tracked_device_send_imu_debug(dev);

		if (dev->debug_file != NULL) {
			fprintf(dev->debug_file, ",\n{ \"type\": \"frame-captured\", \"local-ts\": %llu, "
				"\"frame-start-local-ts\": %llu, \"source\": \"%s\" }",
				(unsigned long long) local_ts, (unsigned long long) frame_start_local_ts, source);
		}
		ohmd_unlock_mutex (dev->device_lock);
	}

done:
	ohmd_unlock_mutex (ctx->tracker_lock);

	return have_exposure_info;
}

void
rift_tracker_frame_release (rift_tracker_ctx *ctx, uint64_t local_ts, uint64_t frame_local_ts, rift_tracker_exposure_info *info, const char *source)
{
	int i;
	ohmd_lock_mutex (ctx->tracker_lock);
	for (i = 0; i < ctx->n_devices; i++) {
		rift_tracked_device_priv *dev = ctx->devices + i;

		ohmd_lock_mutex (dev->device_lock);

		/* This device might not have exposure info for this frame if it
		 * recently came online */
		if (info && i < info->n_devices) {
			rift_tracked_device_exposure_info *dev_info = info->devices + i;
			rift_tracked_device_exposure_release_locked(dev, dev_info);
		}

		rift_tracked_device_send_imu_debug(dev);

		if (dev->debug_file != NULL) {
			fprintf(dev->debug_file, ",\n{ \"type\": \"frame-release\", \"local-ts\": %llu, "
				"\"frame-local-ts\": %llu, \"source\": \"%s\" }",
				(unsigned long long) local_ts, (unsigned long long) frame_local_ts, source);
		}
		ohmd_unlock_mutex (dev->device_lock);
	}
	ohmd_unlock_mutex (ctx->tracker_lock);
}

void
rift_tracker_free (rift_tracker_ctx *tracker_ctx)
{
	int i;

	if (!tracker_ctx)
		return;

	for (i = 0; i < tracker_ctx->n_sensors; i++) {
		rift_sensor_ctx *sensor_ctx = tracker_ctx->sensors[i];
		rift_sensor_free (sensor_ctx);
	}

	for (i = 0; i < RIFT_MAX_TRACKED_DEVICES; i++) {
		rift_tracked_device_priv *dev = tracker_ctx->devices + i;
		if (dev->base.led_search)
			led_search_model_free (dev->base.led_search);
		free (dev->led_pos);
		dev->led_pos = NULL;
		if (dev->debug_metadata != NULL)
			ohmd_pw_debug_stream_free (dev->debug_metadata);

		rift_kalman_6dof_clear(&dev->ukf_fusion);
		ohmd_destroy_mutex (dev->device_lock);
	}

	/* Stop USB event thread */
	tracker_ctx->usb_completed = true;
	ohmd_destroy_thread (tracker_ctx->usb_thread);

	if (tracker_ctx->usb_ctx)
		libusb_exit (tracker_ctx->usb_ctx);

	ohmd_destroy_mutex (tracker_ctx->tracker_lock);
	ohmd_destroy_mutex (tracker_ctx->refine_lock);
	ohmd_destroy_mutex (tracker_ctx->calib_lock);
	free (tracker_ctx);
}

void rift_tracked_device_imu_update(rift_tracked_device *dev_base, uint64_t local_ts, uint32_t device_ts, float dt, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field, rift_imu_sample_flags flags)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	rift_tracked_device_imu_observation *obs;

	ohmd_lock_mutex (dev->device_lock);

	/* Handle device_ts wrap by extending to 64-bit and working in nanoseconds */
	if (dev->device_time_ns == 0) {
		dev->device_time_ns = device_ts * 1000;
	} else {
		uint64_t dt_ns = ((uint32_t)(device_ts - dev->last_device_ts)) * 1000;
		dev->device_time_ns += dt_ns;
	}
	dev->last_device_ts = device_ts;
	dev->last_imu_local_ts = local_ts;

	if (flags != RIFT_IMU_SAMPLE_OK) {
		dev->imu_saturated_samples++;
		if ((dev->imu_saturated_samples % 500) == 1) {
			LOGI("Device %d: IMU saturation (%s%s), %u samples so far - the reading is "
				"clipped, so it is being ignored rather than integrated",
				dev->base.id,
				(flags & RIFT_IMU_ACCEL_SATURATED) ? "accel" : "",
				(flags & RIFT_IMU_GYRO_SATURATED) ? " gyro" : "",
				dev->imu_saturated_samples);
		}
	}

	fusion_imu_update(dev, dev->device_time_ns, ang_vel, accel, mag_field,
		(flags & RIFT_IMU_ACCEL_SATURATED) != 0);

	obs = dev->pending_imu_observations + dev->num_pending_imu_observations;
	obs->local_ts = local_ts;
	obs->device_ts = dev->device_time_ns;
	obs->dt = dt;
	obs->ang_vel = *ang_vel;
	obs->accel = *accel;
	obs->mag = *mag_field;

	dev->num_pending_imu_observations++;

	if (dev->num_pending_imu_observations == RIFT_MAX_PENDING_IMU_OBSERVATIONS) {
		/* No camera observations for a while - send our observations from here instead */
		rift_tracked_device_send_imu_debug(dev);
	}

	ohmd_unlock_mutex (dev->device_lock);
}

/* Forward-prediction horizon for the pose handed to API consumers, in seconds.
 * OHMD_RIFT_PREDICT_MS=<ms>, default 0 (off). See the note in
 * rift_tracked_device_get_view_pose about double-prediction under SteamVR. */
static float out_predict_secs(void)
{
	static float predict_s = -1.0f;

	if (predict_s < 0.0f) {
		const char *e = getenv("OHMD_RIFT_PREDICT_MS");
		float ms = e ? (float) atof(e) : 0.0f;

		if (ms < 0.0f)
			ms = 0.0f;
		if (ms > 100.0f)  /* sanity clamp; beyond this the linear model is junk */
			ms = 100.0f;
		predict_s = ms / 1000.0f;
	}

	return predict_s;
}

/* Dead-reckon a pose forward by dt.
 *
 * Orientation integrates the DEVICE-LOCAL angular velocity, so the delta
 * quaternion right-multiplies (body-frame rotation) — the same form Oculus
 * used in SDK 0.3.2's SensorFusion::GetPredictedOrientation. Position uses the
 * world-frame velocity and acceleration, which is the frame the fusion already
 * reports them in:  p += v*dt + a*dt^2/2,  v += a*dt. */
static void rift_predict_pose(posef *pose, vec3f *vel, const vec3f *accel,
	const vec3f *ang_vel, float dt)
{
	float w = ovec3f_get_length((vec3f *) ang_vel);

	if (w > 1e-6f) {
		vec3f axis;
		quatf dq, out;
		float half = 0.5f * w * dt;
		float s = sinf(half);

		ovec3f_multiply_scalar((vec3f *) ang_vel, 1.0f / w, &axis);
		dq.x = axis.x * s;
		dq.y = axis.y * s;
		dq.z = axis.z * s;
		dq.w = cosf(half);

		oquatf_mult(&pose->orient, &dq, &out);
		oquatf_normalize_me(&out);
		pose->orient = out;
	}

	vec3f tmp;
	ovec3f_multiply_scalar(vel, dt, &tmp);
	ovec3f_add(&pose->pos, &tmp, &pose->pos);
	ovec3f_multiply_scalar((vec3f *) accel, 0.5f * dt * dt, &tmp);
	ovec3f_add(&pose->pos, &tmp, &pose->pos);

	ovec3f_multiply_scalar((vec3f *) accel, dt, &tmp);
	ovec3f_add(vel, &tmp, vel);
}

void rift_tracked_device_get_view_pose(rift_tracked_device *dev_base, posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);

	ohmd_lock_mutex (dev->device_lock);

	if (dev->device_time_ns > dev->last_reported_pose) {
		posef device_pose;
		posef imu_global_pose;
		vec3f imu_ang_vel = { 0, };
		vec3f imu_vel = { 0, }, imu_accel = { 0, };

		fusion_get_pose_at(dev, dev->device_time_ns, &imu_global_pose, &imu_vel, &imu_accel, &imu_ang_vel, NULL, NULL);

		/* Bleed off the output correction offset, faster while the head is
		 * moving, then apply what remains so optical corrections leak into
		 * the displayed pose instead of stepping it at camera rate */
		if (out_corr_enabled()) {
			float dt = 0.1f;
			if (dev->last_reported_pose != 0 && dev->device_time_ns > dev->last_reported_pose)
				dt = (float)(dev->device_time_ns - dev->last_reported_pose) * 1e-9f;
			if (dt > 0.1f)
				dt = 0.1f;

			vec3f corr_rot;
			oquatf_to_rotation(&dev->out_corr_orient, &corr_rot);
			float angle = ovec3f_get_length(&corr_rot);
			if (angle > 1e-6f) {
				float rate = (1.0f / OUT_CORR_TAU) * (1.0f + ovec3f_get_length(&imu_ang_vel) / OUT_CORR_ANG_REF);
				float new_angle = angle * expf(-rate * dt) - OUT_CORR_MIN_ANG_RATE * dt;
				if (new_angle < 0.0f)
					new_angle = 0.0f;
				if (new_angle > OUT_CORR_MAX_ANG)
					new_angle = OUT_CORR_MAX_ANG;
				ovec3f_multiply_scalar(&corr_rot, new_angle / angle, &corr_rot);
				oquatf_from_rotation(&dev->out_corr_orient, &corr_rot);
			} else {
				oquatf_set(&dev->out_corr_orient, 0, 0, 0, 1);
			}

			float dist = ovec3f_get_length(&dev->out_corr_pos);
			if (dist > 1e-6f) {
				float rate = (1.0f / OUT_CORR_TAU) * (1.0f + ovec3f_get_length(&imu_vel) / OUT_CORR_LIN_REF);
				float new_dist = dist * expf(-rate * dt) - OUT_CORR_MIN_LIN_RATE * dt;
				if (new_dist < 0.0f)
					new_dist = 0.0f;
				if (new_dist > OUT_CORR_MAX_LIN)
					new_dist = OUT_CORR_MAX_LIN;
				ovec3f_multiply_scalar(&dev->out_corr_pos, new_dist / dist, &dev->out_corr_pos);
			} else {
				ovec3f_set(&dev->out_corr_pos, 0, 0, 0);
			}

			quatf corrected_orient;
			oquatf_mult(&dev->out_corr_orient, &imu_global_pose.orient, &corrected_orient);
			oquatf_normalize_me(&corrected_orient);
			imu_global_pose.orient = corrected_orient;
			ovec3f_add(&imu_global_pose.pos, &dev->out_corr_pos, &imu_global_pose.pos);
		}

		/* Take our fusion / IMU global pose back to device pose by
		 * computing the IMU->device pose and applying the
		 * IMU->world pose to get device->world pose */
		oposef_apply(&dev->device_from_fusion, &imu_global_pose, &device_pose);

		dev->reported_pose.orient = device_pose.orient;
		if (dev->device_time_ns - dev->last_observed_pose_ts >= (POSE_LOST_THRESHOLD * 1000000UL)) {
			/* Don't let the device move unless there's a recent observation of actual position */
			device_pose.pos = dev->reported_pose.pos;
			imu_vel.x = imu_vel.y = imu_vel.z = 0.0;
			imu_accel.x = imu_accel.y = imu_accel.z = 0.0;
		}

		exp_filter_pose_run(&dev->pose_output_filter, dev->device_time_ns, &device_pose, &dev->reported_pose);
		dev->last_reported_pose = dev->device_time_ns;

		/* SteamVR's pose prediction expects LINEAR velocity in world space
		 * but ANGULAR velocity in the device-local frame ("controller
		 * space") — see Monado's ovrd_driver.cpp (world->local inversion
		 * with that comment) and steamvr_lh/device.cpp (rotating Valve's
		 * own lighthouse driver output local->world when consuming it).
		 *
		 * The gyro rate is in the IMU body frame; device-local is one
		 * static mount rotation away: w_dev = R_dff^-1 * w_imu (identity
		 * for the CV1 HMD, non-trivial for Touch controllers).
		 *
		 * The fusion velocity and acceleration are already world-frame.
		 * Linear velocity also acquires a component from rotation at the
		 * IMU->device lever arm (body frame), rotated into the world.
		 *
		 * NOTE: Valve's driver docs claim world space for vecAngularVelocity,
		 * contradicting the lighthouse driver's observed behavior. Default
		 * device-local; set OHMD_RIFT_ANGVEL_FRAME=world to A/B. */
		static int angvel_world = -1;
		if (angvel_world == -1) {
			const char *e = getenv("OHMD_RIFT_ANGVEL_FRAME");
			angvel_world = (e && strcmp(e, "world") == 0);
			LOGI("angular velocity exported in the %s frame",
			     angvel_world ? "world" : "device-local");
		}
		if (angvel_world) {
			oquatf_get_rotated(&imu_global_pose.orient, &imu_ang_vel, &dev->reported_ang_vel);
		} else {
			quatf fusion_from_device_orient = dev->device_from_fusion.orient;
			oquatf_inverse(&fusion_from_device_orient);
			oquatf_get_rotated(&fusion_from_device_orient, &imu_ang_vel, &dev->reported_ang_vel);
		}
		dev->reported_lin_accel = imu_accel;

		vec3f lever_vel_body, lever_vel_world;
		ovec3f_cross(&imu_ang_vel, &dev->device_from_fusion.pos, &lever_vel_body);
		oquatf_get_rotated(&imu_global_pose.orient, &lever_vel_body, &lever_vel_world);
		ovec3f_add(&imu_vel, &lever_vel_world, &dev->reported_lin_vel);

		/* Low-pass the exported velocities. Consumers (SteamVR's compositor)
		 * multiply them by a ~40 ms photon-prediction horizon EVERY rendered
		 * frame, so raw instantaneous velocity noise becomes visible shimmer:
		 * 0.1 m/s of rest noise = +-4 mm of wobble in the rendered pose. An
		 * EMA with a ~25 ms time constant kills the shimmer for the cost of
		 * ~25 ms of velocity lag under hard acceleration (partially covered
		 * by the separately-exported acceleration term).
		 * OHMD_RIFT_VEL_SMOOTH_MS overrides tau, 0 disables.
		 *
		 * The smoothing is SPEED-ADAPTIVE, because a fixed tau pays that lag
		 * all the time to solve a problem that only exists at rest. The noise
		 * it suppresses is a roughly constant ~0.1 m/s; genuine head motion is
		 * many times that, so once the device is really moving the average is
		 * buying nothing and the lag is pure cost. Reported as "when I move
		 * slowly it's fine, when I move quickly it lags behind" - which is the
		 * signature exactly, since lag from a velocity EMA is proportional to
		 * ACCELERATION and vanishes at constant speed.
		 *
		 * It bites harder in rotation. Linear prediction is
		 * p += v*dt + a*dt^2/2, so the separately-exported acceleration term
		 * partly covers a stale velocity. DriverPose_t does have a
		 * vecAngularAcceleration field, but we export nothing into it, so
		 * orientation is predicted from omega alone and the lag there is
		 * uncompensated. At a 40 ms photon horizon, ramping to 200 deg/s in
		 * 150 ms is ~1.3 deg of view error, appearing only under fast motion
		 * and overshooting on the way out of it.
		 *
		 * Measured on this rig, this is NOT the cause of the reported
		 * "movement feels wrong": disabling the smoothing outright changed
		 * nothing perceptible. Kept because the lag is real and the fix is
		 * free, not because it fixed the complaint.
		 *
		 * So: hold tau at rest, and shorten it in proportion to how fast the
		 * device is actually going, past a deadband set above the noise floor
		 * so the noise cannot unblank the filter it exists to suppress. Same
		 * shape as the output-correction bleed above, which likewise runs
		 * faster while the head is moving. OHMD_RIFT_VEL_ADAPTIVE=0 pins the
		 * old fixed-tau behaviour for comparison. */
		{
			/* Rest noise to ignore, and the speed at which tau is halved. */
			static const float LIN_FLOOR = 0.10f, LIN_REF = 0.25f;  /* m/s */
			static const float ANG_FLOOR = 0.10f, ANG_REF = 0.50f;  /* rad/s */
			static float tau_s = -1.0f;
			static bool adaptive = true;

			if (tau_s < 0.0f) {
				const char *e = getenv("OHMD_RIFT_VEL_SMOOTH_MS");
				float ms = e ? (float) atof(e) : 25.0f;
				if (ms < 0.0f)
					ms = 0.0f;
				tau_s = ms / 1000.0f;

				e = getenv("OHMD_RIFT_VEL_ADAPTIVE");
				adaptive = !(e && atoi(e) == 0);
				LOGI("velocity smoothing: tau %.0f ms, %s", tau_s * 1000.0f,
				     adaptive ? "speed-adaptive" : "fixed");
			}
			if (tau_s > 0.0f) {
				float sdt = 0.001f;
				if (dev->vel_filt_ts != 0 && dev->device_time_ns > dev->vel_filt_ts) {
					sdt = (float)(dev->device_time_ns - dev->vel_filt_ts) * 1e-9f;
					if (sdt > 0.1f)
						sdt = 0.1f;
				}
				dev->vel_filt_ts = dev->device_time_ns;
				vec3f tmp;

				/* Gate on the INCOMING magnitude, not the filtered one: it has
				 * to react at the instant motion starts, which is precisely
				 * when a stale velocity does the visible damage. */
				float lin_tau = tau_s, ang_tau = tau_s;
				if (adaptive) {
					float ls = ovec3f_get_length(&dev->reported_lin_vel) - LIN_FLOOR;
					float as = ovec3f_get_length(&dev->reported_ang_vel) - ANG_FLOOR;
					if (ls < 0.0f)
						ls = 0.0f;
					if (as < 0.0f)
						as = 0.0f;
					lin_tau = tau_s / (1.0f + ls / LIN_REF);
					ang_tau = tau_s / (1.0f + as / ANG_REF);
				}

				float lin_alpha = sdt / (lin_tau + sdt);
				float ang_alpha = sdt / (ang_tau + sdt);

				ovec3f_subtract(&dev->reported_lin_vel, &dev->vel_filt, &tmp);
				ovec3f_multiply_scalar(&tmp, lin_alpha, &tmp);
				ovec3f_add(&dev->vel_filt, &tmp, &dev->vel_filt);
				dev->reported_lin_vel = dev->vel_filt;

				ovec3f_subtract(&dev->reported_ang_vel, &dev->ang_vel_filt, &tmp);
				ovec3f_multiply_scalar(&tmp, ang_alpha, &tmp);
				ovec3f_add(&dev->ang_vel_filt, &tmp, &dev->ang_vel_filt);
				dev->reported_ang_vel = dev->ang_vel_filt;
			}
		}
	}

	/* Optional in-driver forward prediction, off by default.
	 *
	 * SteamVR already extrapolates the pose to photon time itself, using the
	 * velocities and poseTimeOffset we hand it (see driver_openhmd.cpp), so
	 * predicting here as well would DOUBLE-predict and overshoot. This exists
	 * for A/B measurement, and for API consumers that do no prediction of
	 * their own (openhmd_simple_example, the pose-log harness). Leave it at 0
	 * for SteamVR. */
	posef out_pose = dev->reported_pose;
	vec3f out_vel = dev->reported_lin_vel;
	float predict_s = out_predict_secs();
	if (predict_s > 0.0f)
		rift_predict_pose(&out_pose, &out_vel, &dev->reported_lin_accel,
		                  &dev->reported_ang_vel, predict_s);

	if (pose)
		*pose = out_pose;
	if (ang_vel)
		*ang_vel = dev->reported_ang_vel;
	if (accel)
		*accel = dev->reported_lin_accel;
	if (vel)
		*vel = out_vel;

	ohmd_unlock_mutex (dev->device_lock);
}

/* Age of the fused pose: how long ago the IMU sample it is an estimate at
 * arrived, on the host clock. SteamVR's DriverPose_t.poseTimeOffset wants the
 * negative of this, so it predicts forward from the correct epoch instead of
 * assuming the pose is fresh. */
uint64_t rift_tracked_device_get_pose_age_ns(rift_tracked_device *dev_base, uint64_t now_local_ts)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	uint64_t age = 0;

	ohmd_lock_mutex (dev->device_lock);
	if (dev->last_imu_local_ts != 0 && now_local_ts > dev->last_imu_local_ts)
		age = now_local_ts - dev->last_imu_local_ts;

	/* This age IS the prediction horizon: SteamVR extrapolates from here to
	 * photon time, so every millisecond of it multiplies whatever error is in
	 * the exported velocity. Over-predicting looks like the image continuing
	 * to move after the head has stopped - and costs nothing at rest, where
	 * the velocities are zero. So report the horizon and the speeds it gets
	 * multiplied by, which together bound how far the displayed pose can be
	 * thrown past the measured one. */
	if (dev->base.id == 0) {
		static uint64_t n, age_sum, age_max, last_report;
		static double speed_sum, speed_max, ang_sum, ang_max, throw_max;
		float speed = ovec3f_get_length(&dev->reported_lin_vel);
		float ang = ovec3f_get_length(&dev->reported_ang_vel);
		double age_s = age * 1e-9;
		double thrown = speed * age_s;

		n++;
		age_sum += age;
		if (age > age_max)
			age_max = age;
		speed_sum += speed;
		if (speed > speed_max)
			speed_max = speed;
		ang_sum += ang;
		if (ang > ang_max)
			ang_max = ang;
		if (thrown > throw_max)
			throw_max = thrown;

		if (last_report == 0)
			last_report = now_local_ts;
		if (now_local_ts - last_report > 10000000000ULL) {
			LOGI("pose export over 10 s: age mean %.1f ms max %.1f ms | speed "
				"mean %.2f max %.2f m/s, ang mean %.0f max %.0f deg/s | "
				"age*speed worst %.1f mm (%"PRIu64" samples)",
				1e-6 * age_sum / n, 1e-6 * age_max,
				speed_sum / n, speed_max,
				ang_sum / n * 180.0 / M_PI, ang_max * 180.0 / M_PI,
				1000.0 * throw_max, n);
			n = age_sum = age_max = 0;
			speed_sum = speed_max = ang_sum = ang_max = throw_max = 0.0;
			last_report = now_local_ts;
		}
	}
	ohmd_unlock_mutex (dev->device_lock);

	return age;
}

static rift_tracker_pose_delay_slot *get_matching_delay_slot(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info);

/* Retrieve the latest model pose estimate from a delay slot into the exposure info.
 * Because we can receive pose updates and new IMU data between frame capture and
 * when we go to do a visual search, and those can improve the estimate of the
 * pose estimate we had when the exposure happened */
bool rift_tracked_device_get_latest_exposure_info_pose (rift_tracked_device *dev_base, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	rift_tracker_pose_delay_slot *slot = NULL;
	bool res = false;

	if (dev_info->fusion_slot == -1)
		return false;

	ohmd_lock_mutex (dev->device_lock);

	slot = get_matching_delay_slot(dev, dev_info);
	if (slot != NULL) {
		posef imu_global_pose;
		vec3f global_pos_error, global_rot_error;

		fusion_get_delay_slot_pose_at(dev, dev_info->device_time_ns, slot->slot_id, &imu_global_pose,
						NULL, NULL, NULL, &global_pos_error, &global_rot_error);

		oposef_apply(&dev->model_from_fusion, &imu_global_pose, &dev_info->capture_pose);

		int i;
		for (i = 0; i < 3; i++) {
			if (global_rot_error.arr[i] < MIN_ROT_ERROR)
				global_rot_error.arr[i] = MIN_ROT_ERROR;
			if (global_pos_error.arr[i] < MIN_POS_ERROR)
				global_pos_error.arr[i] = MIN_POS_ERROR;
		}

		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_pos_error, &dev_info->pos_error);
		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_rot_error, &dev_info->rot_error);
		res = true;
	}
	else {
		/* If we failed to get the pose, it means the delay slot was overridden,
		 * so clear it in the device info */
		dev_info->fusion_slot = -1;
	}

	ohmd_unlock_mutex (dev->device_lock);

	return res;
}

bool rift_tracked_device_get_gravity_model(rift_tracked_device *dev_base, vec3f *out)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	vec3f grav_fusion;
	bool ret = false;

	ohmd_lock_mutex (dev->device_lock);
	if (use_ovr_fusion() && rift_fusion_ovr_get_gravity_body(&dev->ovr_fusion, &grav_fusion)) {
		/* the filter works in the fusion (IMU) frame; the capture and the
		 * optical solve are both in the model frame */
		quatf model_from_fusion = dev->fusion_from_model.orient;
		oquatf_inverse(&model_from_fusion);
		oquatf_get_rotated(&model_from_fusion, &grav_fusion, out);
		ovec3f_normalize_me(out);
		ret = true;
	}
	ohmd_unlock_mutex (dev->device_lock);

	return ret;
}

bool rift_tracked_device_model_pose_update(rift_tracked_device *dev_base, uint64_t local_ts, uint64_t frame_start_local_ts, rift_tracker_exposure_info *exposure_info,
    rift_pose_metrics *score, posef *model_pose, const rift_joint_view *view, const char *source)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);
	uint64_t frame_device_time_ns = 0;
	rift_tracker_pose_delay_slot *slot = NULL;
	int frame_fusion_slot = -1;
	bool update_position = false;
	bool update_orientation = false;
	posef imu_pose;

	ohmd_lock_mutex (dev->device_lock);

	/* Apply the fusion->model pose on top of the passed model->global pose,
	 * to get the global IMU pose */
	oposef_apply(&dev->fusion_from_model, model_pose, &imu_pose);

	rift_tracked_device_send_imu_debug(dev);

	if (dev->index >= exposure_info->n_devices) {
		ohmd_unlock_mutex (dev->device_lock);
		return true; /* Returning true means the sensor will stop searching any further */
	}

	/* This device existed when the exposure was taken and therefore has info */
	rift_tracked_device_exposure_info *dev_info = exposure_info->devices + dev->index;
	frame_device_time_ns = dev_info->device_time_ns;

	/* Timing health: how stale this fix is by the time it lands. */
	if (dev->device_time_ns > frame_device_time_ns) {
		uint64_t age_ns = dev->device_time_ns - frame_device_time_ns;
		if (rift_sync_monitor_pose_age(&dev->sync, age_ns) &&
		    (dev->sync.late_poses % 100) == 1) {
			LOGW("Device %d: late pose time %.3f ms from %s (%u so far, worst %.3f ms)",
				dev->base.id, age_ns / 1000000.0, source, dev->sync.late_poses,
				dev->sync.worst_pose_age_ns / 1000000.0);
		}
	}

	slot = get_matching_delay_slot(dev, dev_info);
	if (slot == NULL) {
		dev->reports_dropped_late++;
		if ((dev->reports_dropped_late % 100) == 1) {
			LOGW("Device %d: %u observations dropped because their exposure's "
				"delay slot had already been recycled (latest from %s, %.1f ms "
				"old) - the joint reconstruction loses a view each time",
				dev->base.id, dev->reports_dropped_late, source,
				(double)(dev->device_time_ns - frame_device_time_ns) / 1000000.0);
		}
	}
	if (slot != NULL) {
		quatf orient_diff;
		vec3f pos_error, rot_error;

		ovec3f_subtract(&model_pose->pos, &dev_info->capture_pose.pos, &pos_error);

		oquatf_diff(&model_pose->orient, &dev_info->capture_pose.orient, &orient_diff);
		oquatf_normalize_me(&orient_diff);
		oquatf_to_rotation(&orient_diff, &rot_error);

		LOGD ("Got pose update for delay slot %d for dev %d, ts %llu (delay %f) orient %f %f %f %f diff %f %f %f pos %f %f %f diff %f %f %f from %s\n",
			slot->slot_id, dev->base.id,
			(unsigned long long) frame_device_time_ns, (double) (dev->device_time_ns - frame_device_time_ns) / 1000000000.0,
			model_pose->orient.x, model_pose->orient.y, model_pose->orient.z, model_pose->orient.w,
			rot_error.x, rot_error.y, rot_error.z,
			model_pose->pos.x, model_pose->pos.y, model_pose->pos.z,
			pos_error.x, pos_error.y, pos_error.z,
			source);

		/* Confidence tiering: only strong, LED-ID-verified observations
		 * correct the pose at full weight. Weak matches that disagree with
		 * the prior are rejected outright while tracking is healthy - the
		 * IMU rides through until a trustworthy match arrives. */
		bool recently_tracked = dev->last_observed_pose_ts != 0 &&
			(dev->device_time_ns - dev->last_observed_pose_ts) < (POSE_LOST_THRESHOLD * 1000000UL);
		float obs_scale;
		if (POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_STRONG | RIFT_POSE_MATCH_LED_IDS))
			obs_scale = 1.0f;
		else if (POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_LED_IDS))
			obs_scale = 1.5f; /* LED IDs verified: correspondence is right, geometry merely imprecise */
		else if (POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_STRONG))
			obs_scale = 2.0f;
		else if (POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_POSITION | RIFT_POSE_MATCH_ORIENT))
			obs_scale = 4.0f; /* agrees with the prior, reinforce weakly */
		else if (!recently_tracked)
			obs_scale = 6.0f; /* re-acquiring after loss: accept what we can get */
		else
			obs_scale = 0.0f; /* weak and disagreeing while tracked: reject */

		/* Temporary telemetry: log the observation quality mix for the HMD */
		if (dev->base.id == 0) {
			static int q_total = 0, q_full = 0, q_ids = 0, q_strong = 0, q_prior = 0, q_reacq = 0, q_rej = 0;
			static int f_ids = 0, f_strong = 0;
			q_total++;
			if (POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_LED_IDS)) f_ids++;
			if (POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_STRONG)) f_strong++;
			if (obs_scale == 1.0f) q_full++;
			else if (obs_scale == 1.5f) q_ids++;
			else if (obs_scale == 2.0f) q_strong++;
			else if (obs_scale == 4.0f) q_prior++;
			else if (obs_scale == 6.0f) q_reacq++;
			else q_rej++;
			if (q_total >= 300) {
				LOGI("HMD obs quality: full(strong+ids) %d ids-verified %d strong %d prior-agree %d reacquire %d rejected %d | flags: led-ids %d strong %d",
					q_full, q_ids, q_strong, q_prior, q_reacq, q_rej, f_ids, f_strong);
				q_total = q_full = q_ids = q_strong = q_prior = q_reacq = q_rej = 0;
				f_ids = f_strong = 0;
			}
		}

		/* If this observation was based on a prior, but position didn't match and we already received a newer observation,
		 * ignore it. */
		if (obs_scale == 0.0f) {
			update_position = false;
		}
		else if (dev_info->had_pose_lock && !POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_POSITION) && dev->last_observed_pose_ts > frame_device_time_ns) {
			update_position = false;
			LOGI("Ignoring position observation with error %f %f %f (prior stddev was %f %f %f)\n",
				pos_error.x, pos_error.y, pos_error.z,
				dev_info->pos_error.x, dev_info->pos_error.y, dev_info->pos_error.z);
		}
		else {
			update_position = true;
		}

		/* If we have a strong match, update both position and orientation */
		if (obs_scale != 0.0f && POSE_HAS_FLAGS(score, RIFT_POSE_MATCH_ORIENT)) {
			update_orientation = true;
			if ((dev->device_time_ns - dev->last_observed_orient_ts) > (POSE_LOST_ORIENT_THRESHOLD * 1000000UL)) {
				LOGI("Matched orientation after %f sec", (dev->device_time_ns - dev->last_observed_pose_ts) / 1000000000.0);
			}
			/* Only update the time if we're actually going to apply this matched orientation below */
			if (update_position)
				dev->last_observed_orient_ts = dev->device_time_ns;
		}
		else if (obs_scale != 0.0f &&
		    (dev->device_time_ns - dev->last_observed_orient_ts) > (POSE_LOST_ORIENT_THRESHOLD * 1000000UL)) {
			LOGI("Forcing orientation observation");
			update_orientation = true;
			/* Don't update the orientation match time here - only do that on an actual match */
		}
		else {
			/* FIXME: If roll and pitch are acceptable (the gravity vector matched), but yaw is out of spec, we could perhaps do a
			 * yaw-only update for this device and see if that brings it into matching orientation */
		}

		if (update_position) {
			if (dev->last_acquired_pose_lock_ts == 0)
				dev->last_acquired_pose_lock_ts = dev->last_acquired_pose_lock_ts;

			/* Same-exposure observation merge: both sensors expose on the
			 * same sync pulse, so their fixes reference the same delay slot
			 * and arrive ms apart in racing order. Applied sequentially at
			 * near-full authority, the fused pose lands at whichever
			 * sensor's solution corrected last and oscillates between them
			 * as the race winner flips (~7.5 Hz, amplitude = cross-camera
			 * disagreement). Instead, correct toward the confidence-weighted
			 * mean position of ALL of this exposure's used observations: the
			 * cycle then ends at the same midpoint regardless of order.
			 *
			 * Orientation merges too, but ONLY across same-exposure reports
			 * whose orientation was actually applied (orient_used): the two
			 * simultaneous verified solutions can disagree by degrees on
			 * sparse LED geometry (measured 4.7 deg constant on a Touch
			 * ring, position dragged ~86 mm through the lever arm), and
			 * overwrite semantics alternate the fused orientation between
			 * them with arrival order. This is unlike the earlier
			 * cross-EXPOSURE orientation blending that caused output thrash:
			 * nothing persists past the exposure, and a bad orientation is
			 * still superseded by the next exposure's fix. */
			posef fusion_target = imu_pose;
			float fusion_scale = obs_scale;
			bool merged = false;

			/* Preferred path: reconstruct ONE pose from every sensor's
			 * correspondences for this exposure. Falls through to the
			 * weighted merge when there is only one view, when the solve
			 * fails, or when it cannot satisfy every camera - the last of
			 * which means the extrinsics are wrong, not the pose. */
			if (joint_solve_enabled() && view != NULL && dev->led_pos != NULL) {
				rift_joint_view views[RIFT_MAX_SENSORS];
				int n_views = 0;

				for (int vi = 0; vi < slot->n_pose_reports && n_views < RIFT_MAX_SENSORS - 1; vi++) {
					if (slot->pose_reports[vi].have_view)
						views[n_views++] = slot->pose_reports[vi].view;
				}

				if (n_views > 0) {
					posef joint_model;
					rift_joint_result jres;

					views[n_views++] = *view;

					if (rift_joint_pose_solve(dev->led_pos, dev->n_led_pos,
							views, n_views, model_pose, &joint_model, &jres)) {
						if (jres.worst_view_px <= JOINT_ACCEPT_PX) {
							oposef_apply(&dev->fusion_from_model, &joint_model, &fusion_target);
							/* n independent views: the reconstruction is
							 * correspondingly tighter than one sensor's */
							fusion_scale = obs_scale / sqrtf((float) n_views);
							merged = true;
							dev->joint_solved++;
						} else {
							dev->joint_rejected++;
							if ((dev->joint_rejected % 100) == 1) {
								LOGI("Device %d: joint reconstruction over %d cameras left %.2f px "
									"in the worst camera (>%.1f px) - check sensor calibration",
									dev->base.id, n_views, jres.worst_view_px, JOINT_ACCEPT_PX);
							}
						}
					}
				} else {
					dev->joint_single++;
				}

				if (merged && (dev->joint_solved % 300) == 1) {
					LOGI("Device %d: joint reconstruction %u solved, %u rejected, "
						"%u single-camera (%u observations arrived too late to count)",
						dev->base.id, dev->joint_solved, dev->joint_rejected,
						dev->joint_single, dev->reports_dropped_late);
				}
			}

			if (!merged && obs_merge_enabled() && slot->n_used_reports > 0) {
				float w_sum = 1.0f / (obs_scale * obs_scale);
				vec3f p_acc = imu_pose.pos;
				ovec3f_multiply_scalar(&p_acc, w_sum, &p_acc);
				float qw_sum = update_orientation ? 1.0f / (obs_scale * obs_scale) : 0.0f;
				quatf q_acc = imu_pose.orient;
				int i;

				for (i = 0; i < slot->n_pose_reports; i++) {
					rift_tracker_pose_report *prev = slot->pose_reports + i;
					if (!prev->report_used || prev->obs_scale <= 0.0f)
						continue;
					float w = 1.0f / (prev->obs_scale * prev->obs_scale);
					vec3f p = prev->pose.pos;
					ovec3f_multiply_scalar(&p, w, &p);
					ovec3f_add(&p_acc, &p, &p_acc);
					w_sum += w;
					merged = true;

					if (update_orientation && prev->orient_used) {
						/* incremental weighted quaternion mean */
						oquatf_slerp(w / (qw_sum + w), &q_acc, &prev->pose.orient, true, &q_acc);
						oquatf_normalize_me(&q_acc);
						qw_sum += w;
					}
				}

				if (merged) {
					ovec3f_multiply_scalar(&p_acc, 1.0f / w_sum, &fusion_target.pos);
					fusion_target.orient = q_acc;
					fusion_scale = 1.0f / sqrtf(w_sum);

					vec3f merge_shift;
					ovec3f_subtract(&fusion_target.pos, &imu_pose.pos, &merge_shift);
					LOGD("dev %d slot %d: merged %d same-exposure obs, shift %f %f %f",
						dev->base.id, slot->slot_id, slot->n_used_reports + 1,
						merge_shift.x, merge_shift.y, merge_shift.z);

					/* Rate-limited merge telemetry (per device) */
					dev->merge_count++;
					dev->merge_shift_accum += ovec3f_get_length(&merge_shift);
					if (dev->merge_count % 300 == 0) {
						LOGI("dev %d: %d same-exposure merges, mean |shift| %.1f mm (orient merged: %s)",
							dev->base.id, dev->merge_count,
							1000.0 * dev->merge_shift_accum / 300.0,
							(update_orientation && qw_sum > 1.0f / (obs_scale * obs_scale)) ? "yes" : "no");
						dev->merge_shift_accum = 0;
					}
				}
			}

			/* While tracking is healthy, fold the discrete correction this
			 * update makes to the present state into the output offset, so
			 * the displayed pose stays continuous and the correction bleeds
			 * in smoothly (get_view_pose). On re-acquisition, snap. */
			bool absorb_jump = out_corr_enabled() &&
				recently_tracked && obs_scale > 0.0f && obs_scale < 6.0f;
			posef state_before, state_after;

			if (absorb_jump)
				fusion_get_pose_at(dev, dev->device_time_ns, &state_before, NULL, NULL, NULL, NULL, NULL);

			/* A merged fix supersedes the pending error from this exposure's
			 * earlier fix(es) — replace, don't blend. */
			if (update_orientation) {
				fusion_pose_update(dev, dev->device_time_ns, &fusion_target, slot->slot_id, fusion_scale, merged);
			} else {
				fusion_position_update(dev, dev->device_time_ns, &fusion_target.pos, slot->slot_id, fusion_scale, merged);
			}

			if (absorb_jump) {
				fusion_get_pose_at(dev, dev->device_time_ns, &state_after, NULL, NULL, NULL, NULL, NULL);

				quatf after_inv = state_after.orient, orient_jump, new_corr;
				oquatf_inverse(&after_inv);
				oquatf_mult(&state_before.orient, &after_inv, &orient_jump);
				oquatf_mult(&dev->out_corr_orient, &orient_jump, &new_corr);
				oquatf_normalize_me(&new_corr);
				dev->out_corr_orient = new_corr;

				vec3f pos_jump;
				ovec3f_subtract(&state_before.pos, &state_after.pos, &pos_jump);
				ovec3f_add(&dev->out_corr_pos, &pos_jump, &dev->out_corr_pos);
			}

			dev->last_observed_pose_ts = dev->device_time_ns;
			/* the merged position is the better estimate for search priors */
			dev->last_observed_pose = fusion_target;
		}

		frame_fusion_slot = slot->slot_id;

		if (slot->n_pose_reports < RIFT_MAX_SENSORS) {
			rift_tracker_pose_report *report = slot->pose_reports + slot->n_pose_reports;

			report->report_used = update_position;
			report->orient_used = update_position && update_orientation;
			report->source = source;
			/* store the RAW observation (not the merged target) so later
			 * same-exposure merges weight original measurements, and the
			 * per-sensor confidence it was integrated with */
			report->pose = imu_pose;
			report->score = *score;
			report->obs_scale = obs_scale;
			report->have_view = (view != NULL);
			if (view != NULL)
				report->view = *view;

			if (update_position)
				slot->n_used_reports++;
			slot->n_pose_reports++;

			if (update_position)
				extrinsic_refine_measure(dev, slot, report);
		}
	}

	rift_tracked_device_send_debug_printf(dev, local_ts, "{ \"type\": \"pose\", \"local-ts\": %llu, "
		"\"device-ts\": %u, \"frame-start-local-ts\": %llu, "
		"\"frame-local-ts\": %llu, \"frame-hmd-ts\": %u, "
		"\"frame-exposure-count\": %u, \"frame-device-ts\": %llu, \"frame-fusion-slot\": %d, "
		"\"source\": \"%s\", "
		"\"score-flags\": %d, \"update-position\": %d, \"update-orient\": %d, "
		"\"pos\" : [ %f, %f, %f ], "
		"\"orient\" : [ %f, %f, %f, %f ], "
		"\"capture-pos\" : [ %f, %f, %f ], "
		"\"capture-orient\" : [ %f, %f, %f, %f ], "
		"\"rot-std-dev\" : [ %f, %f, %f ], "
		"\"pos-std-dev\" : [ %f, %f, %f ] "
		"}",
		(unsigned long long) local_ts, dev->device_time_ns,
		(unsigned long long) frame_start_local_ts,
		(unsigned long long) exposure_info->local_ts, exposure_info->hmd_ts,
		exposure_info->count,
		(unsigned long long) frame_device_time_ns, frame_fusion_slot,
		source, score->match_flags, update_position, update_orientation,
		model_pose->pos.x, model_pose->pos.y, model_pose->pos.z,
		model_pose->orient.x, model_pose->orient.y, model_pose->orient.z, model_pose->orient.w,
		dev_info->capture_pose.pos.x, dev_info->capture_pose.pos.y,
		dev_info->capture_pose.pos.z, dev_info->capture_pose.orient.x,
		dev_info->capture_pose.orient.y, dev_info->capture_pose.orient.z,
		dev_info->capture_pose.orient.w,
		dev_info->rot_error.x, dev_info->rot_error.y, dev_info->rot_error.z,
		dev_info->pos_error.x, dev_info->pos_error.y, dev_info->pos_error.z
	);
	ohmd_unlock_mutex (dev->device_lock);

	return update_position || update_orientation;
}

/* Called with the device lock held */
void rift_tracked_device_get_model_pose_locked(rift_tracked_device_priv *dev, uint64_t device_ts, posef *pose, vec3f *pos_error, vec3f *rot_error)
{
	posef imu_global_pose, model_pose;
	vec3f global_pos_error, global_rot_error, vel;

	fusion_get_pose_at(dev, dev->device_time_ns, &imu_global_pose, &vel, NULL, NULL, &global_pos_error, &global_rot_error);

	/* The fusion state is only current as of the integration head;
	 * extrapolate to the requested time (constant velocity, clamped) */
	if (device_ts > dev->device_time_ns) {
		uint64_t gap = device_ts - dev->device_time_ns;
		if (gap < 30000000ULL) {
			vec3f adv;
			ovec3f_multiply_scalar(&vel, (float)(gap * 1e-9), &adv);
			ovec3f_add(&imu_global_pose.pos, &adv, &imu_global_pose.pos);
		}
	}

	/* Apply the pose conversion from IMU->model */
	oposef_apply(&dev->model_from_fusion, &imu_global_pose, &model_pose);

	if (pos_error) {
		int i;

		for (i = 0; i < 3; i++) {
			if (global_pos_error.arr[i] < MIN_POS_ERROR)
				global_pos_error.arr[i] = MIN_POS_ERROR;
		}

		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_pos_error, pos_error);
	}
	if (rot_error) {
		int i;

		for (i = 0; i < 3; i++) {
			if (global_rot_error.arr[i] < MIN_ROT_ERROR)
				global_rot_error.arr[i] = MIN_ROT_ERROR;
		}

		oquatf_get_rotated_abs(&dev->model_from_fusion.orient, &global_rot_error, rot_error);
	}

	dev->model_pose.orient = model_pose.orient;
	if (dev->device_time_ns - dev->last_observed_pose_ts < (POSE_LOST_THRESHOLD * 1000000UL)) {
		/* Don't let the device move unless there's a recent observation of actual position */
		dev->model_pose.pos = model_pose.pos;
	}
	*pose = dev->model_pose;

	LOGD ("Reporting pose for dev %d, orient %f %f %f %f pos %f %f %f",
		dev->base.id,
		pose->orient.x, pose->orient.y, pose->orient.z, pose->orient.w,
		pose->pos.x, pose->pos.y, pose->pos.z);
}

/* Called with the device lock held */
static void
rift_tracked_device_send_imu_debug(rift_tracked_device_priv *dev)
{
	int i;

	if (dev->num_pending_imu_observations == 0)
		return;

	if (dev->debug_metadata && ohmd_pw_debug_stream_connected(dev->debug_metadata)) {
		char debug_str[1024];

		for (i = 0; i < dev->num_pending_imu_observations; i++) {
			rift_tracked_device_imu_observation *obs = dev->pending_imu_observations + i;

			snprintf (debug_str, 1024, ",\n{ \"type\": \"imu\", \"local-ts\": %llu, "
				 "\"device-ts\": %llu, \"dt\": %f, "
				 "\"ang_vel\": [ %f, %f, %f ], \"accel\": [ %f, %f, %f ], "
				 "\"mag\": [ %f, %f, %f ] }",
				(unsigned long long) obs->local_ts,
				(unsigned long long) obs->device_ts,
				obs->dt,
				obs->ang_vel.x, obs->ang_vel.y, obs->ang_vel.z,
				obs->accel.x, obs->accel.y, obs->accel.z,
				obs->mag.x, obs->mag.y, obs->mag.z);

			debug_str[1023] = '\0';

			ohmd_pw_debug_stream_push (dev->debug_metadata, obs->local_ts, debug_str);
		}
	}

	dev->num_pending_imu_observations = 0;
}

static void
rift_tracked_device_send_debug_printf(rift_tracked_device_priv *dev, uint64_t local_ts, const char *fmt, ...)
{
	if (dev->debug_metadata && ohmd_pw_debug_stream_connected(dev->debug_metadata)) {
		char debug_str[1024];
		va_list args;

		/* Send any pending IMU debug first */
		rift_tracked_device_send_imu_debug(dev);

		/* Print output string and send */
		va_start(args, fmt);
		vsnprintf(debug_str, 1024, fmt, args);
		va_end(args);

		debug_str[1023] = '\0';

		ohmd_pw_debug_stream_push (dev->debug_metadata, local_ts, debug_str);
	}
}

static rift_tracker_pose_delay_slot *
find_free_delay_slot(rift_tracked_device_priv *dev)
{
	/* Pose observation delay slots */
	for (int i = 0; i < dev->n_delay_slots; i++) {
		int slot_no = dev->delay_slot_index;
		rift_tracker_pose_delay_slot *slot = dev->delay_slots + slot_no;

		/* Cycle through the free delay slots */
		dev->delay_slot_index = (slot_no+1) % dev->n_delay_slots;

		if (slot->use_count == 0)
			return slot;
	}

	/* Failed to find a free slot */
	return NULL;
}

static rift_tracker_pose_delay_slot *
reclaim_delay_slot(rift_tracked_device_priv *dev)
{
	/* Pose observation delay slots */
	for (int i = 0; i < dev->n_delay_slots; i++) {
		rift_tracker_pose_delay_slot *slot = dev->delay_slots + i;

		/* If a slot already received a pose observation, use that one */
		/* FIXME: Check that the poses were integrated, and integrate them as-needed if not */
		if (slot->valid && slot->n_used_reports > 0)
			return slot;
	}

	/* Failed to find a free slot */
	return NULL;
}


static rift_tracker_pose_delay_slot *
get_matching_delay_slot(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracker_pose_delay_slot *slot = NULL;
	int slot_no = dev_info->fusion_slot;

	if (slot_no >= 0 && slot_no < dev->n_delay_slots) {
		slot = dev->delay_slots + slot_no;
	}

	if (slot && slot->valid && slot->device_time_ns == dev_info->device_time_ns)
		return slot;

	return NULL;
}

/* Called with the device lock held. Allocate a delay slot and populate the device exposure info */
static void
rift_tracked_device_on_new_exposure(rift_tracked_device_priv *dev, uint64_t exposure_local_ts, rift_tracked_device_exposure_info *dev_info) {
	rift_tracker_pose_delay_slot *slot = find_free_delay_slot(dev);

	/* Map the exposure moment onto this device's clock. The integration
	 * head (device_time_ns) is only current as of the last IMU sample;
	 * for radio-connected devices that sample is transport-latency old
	 * (compensated in rift.c) and the exposure notification arrives later
	 * still. Without this, every vision fix is applied to a state from
	 * AFTER the exposure and drags the fused pose backward along the
	 * motion vector (measured: fusion trails optics by v * ~10-15 ms). */
	uint64_t device_time = dev->device_time_ns;
	if (dev->last_imu_local_ts != 0 && exposure_local_ts > dev->last_imu_local_ts) {
		uint64_t gap = exposure_local_ts - dev->last_imu_local_ts;
		if (gap < 30000000ULL) /* sanity: ignore stale/sleeping streams */
			device_time += gap;
	}
	dev_info->device_time_ns = device_time;

	if (slot == NULL) {
		/* We might reclaim a busy delay slot if some frame search is being slow and we already got an observation from another camera */
		slot = reclaim_delay_slot(dev);
		if (slot) {
			LOGI ("Reclaimed delay slot %d for dev %d, ts %llu (delay %f)", slot->slot_id, dev->base.id, (unsigned long long) dev->device_time_ns,
				(double) (dev->device_time_ns - slot->device_time_ns) / 1000000000.0);
		}
	}

	if (dev->device_time_ns - dev->last_observed_pose_ts < (POSE_LOST_THRESHOLD * 1000000UL))
		dev_info->had_pose_lock = true;
	else {
		dev_info->had_pose_lock = false;
		if (dev->last_acquired_pose_lock_ts != 0) {
			LOGI("Device %d Lost pose_lock at TS %" PRIu64 " after %" PRIu64 "\n",
			    dev->base.id, dev->device_time_ns, dev->device_time_ns - dev->last_acquired_pose_lock_ts);
			dev->last_acquired_pose_lock_ts = 0;
		}
	}
	dev_info->last_acquired_pose_lock_ts = dev->last_acquired_pose_lock_ts;

	rift_tracked_device_get_model_pose_locked(dev, dev_info->device_time_ns, &dev_info->capture_pose, &dev_info->pos_error, &dev_info->rot_error);

	if (slot) {
		slot->device_time_ns = dev_info->device_time_ns;
		slot->valid = true;
		slot->use_count = 0;
		slot->n_pose_reports = 0;
		slot->n_used_reports = 0;

		LOGD ("Assigning free delay slot %d for dev %d, ts %llu", slot->slot_id, dev->base.id, (unsigned long long) dev->device_time_ns);
		dev_info->fusion_slot = slot->slot_id;

		/* Tell the kalman filter to prepare the delay slot */
		fusion_prepare_delay_slot(dev, dev_info->device_time_ns, slot->slot_id);

		/* Clear the last no-free-delay-slot tracking to avoid logging noise */
		dev->last_no_free_delay_slot = 0;
	}
	else {
		if (dev->last_no_free_delay_slot != 0 && (dev->device_time_ns - dev->last_no_free_delay_slot > (NO_FREE_DELAY_SLOT_THRESHOLD * 1000000UL))) {
			LOGW("No free delay slot for dev %d @ ts %llu (for %ums now)", dev->base.id,
				  (unsigned long long) dev->device_time_ns, NO_FREE_DELAY_SLOT_THRESHOLD);
		}
		dev->last_no_free_delay_slot = dev->device_time_ns;
		dev_info->fusion_slot = -1;
	}
}

static int
rift_tracked_device_exposure_claim(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracker_pose_delay_slot *slot = get_matching_delay_slot(dev, dev_info);

	/* There is a delay slot for this frame, claim it */
	if (slot) {
		slot->use_count++;

		LOGD ("Claimed delay slot %d for dev %d, ts %llu. use_count now %d",
			slot->slot_id, dev->base.id, (unsigned long long) dev_info->device_time_ns, slot->use_count);

		return slot->slot_id;
	}
	else {
		/* The slot was not allocated (we missed the exposure event), or it
		 * was overridden by a later exposure because there's not enough slots */
		if (dev_info->fusion_slot != -1) {
#if LOGLEVEL == 0
			rift_tracker_pose_delay_slot *slot = dev->delay_slots + dev_info->fusion_slot;

			LOGD ("Lost delay slot %d for dev %d, ts %llu (slot valid %d ts %llu)",
				dev_info->fusion_slot, dev->base.id, (unsigned long long) dev_info->device_time_ns,
				slot->valid, (unsigned long long) slot->device_time_ns);
#endif
		}
	}

	return -1;
}

static void
rift_tracked_device_exposure_release_locked(rift_tracked_device_priv *dev, rift_tracked_device_exposure_info *dev_info)
{
	rift_tracker_pose_delay_slot *slot = get_matching_delay_slot(dev, dev_info);

	/* There is a delay slot for this frame, release it */
	if (slot) {
		if (slot->use_count > 0) {
			slot->use_count--;
			LOGD ("Released delay slot %d for dev %d, ts %llu. use_count now %d",
				dev_info->fusion_slot, dev->base.id, (unsigned long long) dev_info->device_time_ns,
				slot->use_count);
			/* Clear the fusion slot in the dev info now that it's released */
			dev_info->fusion_slot = -1;
		}
	}
}

void rift_tracked_device_frame_release(rift_tracked_device *dev_base, rift_tracker_exposure_info *exposure_info)
{
	rift_tracked_device_priv *dev = (rift_tracked_device_priv *) (dev_base);

	ohmd_lock_mutex (dev->device_lock);
	if (dev->index < exposure_info->n_devices) {
		/* This device existed when the exposure was taken and therefore has info */
		rift_tracked_device_exposure_info *dev_info = exposure_info->devices + dev->index;
		/* Check that the exposure_info matches the device we expect */
		assert (dev_info->device_index == dev->index);
		rift_tracked_device_exposure_release_locked(dev, dev_info);
	}
	ohmd_unlock_mutex (dev->device_lock);
}

void rift_tracker_update_sensor_pose(rift_tracker_ctx *tracker_ctx, rift_sensor_ctx *sensor, posef *new_pose)
{
	const char *serial_no = rift_sensor_serial_no(sensor);

	ohmd_lock_mutex (tracker_ctx->tracker_lock);
	rift_tracker_config_set_sensor_pose(&tracker_ctx->config, serial_no, new_pose);
	rift_tracker_config_save(tracker_ctx->ohmd_ctx, &tracker_ctx->config);
	ohmd_unlock_mutex (tracker_ctx->tracker_lock);
}

/* Set OHMD_RIFT_NO_AUTO_CALIB=1 to disable automatic extrinsic calibration and
 * rely purely on the stored room config (for A/B testing). */
static bool auto_calib_enabled(void)
{
	static int enabled = -1;
	if (enabled == -1) {
		const char *e = getenv("OHMD_RIFT_NO_AUTO_CALIB");
		enabled = !(e && e[0] == '1');
	}
	return enabled;
}

static int sensor_index(rift_tracker_ctx *ctx, const char *serial)
{
	int i;
	for (i = 0; i < ctx->n_sensors; i++) {
		if (strcmp(rift_sensor_serial_no(ctx->sensors[i]), serial) == 0)
			return i;
	}
	return -1;
}

/* Called from a sensor's analysis thread with the device pose solved in that
 * sensor's OWN frame, whether or not the sensor knows where it is. Pairs it
 * with the anchor sensor's solution for the same exposure; the two together
 * determine the transform between the cameras outright, with no movement and
 * no user step ("Single frame calibration, camera %d" in the Oculus runtime).
 */
void rift_tracker_add_calib_obs(rift_tracker_ctx *ctx, rift_sensor_ctx *sensor,
	rift_tracked_device *dev, rift_tracker_exposure_info *exposure_info,
	const posef *obj_cam_pose)
{
	rift_calib_exposure *e = NULL;
	int idx, i;

	/* Counted before any early return, so a feed that never starts is
	 * distinguishable from one that starts and stalls - the two look identical
	 * from the outside, both being silent. */
	ctx->cam_calib_obs++;
	if ((ctx->cam_calib_obs % 900) == 1) {
		LOGI("calib feed: %u observations offered (auto_calib %d, sensors %u, "
			"exposure_info %s, device %d)",
			ctx->cam_calib_obs, auto_calib_enabled() ? 1 : 0, ctx->n_sensors,
			exposure_info ? "yes" : "NULL", dev->id);
	}

	if (!auto_calib_enabled() || ctx->n_sensors < 2 || exposure_info == NULL)
		return;
	/* Only the HMD: its constellation is dense enough that a single-frame
	 * solve is trustworthy, which a Touch ring's is not. */
	if (dev->id != 0)
		return;

	idx = sensor_index(ctx, rift_sensor_serial_no(sensor));
	if (idx < 0)
		return;

	ohmd_lock_mutex(ctx->calib_lock);

	for (i = 0; i < RIFT_CALIB_EXP_SLOTS; i++) {
		if (ctx->calib_exp[i].valid && ctx->calib_exp[i].count == exposure_info->count) {
			e = ctx->calib_exp + i;
			break;
		}
	}
	if (e == NULL) {
		e = ctx->calib_exp + ctx->calib_exp_next;
		ctx->calib_exp_next = (ctx->calib_exp_next + 1) % RIFT_CALIB_EXP_SLOTS;
		memset(e, 0, sizeof(*e));
		e->valid = true;
		e->count = exposure_info->count;
	}

	e->obj_cam[idx] = *obj_cam_pose;
	e->have[idx] = true;

	/* Everything is measured relative to sensor 0, the same anchor the
	 * online refinement uses. */
	if (idx != 0 && e->have[0]) {
		rift_cam_calib_add(ctx->cam_calib + idx, e->obj_cam + 0, e->obj_cam + idx);
		ctx->cam_calib_pairs++;
	} else if (idx == 0) {
		for (i = 1; i < ctx->n_sensors; i++) {
			if (e->have[i]) {
				rift_cam_calib_add(ctx->cam_calib + i, e->obj_cam + 0, e->obj_cam + i);
				ctx->cam_calib_pairs++;
			}
		}
	}

	/* Periodic visibility into the calibration feed. Without it a silent
	 * pairing failure is indistinguishable from a healthy converged one -
	 * both simply produce no output. */
	if ((ctx->cam_calib_obs % 900) == 1) {
		int si;
		for (si = 1; si < ctx->n_sensors; si++) {
			rift_cam_calib *cc = ctx->cam_calib + si;
			LOGI("calib feed %s: %u obs, %u paired, history %u over %u viewpoints, "
				"%u solves, residual %.2f px, %s",
				rift_sensor_serial_no(ctx->sensors[si]), ctx->cam_calib_obs,
				ctx->cam_calib_pairs, cc->n_hist, cc->bins_seen, cc->n_solves,
				cc->residual_px,
				cc->state == RIFT_CAM_CALIBRATED ? "CALIBRATED" :
				cc->state == RIFT_CAM_ESTIMATED ? "estimated" : "uncalibrated");
		}
	}

	ohmd_unlock_mutex(ctx->calib_lock);
}

/* Called from a sensor's own analysis thread (no locks held). Once this
 * sensor's relative pose to the anchor has converged, place it. This runs
 * even for a sensor that has no pose at all, which is the case the old
 * gravity bootstrap could not cover: it needs the HMD's fused pose, and a
 * sensor that has never contributed a fix does not get one. */
void rift_tracker_cam_calib_apply(rift_tracker_ctx *ctx, rift_sensor_ctx *sensor)
{
	posef rel, rel_in_use, anchor_world, newp, cur;
	rift_cam_calib snapshot;
	rift_cam_calib_action action;
	const posef *in_use = NULL;
	float r_in_use = -1.0f, r_est = -1.0f;
	int idx, stored_views = 0;
	bool adopted, recovering, had_pose;
	vec3f d = {{ 0, 0, 0 }};
	float dang = 0.0f;

	if (!auto_calib_enabled() || ctx->n_sensors < 2)
		return;

	idx = sensor_index(ctx, rift_sensor_serial_no(sensor));
	if (idx <= 0)
		return; /* the anchor defines the frame; it has nothing to adopt */

	ohmd_lock_mutex(ctx->calib_lock);
	snapshot = ctx->cam_calib[idx];
	adopted = ctx->cam_calib_adopted[idx];
	recovering = ctx->cam_calib_recovering[idx];
	ohmd_unlock_mutex(ctx->calib_lock);

	/* The anchor has to know where IT is before anything can be placed
	 * against it. It gets that from the existing gravity bootstrap in
	 * rift-sensor-pose-search.c, or from the stored config. */
	if (!rift_sensor_have_pose(ctx->sensors[0]))
		return;
	rift_sensor_get_pose(ctx->sensors[0], &anchor_world);

	had_pose = rift_sensor_have_pose(sensor);
	if (had_pose) {
		/* What the poses currently in use claim the relative geometry is.
		 * On the first pass that IS the stored room config, so this is where
		 * a stale file is caught; afterwards it is the pose we adopted
		 * ourselves, so a knocked sensor is caught the same way. */
		posef ref_inv = anchor_world;
		rift_sensor_get_pose(sensor, &cur);
		oposef_inverse(&ref_inv);
		oposef_apply(&cur, &ref_inv, &rel_in_use);
		in_use = &rel_in_use;

		ohmd_lock_mutex(ctx->tracker_lock);
		stored_views = rift_tracker_config_get_sensor_viewpoints(&ctx->config,
			rift_sensor_serial_no(sensor));
		ohmd_unlock_mutex(ctx->tracker_lock);
	}

	action = rift_cam_calib_decide(&snapshot, in_use, adopted, recovering,
		stored_views, &r_in_use, &r_est);

	switch (action) {
	case RIFT_CAM_CALIB_WAIT:
		return;

	case RIFT_CAM_CALIB_KEEP:
		if (!adopted) {
			LOGI("sensor %s: keeping the stored calibration - it was fitted "
				"over %d viewpoints and this session has seen %u "
				"(it leaves %.2f px here)",
				rift_sensor_serial_no(sensor), stored_views,
				snapshot.bins_seen, r_in_use);
			ohmd_lock_mutex(ctx->calib_lock);
			ctx->cam_calib_adopted[idx] = true;
			ohmd_unlock_mutex(ctx->calib_lock);
		}
		return;

	case RIFT_CAM_CALIB_RESET:
		LOGI("sensor %s: camera moved - the calibration in use leaves %.1f px "
			"over %u observations (>%.0f px). Resetting history and "
			"re-deriving it from what is seen now.",
			rift_sensor_serial_no(sensor), r_in_use, snapshot.n_hist,
			(double) RIFT_CAM_CALIB_MOVED_PX);
		ohmd_lock_mutex(ctx->calib_lock);
		rift_cam_calib_reset(ctx->cam_calib + idx);
		ctx->cam_calib[idx].n_resets++;
		ctx->cam_calib_adopted[idx] = false;
		ctx->cam_calib_recovering[idx] = true;
		ohmd_unlock_mutex(ctx->calib_lock);
		return;

	case RIFT_CAM_CALIB_ADOPT:
		break;
	}

	if (!rift_cam_calib_get(&snapshot, &rel))
		return;
	rift_cam_calib_to_world(&anchor_world, &rel, &newp);

	if (had_pose) {
		quatf inv = cur.orient, dq;
		ovec3f_subtract(&newp.pos, &cur.pos, &d);
		oquatf_inverse(&inv);
		oquatf_mult(&newp.orient, &inv, &dq);
		dang = 2.0f * acosf(OHMD_MIN(1.0f, fabsf(dq.w)));
	}

	LOGI("sensor %s: %s calibration from %u poses over %u viewpoints "
		"(stored: %d), residual %.2f px%s",
		rift_sensor_serial_no(sensor),
		recovering ? "re-derived" : (adopted ? "re-solved" : "adopted"),
		snapshot.n_hist, snapshot.bins_seen, stored_views, r_est,
		snapshot.settled ? " - SETTLED" : " (estimated)");
	if (had_pose) {
		LOGI("sensor %s: %s calibration left %.2f px; moving the sensor "
			"%.1f mm / %.2f deg",
			rift_sensor_serial_no(sensor),
			recovering ? "the pre-move" : (adopted ? "its previous" : "the stored"),
			r_in_use, ovec3f_get_length(&d) * 1000.0f, RAD_TO_DEG(dang));
	}

	rift_sensor_set_pose(sensor, &newp);
	rift_tracker_update_sensor_pose(ctx, sensor, &newp);
	ohmd_lock_mutex(ctx->tracker_lock);
	rift_tracker_config_set_sensor_viewpoints(&ctx->config,
		rift_sensor_serial_no(sensor), snapshot.bins_seen);
	ohmd_unlock_mutex(ctx->tracker_lock);
	notify_camera_moved(ctx, &d, dang);

	/* The online refiner's window measures mismatch against the pose that
	 * was just replaced, so every measurement in it is now about a world
	 * that no longer exists - applying it would drag the sensor back.
	 * Restart the window, and re-anchor the net-drift reference so the
	 * deliberate jump isn't reported as creep. */
	ohmd_lock_mutex(ctx->refine_lock);
	ctx->refine[idx].n_meas = 0;
	ctx->refine[idx].have_start = false;
	ctx->refine[idx].quiet_rounds = 0;
	ctx->refine[idx].settled = false;
	ohmd_unlock_mutex(ctx->refine_lock);

	ohmd_lock_mutex(ctx->calib_lock);
	ctx->cam_calib_adopted[idx] = true;
	ctx->cam_calib_recovering[idx] = false;
	ohmd_unlock_mutex(ctx->calib_lock);
}

/* Called from a sensor's own analysis thread (no locks held) after each
 * pose delivery: if its refinement window is ripe, take a damped clamped
 * step of this sensor's camera pose toward agreement with the anchor.
 * The window only applies when the HMD covered enough space that static
 * PnP bias (which looks like a large phantom mismatch) has averaged out. */
void rift_tracker_extrinsic_refine_apply(rift_tracker_ctx *ctx, rift_sensor_ctx *sensor)
{
	int idx = -1, i;
	vec3f mean_pos;
	quatf mean_orient;
	bool have = false;

	if (!extrinsic_refine_enabled() || ctx->n_sensors < 2)
		return;
	for (i = 1; i < ctx->n_sensors; i++) {
		if (ctx->sensors[i] == sensor) {
			idx = i;
			break;
		}
	}
	if (idx < 0)
		return; /* the anchor sensor (index 0) is never refined */

	uint64_t now = ohmd_monotonic_get(ctx->ohmd_ctx);
	rift_extrinsic_refine *r = ctx->refine + idx;

	ohmd_lock_mutex(ctx->refine_lock);
	if (r->n_meas >= EXTRINSIC_REFINE_MIN_MEAS &&
	    now - r->last_apply_ts >= EXTRINSIC_REFINE_INTERVAL_NS) {
		vec3f span;
		ovec3f_subtract(&r->span_max, &r->span_min, &span);
		float coherence = ovec3f_get_length(&r->fwd_sum) / r->n_meas;
		float mm = ovec3f_get_length(&r->mean_dpos);
		float ma = 2.0f * acosf(OHMD_MIN(1.0f, fabsf(r->mean_dorient.w)));
		if (ovec3f_get_length(&span) >= EXTRINSIC_REFINE_MIN_SPAN_M &&
		    coherence <= EXTRINSIC_REFINE_MAX_FWD_COHERENCE &&
		    (mm >= EXTRINSIC_REFINE_DEADBAND_POS ||
		     ma >= EXTRINSIC_REFINE_DEADBAND_ANG)) {
			mean_pos = r->mean_dpos;
			mean_orient = r->mean_dorient;
			have = true;
		}
		/* restart the window either way, so a stale static-geometry
		 * accumulation can't linger and get applied much later */
		r->n_meas = 0;
		r->last_apply_ts = now;
	}
	ohmd_unlock_mutex(ctx->refine_lock);

	if (!have) {
		/* Nothing worth correcting this round. Enough of those in a row and
		 * the sensor's calibration is settled. */
		ohmd_lock_mutex(ctx->refine_lock);
		if (!r->settled && ++r->quiet_rounds >= EXTRINSIC_SETTLE_QUIET_ROUNDS) {
			r->settled = true;
			LOGI("sensor %s calibration SETTLED", rift_sensor_serial_no(sensor));
		}
		ohmd_unlock_mutex(ctx->refine_lock);
		return;
	}

	float dist = ovec3f_get_length(&mean_pos);
	float t_pos = EXTRINSIC_REFINE_GAIN;
	if (dist * t_pos > EXTRINSIC_REFINE_MAX_POS_STEP)
		t_pos = EXTRINSIC_REFINE_MAX_POS_STEP / dist;

	float ang = 2.0f * acosf(OHMD_MIN(1.0f, fabsf(mean_orient.w)));
	float t_ang = EXTRINSIC_REFINE_GAIN;
	if (ang > 0.0f && ang * t_ang > EXTRINSIC_REFINE_MAX_ANG_STEP)
		t_ang = EXTRINSIC_REFINE_MAX_ANG_STEP / ang;

	posef step, cur, newp;
	quatf id = {{ 0.0, 0.0, 0.0, 1.0 }};
	ovec3f_multiply_scalar(&mean_pos, t_pos, &step.pos);
	oquatf_slerp(t_ang, &id, &mean_orient, true, &step.orient);
	oquatf_normalize_me(&step.orient);

	rift_sensor_get_pose(sensor, &cur);
	oposef_apply(&cur, &step, &newp);
	rift_sensor_set_pose(sensor, &newp);

	/* Every fix taken through this sensor was computed against the pose we
	 * just changed. Say so, in the same terms the runtime does, and make the
	 * fusion stop trusting its pending error. */
	{
		vec3f applied_d;
		float applied_ang = ang * t_ang;
		ovec3f_subtract(&newp.pos, &cur.pos, &applied_d);

		LOGI("sensor %s CameraPoseChange: dt %.1f %.1f %.1f (%.1f) mm  dr %.2f deg",
			rift_sensor_serial_no(sensor),
			applied_d.x * 1000.0, applied_d.y * 1000.0, applied_d.z * 1000.0,
			ovec3f_get_length(&applied_d) * 1000.0, RAD_TO_DEG(applied_ang));

		notify_camera_moved(ctx, &applied_d, applied_ang);

		ohmd_lock_mutex(ctx->refine_lock);
		r->quiet_rounds = 0;
		if (r->settled) {
			r->settled = false;
			LOGI("sensor %s calibration UNSETTLED", rift_sensor_serial_no(sensor));
		}
		ohmd_unlock_mutex(ctx->refine_lock);
	}

	bool save = false;
	ohmd_lock_mutex(ctx->refine_lock);
	if (!r->have_start) {
		r->start_pose = cur;
		r->have_start = true;
	}
	r->applied_pos_total += dist * t_pos;
	r->applied_ang_total += ang * t_ang;
	if (now - r->last_save_ts > 60000000000ULL) {
		r->last_save_ts = now;
		save = true;
	}
	double cum_pos = r->applied_pos_total, cum_ang = r->applied_ang_total;
	posef start = r->start_pose;
	ohmd_unlock_mutex(ctx->refine_lock);

	/* net drift from the session-start pose: distinguishes one-way creep
	 * (real correction) from oscillation (bias chasing) */
	vec3f net_d;
	quatf start_inv = start.orient, net_q;
	ovec3f_subtract(&newp.pos, &start.pos, &net_d);
	oquatf_inverse(&start_inv);
	oquatf_mult(&newp.orient, &start_inv, &net_q);
	float net_ang = 2.0f * acosf(OHMD_MIN(1.0f, fabsf(net_q.w)));

	LOGI("extrinsic refine: sensor %s step %.1f mm / %.2f deg "
		"(window mismatch %.1f mm / %.2f deg; net from start %.1f mm / %.2f deg; "
		"summed steps %.1f mm / %.2f deg)",
		rift_sensor_serial_no(sensor), dist * t_pos * 1000.0, RAD_TO_DEG(ang * t_ang),
		dist * 1000.0, RAD_TO_DEG(ang),
		ovec3f_get_length(&net_d) * 1000.0, RAD_TO_DEG(net_ang),
		cum_pos * 1000.0, RAD_TO_DEG(cum_ang));

	/* Persist the healed pose occasionally so it survives restarts.
	 * NOTE: saved as-is (room offset is currently identity when set). */
	if (save)
		rift_tracker_update_sensor_pose(ctx, sensor, &newp);
}
