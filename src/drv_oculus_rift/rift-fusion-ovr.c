/*
 * Complementary-filter fusion ported from Oculus SDK 0.3.2
 * (LibOVR/Src/OVR_SensorFusion.cpp) — see rift-fusion-ovr.h.
 *
 * Constants are the SDK's own:
 *   tilt:      gain 0.25 /s, snap above 0.1 rad, confidence gates 0.75/0.5,
 *              accel low-passed in the body frame with gain 2.5 over ~1 s
 *   vision yaw: gain 0.25 /s, snap above 0.1 rad
 *   position:  gains (10,10,8) /s into position, (50,50,32) /s into
 *              velocity, (25,25,16) /s into accel bias; snap above 0.1 m
 *   vision considered recent for 70 ms
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#include <assert.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#include "rift-fusion-ovr.h"

#define GRAVITY_MAG 9.8f

#define TILT_GAIN 0.25f
#define TILT_SNAP_THRESHOLD 0.1f
#define VISION_YAW_GAIN 0.25f
#define VISION_YAW_SNAP_THRESHOLD 0.1f
#define VISION_POS_SNAP_THRESHOLD 0.1f
#define VISION_RECENT_NS (70000000ULL)          /* 0.07 s */
#define VISION_REACQUIRE_NS (500000000ULL)      /* fixes older than this snap */
#define ACCEL_FILTER_GAIN 2.5f                  /* SensorFilterBodyFrame */
#define GRAV_STAT_TAU 1.0f                      /* ~1000-sample window */

static const vec3f GAIN_POS = {{ 10, 10, 8 }};
static const vec3f GAIN_VEL = {{ 50, 50, 32 }};
static const vec3f GAIN_ACCEL = {{ 25, 25, 16 }};

/* OVR_SensorFusion.cpp vectorAlignmentRotation(): rotation taking 'from'
 * onto 'to' */
static void vector_alignment_rotation(const vec3f *from, const vec3f *to, quatf *out)
{
	vec3f axis;
	ovec3f_cross(from, to, &axis);
	float axis_len = ovec3f_get_length(&axis);
	if (axis_len < 1e-9f) {
		oquatf_set(out, 0, 0, 0, 1);
		return;
	}
	oquatf_init_axis(out, &axis, ovec3f_get_angle(from, to));
}

/* Scale a (small) rotation to fraction t of its angle — the effect of the
 * SDK's error.Nlerp(identity, t) */
static void quat_scale_rotation(const quatf *q, float t, quatf *out)
{
	vec3f rot;
	quatf tmp = *q;
	oquatf_normalize_me(&tmp);
	oquatf_to_rotation(&tmp, &rot);
	ovec3f_multiply_scalar(&rot, t, &rot);
	oquatf_from_rotation(out, &rot);
}

static float quat_angle(const quatf *q)
{
	vec3f rot;
	quatf tmp = *q;
	oquatf_normalize_me(&tmp);
	oquatf_to_rotation(&tmp, &rot);
	return ovec3f_get_length(&rot);
}

/* OVR_SensorFusion.cpp extractYawRotation(): the Y-axis component of a
 * world-frame error rotation */
static void extract_yaw_rotation(const quatf *error, quatf *out)
{
	if (error->y == 0.0f) {
		oquatf_set(out, 0, 0, 0, 1);
		return;
	}
	double phi = atan2(error->w, error->y);
	double alpha = M_PI - 2.0 * phi;
	vec3f up = {{ 0, 1, 0 }};
	oquatf_init_axis(out, &up, (float)alpha);
}

/* Apply world-frame orientation correction C to the live state and remove
 * it from the stored vision error and slot snapshots (the SDK applies C to
 * its exposure records — same algebra: error' = error * C^-1) */
static void apply_orient_correction(rift_fusion_ovr *f, const quatf *correction)
{
	quatf tmp;

	oquatf_mult(correction, &f->pose.orient, &tmp);
	oquatf_normalize_me(&tmp);
	f->pose.orient = tmp;

	if (f->have_vision_error) {
		quatf corr_inv = *correction;
		oquatf_inverse(&corr_inv);
		oquatf_mult(&f->vision_error.orient, &corr_inv, &tmp);
		oquatf_normalize_me(&tmp);
		f->vision_error.orient = tmp;
	}

	for (int i = 0; i < f->num_slots; i++) {
		if (!f->slots[i].used)
			continue;
		oquatf_mult(correction, &f->slots[i].pose.orient, &tmp);
		oquatf_normalize_me(&tmp);
		f->slots[i].pose.orient = tmp;
	}
}

static void apply_pos_correction(rift_fusion_ovr *f, const vec3f *correction)
{
	ovec3f_add(&f->pose.pos, correction, &f->pose.pos);

	if (f->have_vision_error)
		ovec3f_subtract(&f->vision_error.pos, correction, &f->vision_error.pos);

	for (int i = 0; i < f->num_slots; i++) {
		if (f->slots[i].used)
			ovec3f_add(&f->slots[i].pose.pos, correction, &f->slots[i].pose.pos);
	}
}

void rift_fusion_ovr_init(rift_fusion_ovr *f, const posef *init_pose, int num_delay_slots)
{
	assert(num_delay_slots <= RIFT_FUSION_OVR_MAX_SLOTS);
	memset(f, 0, sizeof(*f));
	f->pose = *init_pose;
	f->num_slots = num_delay_slots;
	f->snap_pending = true;
}

void rift_fusion_ovr_clear(rift_fusion_ovr *f)
{
	(void)f;
}

void rift_fusion_ovr_prepare_delay_slot(rift_fusion_ovr *f, uint64_t time, int delay_slot)
{
	assert(delay_slot >= 0 && delay_slot < f->num_slots);
	f->slots[delay_slot].used = true;
	f->slots[delay_slot].time_ns = time;
	f->slots[delay_slot].pose = f->pose;
	f->slots[delay_slot].lin_vel = f->lin_vel;

	/* The slot time can be slightly ahead of the last integrated sample
	 * (radio devices: the exposure lands between IMU arrivals) — advance
	 * the snapshot by constant velocity so vision errors are computed
	 * against the state AT the exposure, not before it */
	if (f->have_time && time > f->time_ns) {
		uint64_t gap = time - f->time_ns;
		if (gap < 30000000ULL) {
			vec3f adv;
			ovec3f_multiply_scalar(&f->lin_vel, (float)(gap * 1e-9), &adv);
			ovec3f_add(&f->slots[delay_slot].pose.pos, &adv, &f->slots[delay_slot].pose.pos);
		}
	}
}

void rift_fusion_ovr_release_delay_slot(rift_fusion_ovr *f, int delay_slot)
{
	assert(delay_slot >= 0 && delay_slot < f->num_slots);
	f->slots[delay_slot].used = false;
}

/* SensorFilterBodyFrame port: low-pass the accelerometer in the body frame
 * (counter-rotating the state by the gyro delta each sample) and track
 * world-frame mean/variance for the confidence gate:
 * confidence = clamp(0.48 - 0.1 ln(stddev), 0, 1) * fill_ratio */
static float gravity_filter_update(rift_fusion_ovr *f, const vec3f *accel,
	float dt, const quatf *delta_q, bool saturated)
{
	if (saturated) {
		/* Keep the stored estimate in the current body frame, but do not let a
		 * clipped reading into it - it would poison the low-pass for as long
		 * as the filter's time constant. */
		if (f->grav_init) {
			quatf dq_inv = *delta_q;
			vec3f rotated;
			oquatf_inverse(&dq_inv);
			oquatf_get_rotated(&dq_inv, &f->grav_filter, &rotated);
			f->grav_filter = rotated;
		}
		return 0.0f;
	}

	if (!f->grav_init) {
		f->grav_filter = *accel;
		f->grav_init = true;
	} else {
		quatf dq_inv = *delta_q;
		vec3f rotated, diff;
		oquatf_inverse(&dq_inv);
		oquatf_get_rotated(&dq_inv, &f->grav_filter, &rotated);
		f->grav_filter = rotated;
		ovec3f_subtract(accel, &f->grav_filter, &diff);
		ovec3f_multiply_scalar(&diff, ACCEL_FILTER_GAIN * dt, &diff);
		ovec3f_add(&f->grav_filter, &diff, &f->grav_filter);
	}

	vec3f in_world, delta;
	oquatf_get_rotated(&f->pose.orient, &f->grav_filter, &in_world);

	float blend = dt / GRAV_STAT_TAU;
	if (blend > 1.0f)
		blend = 1.0f;
	ovec3f_subtract(&in_world, &f->grav_mean, &delta);
	vec3f step = delta;
	ovec3f_multiply_scalar(&step, blend, &step);
	ovec3f_add(&f->grav_mean, &step, &f->grav_mean);
	float dev_sq = ovec3f_get_dot(&delta, &delta);
	f->grav_var += (dev_sq - f->grav_var) * blend;

	f->grav_warmup += blend;
	if (f->grav_warmup > 1.0f)
		f->grav_warmup = 1.0f;

	float stddev = sqrtf(f->grav_var);
	if (stddev < 1e-6f)
		stddev = 1e-6f;
	float conf = 0.48f - 0.1f * logf(stddev);
	if (conf < 0.0f)
		conf = 0.0f;
	if (conf > 1.0f)
		conf = 1.0f;
	return conf * f->grav_warmup;
}

/* OVR_SensorFusion.cpp applyTiltCorrection() */
static void apply_tilt_correction(rift_fusion_ovr *f, float dt, float confidence)
{
	const vec3f up = {{ 0, 1, 0 }};
	vec3f accel_world;
	quatf error, correction;

	oquatf_get_rotated(&f->pose.orient, &f->grav_filter, &accel_world);
	if (ovec3f_get_length(&accel_world) < 1e-6f)
		return;

	vector_alignment_rotation(&accel_world, &up, &error);

	if (f->stage <= 1 || (quat_angle(&error) > TILT_SNAP_THRESHOLD && confidence > 0.75f)) {
		/* full correction at start-up, or large error with high confidence */
		correction = error;
	} else if (confidence > 0.5f) {
		quat_scale_rotation(&error, TILT_GAIN * dt, &correction);
	} else {
		/* accelerometer unreliable due to movement */
		return;
	}

	/* Tilt corrections in the SDK do not touch the exposure records:
	 * apply to the live orientation only */
	quatf tmp;
	oquatf_mult(&correction, &f->pose.orient, &tmp);
	oquatf_normalize_me(&tmp);
	f->pose.orient = tmp;
}

/* The 0.3.2-era SDK corrected tilt from the accelerometer alone and used
 * vision for yaw only — fine for a head, but hand controllers experience
 * sustained linear (centripetal) acceleration that masquerades as tilted
 * gravity with LOW variance, so the accel path confidently locks the tilt
 * up to ~10-15 deg wrong and yaw-only vision can never repair it (measured:
 * constant per-run tilt offsets on Touch, grip/motion dependent, absent on
 * the HMD). Correct the full vision orientation error instead: yaw as the
 * SDK did, then the residual (tilt) with its own gain.
 * OHMD_RIFT_NO_VISION_TILT=1 restores yaw-only behaviour for A/B. */
#define VISION_TILT_GAIN 0.5f
#define VISION_TILT_SNAP_THRESHOLD 0.15f

static bool vision_tilt_enabled(void)
{
	static int enabled = -1;
	if (enabled == -1) {
		const char *e = getenv("OHMD_RIFT_NO_VISION_TILT");
		enabled = !(e && e[0] == '1');
	}
	return enabled;
}

/* OVR_SensorFusion.cpp applyVisionYawCorrection(), extended with tilt */
static void apply_vision_yaw_correction(rift_fusion_ovr *f, float dt)
{
	quatf yaw_error, correction;

	extract_yaw_rotation(&f->vision_error.orient, &yaw_error);

	if (quat_angle(&yaw_error) > VISION_YAW_SNAP_THRESHOLD)
		correction = yaw_error; /* high error: jump to the vision pose */
	else
		quat_scale_rotation(&yaw_error, VISION_YAW_GAIN * dt, &correction);

	apply_orient_correction(f, &correction);

	if (!vision_tilt_enabled())
		return;

	/* apply_orient_correction() updated vision_error: what remains is the
	 * unapplied yaw fraction plus the tilt. Strip the yaw again and treat
	 * the residual as the tilt error. */
	quatf yaw_rem, tilt_error, tilt_corr;
	extract_yaw_rotation(&f->vision_error.orient, &yaw_rem);
	oquatf_inverse(&yaw_rem);
	oquatf_mult(&f->vision_error.orient, &yaw_rem, &tilt_error);
	oquatf_normalize_me(&tilt_error);

	if (quat_angle(&tilt_error) > VISION_TILT_SNAP_THRESHOLD)
		tilt_corr = tilt_error;
	else
		quat_scale_rotation(&tilt_error, VISION_TILT_GAIN * dt, &tilt_corr);

	apply_orient_correction(f, &tilt_corr);
}

/* OVR_SensorFusion.cpp applyPositionCorrection() */
static void apply_position_correction(rift_fusion_ovr *f, float dt)
{
	vec3f err = f->vision_error.pos;

	if (ovec3f_get_length(&err) > VISION_POS_SNAP_THRESHOLD || f->snap_pending) {
		/* high error or reacquire: jump to the vision position */
		apply_pos_correction(f, &err);
		ovec3f_set(&f->accel_offset, 0, 0, 0);
		f->snap_pending = false;
		return;
	}

	vec3f corr_pos, corr_vel, corr_accel;
	for (int i = 0; i < 3; i++) {
		corr_pos.arr[i] = err.arr[i] * GAIN_POS.arr[i] * dt;
		corr_vel.arr[i] = err.arr[i] * GAIN_VEL.arr[i] * dt;
		corr_accel.arr[i] = err.arr[i] * GAIN_ACCEL.arr[i] * dt;
	}
	apply_pos_correction(f, &corr_pos);
	ovec3f_add(&f->lin_vel, &corr_vel, &f->lin_vel);
	ovec3f_add(&f->accel_offset, &corr_accel, &f->accel_offset);
}

void rift_fusion_ovr_imu_update(rift_fusion_ovr *f, uint64_t time,
	const vec3f *ang_vel, const vec3f *accel, const vec3f *mag, bool accel_saturated)
{
	(void)mag; /* vision replaces the magnetometer (SDK: MagCalibrated false) */

	float dt = 0.001f;
	if (f->have_time && time > f->time_ns) {
		dt = (float)(time - f->time_ns) * 1e-9f;
		if (dt > 0.1f)
			dt = 0.1f;
	}
	f->time_ns = time;
	f->have_time = true;
	f->stage++;

	bool vision_recent = f->have_vision_error &&
		(time - f->last_vision_ns) < VISION_RECENT_NS;

	/* StoreAndIntegrateGyro: Q = Q * quat(w, |w| dt) (body frame) */
	quatf delta_q = {{ 0, 0, 0, 1 }};
	float angle = ovec3f_get_length(ang_vel) * dt;
	if (angle > 0.0f) {
		vec3f axis = *ang_vel;
		oquatf_init_axis(&delta_q, &axis, angle);
		quatf tmp;
		oquatf_mult(&f->pose.orient, &delta_q, &tmp);
		f->pose.orient = tmp;
	}
	f->ang_vel = *ang_vel;

	/* World-frame acceleration, gravity removed */
	vec3f accel_world;
	oquatf_get_rotated(&f->pose.orient, accel, &accel_world);
	accel_world.y -= GRAVITY_MAG;
	f->lin_accel = accel_world;

	/* StoreAndIntegrateAccelerometer — position only rides the IMU while
	 * vision is recent; otherwise hold position and zero velocity */
	if (vision_recent) {
		vec3f a, v_step, a_step;
		ovec3f_add(&accel_world, &f->accel_offset, &a);
		v_step = f->lin_vel;
		ovec3f_multiply_scalar(&v_step, dt, &v_step);
		a_step = a;
		ovec3f_multiply_scalar(&a_step, 0.5f * dt * dt, &a_step);
		ovec3f_add(&f->pose.pos, &v_step, &f->pose.pos);
		ovec3f_add(&f->pose.pos, &a_step, &f->pose.pos);
		ovec3f_multiply_scalar(&a, dt, &a);
		ovec3f_add(&f->lin_vel, &a, &f->lin_vel);
	} else {
		ovec3f_set(&f->lin_vel, 0, 0, 0);
	}

	float confidence = gravity_filter_update(f, accel, dt, &delta_q, accel_saturated);

	/* A clipped accelerometer reading points the wrong way, so it cannot say
	 * which way is down. Keep integrating - the gyro is still good - but skip
	 * the tilt correction entirely. Note it is not enough to drop the
	 * confidence to zero: apply_tilt_correction's start-up branch snaps to the
	 * accelerometer regardless of confidence, so a clipped first sample would
	 * otherwise throw the orientation straight over. */
	if (!accel_saturated)
		apply_tilt_correction(f, dt, confidence);
	if (vision_recent) {
		apply_vision_yaw_correction(f, dt);
		apply_position_correction(f, dt);
	}

	if ((f->stage & 0xFF) == 0)
		oquatf_normalize_me(&f->pose.orient);
}

/* Vision fix: compute the world-frame error against the state snapshot
 * taken at the exposure. obs_scale > 1 (weaker observation) divides the
 * correction gains by scaling the stored error's effective weight — we
 * fold it in by shrinking the error itself, matching how the UKF path
 * inflates R. */
static void vision_fix(rift_fusion_ovr *f, uint64_t time, const posef *vision_pose,
	bool have_orient, int delay_slot, float obs_scale, bool replace_pending)
{
	assert(delay_slot >= 0 && delay_slot < f->num_slots);
	rift_fusion_ovr_slot *slot = &f->slots[delay_slot];
	if (!slot->used)
		return;

	bool reacquire = !f->have_vision_error ||
		(time - f->last_vision_ns) > VISION_REACQUIRE_NS;

	posef err;
	quatf rec_inv = slot->pose.orient;
	oquatf_inverse(&rec_inv);
	if (have_orient) {
		oquatf_mult(&vision_pose->orient, &rec_inv, &err.orient);
		oquatf_normalize_me(&err.orient);
	} else {
		oquatf_set(&err.orient, 0, 0, 0, 1);
	}
	ovec3f_subtract(&vision_pose->pos, &slot->pose.pos, &err.pos);

	if (obs_scale > 1.0f) {
		quat_scale_rotation(&err.orient, 1.0f / obs_scale, &err.orient);
		ovec3f_multiply_scalar(&err.pos, 1.0f / obs_scale, &err.pos);
	}

	/* Blend interleaved POSITION fixes instead of overwriting. With two
	 * cameras alternately reporting, overwrite makes the correction loop
	 * chase each camera's own solution in turn: any disagreement between
	 * them pumps the velocity and accel-bias corrections at the alternation
	 * frequency and the state rings (measured 0.2-0.6 m/s phantom velocity
	 * with two sensors, headset stationary). Averaging the remaining error
	 * with the incoming one converges to the sources' weighted midpoint
	 * instead. With a single camera this only adds one fix period (~19 ms)
	 * of lag. Reacquire still takes the fix in full.
	 *
	 * Orientation deliberately keeps the OVERWRITE semantics: orientation
	 * fixes are gated on matching the prior, so during fast motion they are
	 * sparse, and an occasional bad optical orientation (LED-ID flips under
	 * blur) previously vanished under the next fix. Blending let such errors
	 * persist and mix, and the 0.1 rad yaw-snap path amplified the residue
	 * into 8-160 deg output thrash (~18 events/s at |w|~1.5 rad/s).
	 *
	 * Exception: a merged same-exposure fix (replace_pending) already
	 * contains the earlier fix's information averaged in at the tracker
	 * layer — blending would double-count the stale error, so it
	 * overwrites. */
	if (!reacquire && f->have_vision_error && !replace_pending) {
		ovec3f_add(&f->vision_error.pos, &err.pos, &err.pos);
		ovec3f_multiply_scalar(&err.pos, 0.5f, &err.pos);
	}

	f->vision_error = err;
	f->have_vision_error = true;
	f->last_vision_ns = time;
	if (reacquire) {
		f->snap_pending = true;
		/* On reacquire also snap yaw fully on the next IMU sample by
		 * letting the snap threshold do it (error will be large), and
		 * make position integration live again immediately */
	}
}

void rift_fusion_ovr_pose_update(rift_fusion_ovr *f, uint64_t time,
	posef *pose, int delay_slot, float obs_scale, bool replace_pending)
{
	vision_fix(f, time, pose, true, delay_slot, obs_scale, replace_pending);
}

void rift_fusion_ovr_position_update(rift_fusion_ovr *f, uint64_t time,
	vec3f *position, int delay_slot, float obs_scale, bool replace_pending)
{
	posef p = { .pos = *position };
	oquatf_set(&p.orient, 0, 0, 0, 1);
	vision_fix(f, time, &p, false, delay_slot, obs_scale, replace_pending);
}

/* Std-dev heuristics for the pose-search priors: tight right after a
 * vision fix, growing with age (the UKF supplied covariances here) */
static void error_estimates(rift_fusion_ovr *f, uint64_t time,
	vec3f *pos_error, vec3f *rot_error)
{
	float age = 10.0f;
	if (f->have_vision_error && time > f->last_vision_ns)
		age = (float)(time - f->last_vision_ns) * 1e-9f;
	if (age > 10.0f)
		age = 10.0f;

	if (pos_error) {
		float p = 0.005f + 0.05f * age;
		ovec3f_set(pos_error, p, p, p);
	}
	if (rot_error) {
		float r = 0.02f + 0.05f * age;
		ovec3f_set(rot_error, r, r, r);
	}
}

void rift_fusion_ovr_notify_camera_moved(rift_fusion_ovr *f)
{
	f->have_vision_error = false;
	f->snap_pending = true;
}

bool rift_fusion_ovr_get_gravity_body(rift_fusion_ovr *f, vec3f *out)
{
	if (!f->grav_init || f->grav_warmup < 1.0f)
		return false;
	*out = f->grav_filter;
	return true;
}

void rift_fusion_ovr_get_delay_slot_pose_at(rift_fusion_ovr *f, uint64_t time, int delay_slot,
	posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error)
{
	if (delay_slot != -1) {
		assert(delay_slot >= 0 && delay_slot < f->num_slots);
		if (pose)
			*pose = f->slots[delay_slot].pose;
		if (vel)
			*vel = f->slots[delay_slot].lin_vel;
	} else {
		if (pose)
			*pose = f->pose;
		if (vel)
			*vel = f->lin_vel;
	}
	if (accel)
		*accel = f->lin_accel;
	if (ang_vel)
		*ang_vel = f->ang_vel;
	error_estimates(f, time, pos_error, rot_error);
}

void rift_fusion_ovr_get_pose_at(rift_fusion_ovr *f, uint64_t time,
	posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error)
{
	rift_fusion_ovr_get_delay_slot_pose_at(f, time, -1, pose, vel, accel, ang_vel, pos_error, rot_error);
}
