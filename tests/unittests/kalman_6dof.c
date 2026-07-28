/*
 * Synthetic tests for the 6DOF UKF.
 *
 * There is no hardware in this loop, so these drive the filter with generated
 * IMU samples and pose observations whose truth is known exactly.
 *
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

#include <string.h>
#include <stdlib.h>

#include "tests.h"
#include "drv_oculus_rift/rift-kalman-6dof.h"
#include "drv_oculus_rift/rift-fusion-ovr.h"

#define GRAVITY 9.80665

/* Run `seconds` of a perfectly stationary, level device at `hz`, with a pose
 * observation every 20 ms (roughly the CV1 camera cadence). Returns the final
 * fused pose and its reported position uncertainty. */
static void run_stationary(double seconds, int hz, posef *out_pose,
	vec3f *out_vel, vec3f *out_pos_err)
{
	rift_kalman_6dof_filter f;
	posef truth;
	const vec3f accel = {{ 0.0f, (float)GRAVITY, 0.0f }};
	const vec3f gyro = {{ 0.0f, 0.0f, 0.0f }};

	ovec3f_set(&truth.pos, 0.10f, 1.20f, -1.30f);
	oquatf_set(&truth.orient, 0.0f, 0.0f, 0.0f, 1.0f);

	rift_kalman_6dof_init(&f, &truth, 3);

	const uint64_t step_ns = (uint64_t)(1000000000.0 / hz);
	const uint64_t n = (uint64_t)(seconds * hz);
	const uint64_t obs_every = (uint64_t)(hz / 50);   /* 20 ms */
	uint64_t t = 0;

	for (uint64_t i = 1; i <= n; i++) {
		t = i * step_ns;
		rift_kalman_6dof_imu_update(&f, t, &gyro, &accel, NULL, false);

		if (obs_every > 0 && (i % obs_every) == 0) {
			int slot = (int)((i / obs_every) % 3);
			rift_kalman_6dof_prepare_delay_slot(&f, t, slot);
			rift_kalman_6dof_pose_update(&f, t, &truth, slot, 1.0f);
			rift_kalman_6dof_release_delay_slot(&f, slot);
		}
	}

	vec3f accel_out, ang_vel;
	rift_kalman_6dof_get_pose_at(&f, t, out_pose, out_vel, &accel_out, &ang_vel,
		out_pos_err, NULL);
	rift_kalman_6dof_clear(&f);
}

/* A stationary device, observed at its true pose, must stay there. */
void test_rift_kalman_stationary()
{
	posef pose;
	vec3f vel, pos_err;
	vec3f truth_pos = {{ 0.10f, 1.20f, -1.30f }};

	run_stationary(2.0, 1000, &pose, &vel, &pos_err);

	TAssert(isfinite(pose.pos.x) && isfinite(pose.pos.y) && isfinite(pose.pos.z));
	TAssert(vec3f_eq(pose.pos, truth_pos, 0.01f));      /* within 1 cm */
	TAssert(ovec3f_get_length(&vel) < 0.05f);           /* not drifting */
	TAssert(ovec3f_get_length(&pos_err) < 0.5f);        /* covariance sane */
}

/* The filter must converge to the same answer whether it is fed at 1 kHz or
 * 500 Hz.
 *
 * NOTE this does NOT detect the known process-noise defect (Q is added per
 * call rather than per unit time, so its effective magnitude tracks the sample
 * rate). With pose observations arriving every 20 ms the steady state is
 * dominated by the measurements, and the reported uncertainty barely moves.
 * Verified by running this test both with and without dt-scaled Q: it passes
 * either way. Detecting that defect needs a test driven by dead reckoning
 * alone. */
void test_rift_kalman_two_sample_rates()
{
	posef pose_1k, pose_500;
	vec3f vel_1k, vel_500, err_1k, err_500;

	run_stationary(2.0, 1000, &pose_1k, &vel_1k, &err_1k);
	run_stationary(2.0, 500, &pose_500, &vel_500, &err_500);

	/* same simulated time, half the samples: the answer must agree */
	TAssert(vec3f_eq(pose_1k.pos, pose_500.pos, 0.005f));

	float e1 = ovec3f_get_length(&err_1k);
	float e2 = ovec3f_get_length(&err_500);
	TAssert(e1 > 0.0f && e2 > 0.0f);
	/* reported uncertainty within 25% across a 2x rate change */
	TAssert(fabsf(e1 - e2) / (e1 > e2 ? e1 : e2) < 0.25f);
}

/* A delayed position-only update must not drag the filter clock: the delay
 * slot already represents the lag. */
void test_rift_kalman_delayed_position_update()
{
	rift_kalman_6dof_filter f;
	posef truth, pose;
	const vec3f accel = {{ 0.0f, (float)GRAVITY, 0.0f }};
	const vec3f gyro = {{ 0.0f, 0.0f, 0.0f }};

	ovec3f_set(&truth.pos, 0.0f, 1.0f, -1.0f);
	oquatf_set(&truth.orient, 0.0f, 0.0f, 0.0f, 1.0f);
	rift_kalman_6dof_init(&f, &truth, 3);

	uint64_t t = 0;
	for (int i = 1; i <= 500; i++) {
		t = (uint64_t)i * 1000000ULL;
		rift_kalman_6dof_imu_update(&f, t, &gyro, &accel, NULL, false);
	}

	/* snapshot a slot 20 ms in the past, then feed the observation late */
	rift_kalman_6dof_prepare_delay_slot(&f, t, 0);
	for (int i = 501; i <= 520; i++) {
		t = (uint64_t)i * 1000000ULL;
		rift_kalman_6dof_imu_update(&f, t, &gyro, &accel, NULL, false);
	}
	rift_kalman_6dof_position_update(&f, t, &truth.pos, 0, 1.0f);
	rift_kalman_6dof_release_delay_slot(&f, 0);

	vec3f vel, accel_out, ang_vel, pos_err;
	rift_kalman_6dof_get_pose_at(&f, t, &pose, &vel, &accel_out, &ang_vel, &pos_err, NULL);

	TAssert(isfinite(pose.pos.x) && isfinite(pose.pos.y) && isfinite(pose.pos.z));
	TAssert(vec3f_eq(pose.pos, truth.pos, 0.02f));
	TAssert(ovec3f_get_length(&vel) < 0.2f);

	rift_kalman_6dof_clear(&f);
}

/* A saturated accelerometer reading points the wrong way, so it must not be
 * allowed to pull the tilt. The A/B matters: the same bogus reading fed as
 * UNSATURATED has to move the orientation, or this test proves nothing. */
void test_rift_fusion_ovr_saturated_accel_ignored()
{
	/* a reading that claims "down" is sideways - what a clipped axis looks like */
	const vec3f bogus = {{ (float)GRAVITY, 0.0f, 0.0f }};
	const vec3f gyro = {{ 0.0f, 0.0f, 0.0f }};
	posef init;
	float tilt[2];

	ovec3f_set(&init.pos, 0.0f, 1.0f, -1.0f);
	oquatf_set(&init.orient, 0.0f, 0.0f, 0.0f, 1.0f);

	for (int saturated = 0; saturated < 2; saturated++) {
		rift_fusion_ovr f;
		rift_fusion_ovr_init(&f, &init, 3);

		for (int i = 1; i <= 4000; i++)
			rift_fusion_ovr_imu_update(&f, (uint64_t)i * 1000000ULL, &gyro, &bogus,
				NULL, saturated != 0);

		posef out;
		vec3f vel, accel, ang_vel;
		rift_fusion_ovr_get_pose_at(&f, 4000000000ULL, &out, &vel, &accel, &ang_vel,
			NULL, NULL);

		/* how far the world-up axis has been dragged from vertical */
		const vec3f up = {{ 0.0f, 1.0f, 0.0f }};
		vec3f rotated;
		oquatf_get_rotated(&out.orient, &up, &rotated);
		tilt[saturated] = ovec3f_get_angle(&rotated, &up);

		rift_fusion_ovr_clear(&f);
	}

	/* unsaturated: the bogus reading is believed and drags the tilt right over */
	TAssert(tilt[0] > 1.0f);
	/* saturated: it is ignored, and the orientation stays put */
	TAssert(tilt[1] < 0.01f);
}

/* A non-finite IMU sample must not escape into the reported pose. */
void test_rift_kalman_rejects_non_finite_imu()
{
	rift_kalman_6dof_filter f;
	posef truth, pose;
	const vec3f good_accel = {{ 0.0f, (float)GRAVITY, 0.0f }};
	const vec3f gyro = {{ 0.0f, 0.0f, 0.0f }};
	const vec3f nan_accel = {{ 0.0f, (float)NAN, 0.0f }};

	ovec3f_set(&truth.pos, 0.0f, 1.0f, -1.0f);
	oquatf_set(&truth.orient, 0.0f, 0.0f, 0.0f, 1.0f);
	rift_kalman_6dof_init(&f, &truth, 3);

	uint64_t t = 0;
	for (int i = 1; i <= 200; i++) {
		t = (uint64_t)i * 1000000ULL;
		rift_kalman_6dof_imu_update(&f, t, &gyro, &good_accel, NULL, false);
	}

	for (int i = 201; i <= 210; i++) {
		t = (uint64_t)i * 1000000ULL;
		rift_kalman_6dof_imu_update(&f, t, &gyro, &nan_accel, NULL, false);
	}

	for (int i = 211; i <= 400; i++) {
		t = (uint64_t)i * 1000000ULL;
		rift_kalman_6dof_imu_update(&f, t, &gyro, &good_accel, NULL, false);
	}

	vec3f vel, accel_out, ang_vel, pos_err;
	rift_kalman_6dof_get_pose_at(&f, t, &pose, &vel, &accel_out, &ang_vel, &pos_err, NULL);

	TAssert(isfinite(pose.pos.x) && isfinite(pose.pos.y) && isfinite(pose.pos.z));
	TAssert(isfinite(pose.orient.x) && isfinite(pose.orient.y) &&
	        isfinite(pose.orient.z) && isfinite(pose.orient.w));
	TAssert(isfinite(vel.x) && isfinite(vel.y) && isfinite(vel.z));

	rift_kalman_6dof_clear(&f);
}
