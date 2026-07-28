// Copyright 2020, Jan Schmidt <thaytan@noraisin.net>
// SPDX-License-Identifier: BSL-1.0
/*
 * OpenHMD - Free and Open Source API and drivers for immersive technology.
 *
 * 6DOF Unscented Kalman Filter for positional tracking
 *
 */
#ifndef RIFT_KALMAN_6DOF_H
#define RIFT_KALMAN_6DOF_H

#include "rift.h"
#include "../ukf.h"


/* Maximum Number of extra lagged state slots to allow for */
#define MAX_DELAY_SLOTS 5

typedef struct rift_kalman_6dof_filter rift_kalman_6dof_filter;

struct rift_kalman_6dof_filter {
  /* 19 element state vector, 18 covariance dimensions (the quaternion costs
   * 4 state entries but only 3 of covariance, parameterised by an exponential
   * map). Authoritative layout is the STATE_ and COV_ defines in the .c.
   *
   * Inertial frame:
   *  quatd orientation (quaternion) (0:3)
   *  vec3d position (4:6)
   *  vec3d velocity (7:9)
   *  vec3d accel (10:12)
   *
   * Body frame:
   *  vec3d accel-bias (13:15)
   *  vec3d gyro-bias (16:18)
   *
   *  + N lagged slots, each 7 state / 6 covariance:
   *    quatd orientation (0:3)
   *    vec3d position (4:6)
   *
   * Angular velocity is NOT a state - it is the control input, taken straight
   * from the gyro (see `ang_vel` below and process_func in the .c). An earlier
   * version of this comment listed it as states 13:15 and put the biases at
   * 16:21, which never matched the code. */
  /* ukf_base needs to be the first element in the struct */
  ukf_base ukf;

  /* Current time tracking */
  bool first_update;
  uint64_t current_ts;
  bool saw_pose_update;

  /* Control vector (gyro reading) */
  vec3d ang_vel;

  /* Process noise */
  matrix2d *Q_noise;

  /* Measurment 1: IMU accel (gyro is in the input vector)
   *   vec3f accel
   */
  ukf_measurement m1;

  /* Measurment 2: Pose
   *  vec3f position
   *  quatd orientation
   */
  ukf_measurement m2;

  /* Measurment 3: Position
   *  vec3f position
   */
  ukf_measurement m_position;

  int pose_slot; /* slot number to use for pose update */
  int reset_slot; /* slot number to reset during process_func if != -1 */

  int num_delay_slots;
  bool slot_inuse[MAX_DELAY_SLOTS];
};

void rift_kalman_6dof_init(rift_kalman_6dof_filter *state, posef *init_pose, int num_delay_slots);
void rift_kalman_6dof_clear(rift_kalman_6dof_filter *state);

void rift_kalman_6dof_prepare_delay_slot(rift_kalman_6dof_filter *state, uint64_t time, int delay_slot);
void rift_kalman_6dof_release_delay_slot(rift_kalman_6dof_filter *state, int delay_slot);

/* accel_saturated: the accelerometer clipped; its measurement noise is
 * inflated for this sample instead of being taken at face value. */
void rift_kalman_6dof_imu_update (rift_kalman_6dof_filter *state, uint64_t time, const vec3f* ang_vel, const vec3f* accel, const vec3f* mag_field, bool accel_saturated);
/* obs_scale scales the observation error std-dev: 1.0 = fully trusted
 * (1cm position error), larger = weaker correction */
void rift_kalman_6dof_pose_update(rift_kalman_6dof_filter *state, uint64_t time, posef *pose, int delay_slot, float obs_scale);
void rift_kalman_6dof_position_update(rift_kalman_6dof_filter *state, uint64_t time, vec3f *position, int delay_slot, float obs_scale);

void rift_kalman_6dof_get_delay_slot_pose_at(rift_kalman_6dof_filter *state, uint64_t time, int delay_slot, posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error);
void rift_kalman_6dof_get_pose_at(rift_kalman_6dof_filter *state, uint64_t time, posef *pose, vec3f *vel, vec3f *accel, vec3f *ang_vel, vec3f *pos_error, vec3f *rot_error);

#endif
