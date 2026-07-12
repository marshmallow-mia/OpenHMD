/*
 * openhmd_pose_log — high-rate CSV pose logger for prediction analysis.
 *
 * Dumps pose + velocity + angular velocity + acceleration + pose age at the
 * fusion's own rate, which is what tools/score_prediction.py needs in order to
 * score forward prediction offline: predict each sample forward by a horizon
 * using the velocities logged with it, then compare against the pose actually
 * observed a horizon later in the same log.
 *
 * Deliberately separate from openhmd_simple_example so the existing stationary
 * test harness (which parses that example's stdout at its own 100 Hz cadence)
 * keeps behaving exactly as before.
 *
 * Usage: openhmd_pose_log <out.csv> [seconds] [hz]
 *
 * The CSV goes to a file, not stdout: OpenHMD's own logging writes to stdout and
 * would interleave itself into the rows.
 *
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <openhmd.h>

void ohmd_sleep(double);

static double now_s(void)
{
	struct timespec ts;
	clock_gettime(CLOCK_MONOTONIC, &ts);
	return (double) ts.tv_sec + (double) ts.tv_nsec * 1e-9;
}

int main(int argc, char **argv)
{
	if (argc < 2) {
		fprintf(stderr, "usage: %s <out.csv> [seconds] [hz]\n", argv[0]);
		return 1;
	}

	double duration = (argc > 2) ? atof(argv[2]) : 60.0;
	double hz = (argc > 3) ? atof(argv[3]) : 250.0;
	double period = 1.0 / hz;

	FILE *out = fopen(argv[1], "w");
	if (!out) {
		fprintf(stderr, "cannot write %s\n", argv[1]);
		return 1;
	}

	ohmd_context *ctx = ohmd_ctx_create();
	int num = ohmd_ctx_probe(ctx);
	if (num < 0) {
		fprintf(stderr, "probe failed: %s\n", ohmd_ctx_get_error(ctx));
		return 1;
	}

	ohmd_device_settings *settings = ohmd_device_settings_create(ctx);
	int auto_update = 0; /* we drive ohmd_ctx_update() ourselves */
	ohmd_device_settings_seti(settings, OHMD_IDS_AUTOMATIC_UPDATE, &auto_update);

	ohmd_device *hmd = ohmd_list_open_device_s(ctx, 0, settings);
	if (!hmd) {
		fprintf(stderr, "open failed: %s\n", ohmd_ctx_get_error(ctx));
		return 1;
	}

	fprintf(out, "t,px,py,pz,qx,qy,qz,qw,vx,vy,vz,wx,wy,wz,ax,ay,az,age\n");

	double t0 = now_s();
	double next = t0;

	while (now_s() - t0 < duration) {
		ohmd_ctx_update(ctx);

		float q[4] = {0}, p[3] = {0}, v[3] = {0}, w[3] = {0}, a[3] = {0}, age = 0.0f;
		ohmd_device_getf(hmd, OHMD_ROTATION_QUAT, q);
		ohmd_device_getf(hmd, OHMD_POSITION_VECTOR, p);
		ohmd_device_getf(hmd, OHMD_VELOCITY_VECTOR, v);
		ohmd_device_getf(hmd, OHMD_ANGULAR_VELOCITY_VECTOR, w);
		ohmd_device_getf(hmd, OHMD_ACCELERATION_VECTOR, a);
		ohmd_device_getf(hmd, OHMD_POSE_AGE_SECONDS, &age);

		fprintf(out, "%.6f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.7f,"
		       "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
		       now_s() - t0, p[0], p[1], p[2], q[0], q[1], q[2], q[3],
		       v[0], v[1], v[2], w[0], w[1], w[2], a[0], a[1], a[2], age);

		next += period;
		double sleep_s = next - now_s();
		if (sleep_s > 0.0)
			ohmd_sleep(sleep_s);
		else
			next = now_s(); /* fell behind; don't spiral */
	}

	fclose(out);
	ohmd_device_settings_destroy(settings);
	ohmd_ctx_destroy(ctx);
	return 0;
}
