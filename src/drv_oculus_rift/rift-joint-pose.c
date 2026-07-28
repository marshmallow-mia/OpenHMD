/*
 * Joint multi-camera pose reconstruction - see rift-joint-pose.h.
 *
 * Gauss-Newton on the 6-DoF device pose with a Huber loss, analytic Jacobian,
 * no external solver. The perturbation is applied in the world frame:
 *
 *   R <- Exp(dtheta) R,   t <- t + dt
 *
 * so for a model point p (device frame), with camera pose (Rc, tc):
 *
 *   Xw = R p + t                    d Xw / d dtheta = -[R p]_x
 *   Xc = Rc^T (Xw - tc)             d Xw / d dt     = I
 *   r  = (Xc.x/Xc.z, Xc.y/Xc.z) - ray
 *
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#include <math.h>
#include <string.h>

#include "rift-joint-pose.h"

#define MAX_ITERATIONS 12
#define MIN_POINTS_TOTAL 6
#define MIN_POINTS_PER_VIEW 3
#define CONVERGE_TRANS 1e-6f   /* metres */
#define CONVERGE_ROT 1e-6f     /* radians */
#define HUBER_PX 3.0f          /* residuals beyond this are down-weighted */
#define MIN_DEPTH 0.05f        /* metres; behind/at the camera is nonsense */

/* Cholesky solve of a symmetric positive-definite 6x6 system H x = g.
 * Returns false if H is not positive definite (degenerate geometry). */
static bool solve6(const double H[6][6], const double g[6], double x[6])
{
	double L[6][6];
	memset(L, 0, sizeof(L));

	for (int i = 0; i < 6; i++) {
		for (int j = 0; j <= i; j++) {
			double sum = H[i][j];
			for (int k = 0; k < j; k++)
				sum -= L[i][k] * L[j][k];
			if (i == j) {
				if (sum <= 1e-18)
					return false;
				L[i][i] = sqrt(sum);
			} else {
				L[i][j] = sum / L[j][j];
			}
		}
	}

	double y[6];
	for (int i = 0; i < 6; i++) {
		double sum = g[i];
		for (int k = 0; k < i; k++)
			sum -= L[i][k] * y[k];
		y[i] = sum / L[i][i];
	}
	for (int i = 5; i >= 0; i--) {
		double sum = y[i];
		for (int k = i + 1; k < 6; k++)
			sum -= L[k][i] * x[k];
		x[i] = sum / L[i][i];
	}
	return true;
}

/* World -> camera rotation as a 3x3, from the camera->world pose */
static void camera_rot_inv(const posef *camera_pose, float Rct[3][3])
{
	quatf inv = camera_pose->orient;
	oquatf_inverse(&inv);

	const vec3f ex = {{ 1, 0, 0 }}, ey = {{ 0, 1, 0 }}, ez = {{ 0, 0, 1 }};
	vec3f c0, c1, c2;
	oquatf_get_rotated(&inv, &ex, &c0);
	oquatf_get_rotated(&inv, &ey, &c1);
	oquatf_get_rotated(&inv, &ez, &c2);

	Rct[0][0] = c0.x; Rct[1][0] = c0.y; Rct[2][0] = c0.z;
	Rct[0][1] = c1.x; Rct[1][1] = c1.y; Rct[2][1] = c1.z;
	Rct[0][2] = c2.x; Rct[1][2] = c2.y; Rct[2][2] = c2.z;
}

static void mat3_mul_vec(const float M[3][3], const vec3f *v, vec3f *out)
{
	out->x = M[0][0] * v->x + M[0][1] * v->y + M[0][2] * v->z;
	out->y = M[1][0] * v->x + M[1][1] * v->y + M[1][2] * v->z;
	out->z = M[2][0] * v->x + M[2][1] * v->y + M[2][2] * v->z;
}

float rift_joint_pose_rms_px(const vec3f *leds, int num_leds,
	const rift_joint_view *view, const posef *pose)
{
	float Rct[3][3];
	camera_rot_inv(&view->camera_pose, Rct);

	double sum = 0.0;
	int n = 0;

	for (int i = 0; i < view->n_points; i++) {
		const rift_joint_point *pt = view->points + i;
		if (pt->led_index >= num_leds)
			continue;

		vec3f rotated, world, rel, cam;
		oquatf_get_rotated(&pose->orient, &leds[pt->led_index], &rotated);
		ovec3f_add(&rotated, &pose->pos, &world);
		ovec3f_subtract(&world, &view->camera_pose.pos, &rel);
		mat3_mul_vec(Rct, &rel, &cam);
		if (cam.z < MIN_DEPTH)
			continue;

		float dx = cam.x / cam.z - pt->ray[0];
		float dy = cam.y / cam.z - pt->ray[1];
		sum += (double)(dx * dx + dy * dy);
		n++;
	}

	if (n == 0)
		return INFINITY;
	return view->focal_px * (float)sqrt(sum / n);
}

bool rift_joint_pose_solve(const vec3f *leds, int num_leds,
	const rift_joint_view *views, int n_views,
	const posef *init_pose, posef *out_pose, rift_joint_result *out_result)
{
	if (leds == NULL || views == NULL || n_views < 1 || init_pose == NULL)
		return false;

	int usable_views = 0, total_points = 0;
	for (int v = 0; v < n_views; v++) {
		int n = 0;
		for (int i = 0; i < views[v].n_points; i++) {
			if (views[v].points[i].led_index < num_leds)
				n++;
		}
		if (n >= MIN_POINTS_PER_VIEW) {
			usable_views++;
			total_points += n;
		}
	}
	if (usable_views < 1 || total_points < MIN_POINTS_TOTAL)
		return false;

	/* World->camera rotations are constant across iterations */
	float Rct[RIFT_MAX_SENSORS_JOINT][3][3];
	if (n_views > RIFT_MAX_SENSORS_JOINT)
		n_views = RIFT_MAX_SENSORS_JOINT;
	for (int v = 0; v < n_views; v++)
		camera_rot_inv(&views[v].camera_pose, Rct[v]);

	posef pose = *init_pose;
	oquatf_normalize_me(&pose.orient);

	int iter = 0;
	for (; iter < MAX_ITERATIONS; iter++) {
		double H[6][6];
		double g[6];
		memset(H, 0, sizeof(H));
		memset(g, 0, sizeof(g));
		int used = 0;

		for (int v = 0; v < n_views; v++) {
			const rift_joint_view *view = views + v;
			const float huber = HUBER_PX / (view->focal_px > 1.0f ? view->focal_px : 1.0f);

			for (int i = 0; i < view->n_points; i++) {
				const rift_joint_point *pt = view->points + i;
				if (pt->led_index >= num_leds)
					continue;

				vec3f rotated, world, rel, cam;
				oquatf_get_rotated(&pose.orient, &leds[pt->led_index], &rotated);
				ovec3f_add(&rotated, &pose.pos, &world);
				ovec3f_subtract(&world, &view->camera_pose.pos, &rel);
				mat3_mul_vec(Rct[v], &rel, &cam);
				if (cam.z < MIN_DEPTH)
					continue;

				const double inv_z = 1.0 / cam.z;
				const double rx = cam.x * inv_z - pt->ray[0];
				const double ry = cam.y * inv_z - pt->ray[1];

				/* d ray / d Xc */
				const double dr[2][3] = {
					{ inv_z, 0.0, -cam.x * inv_z * inv_z },
					{ 0.0, inv_z, -cam.y * inv_z * inv_z },
				};

				/* d Xc / d (dtheta, dt) = Rct * ( -[rotated]_x | I ) */
				double dXc[3][6];
				for (int r = 0; r < 3; r++) {
					/* -[rotated]_x columns */
					const double sk[3][3] = {
						{ 0.0, rotated.z, -rotated.y },
						{ -rotated.z, 0.0, rotated.x },
						{ rotated.y, -rotated.x, 0.0 },
					};
					for (int c = 0; c < 3; c++) {
						double acc = 0.0;
						for (int k = 0; k < 3; k++)
							acc += Rct[v][r][k] * sk[k][c];
						dXc[r][c] = acc;
						dXc[r][c + 3] = Rct[v][r][c];
					}
				}

				double J[2][6];
				for (int row = 0; row < 2; row++) {
					for (int c = 0; c < 6; c++) {
						double acc = 0.0;
						for (int k = 0; k < 3; k++)
							acc += dr[row][k] * dXc[k][c];
						J[row][c] = acc;
					}
				}

				/* Huber weight on the residual magnitude */
				const double rn = sqrt(rx * rx + ry * ry);
				double w = 1.0;
				if (rn > huber && rn > 0.0)
					w = huber / rn;

				const double res[2] = { rx, ry };
				for (int row = 0; row < 2; row++) {
					for (int a = 0; a < 6; a++) {
						g[a] -= w * J[row][a] * res[row];
						for (int b = 0; b <= a; b++)
							H[a][b] += w * J[row][a] * J[row][b];
					}
				}
				used++;
			}
		}

		if (used < MIN_POINTS_TOTAL)
			return false;

		for (int a = 0; a < 6; a++)
			for (int b = a + 1; b < 6; b++)
				H[a][b] = H[b][a];

		/* Small Levenberg damping keeps the normal equations conditioned
		 * when a device is seen nearly edge-on by every camera. */
		for (int a = 0; a < 6; a++)
			H[a][a] *= 1.0 + 1e-6;

		double step[6];
		if (!solve6(H, g, step))
			return false;

		for (int a = 0; a < 6; a++) {
			if (!isfinite(step[a]))
				return false;
		}

		vec3f dtheta = {{ (float)step[0], (float)step[1], (float)step[2] }};
		vec3f dt = {{ (float)step[3], (float)step[4], (float)step[5] }};

		quatf dq;
		oquatf_from_rotation(&dq, &dtheta);
		quatf updated;
		oquatf_mult(&dq, &pose.orient, &updated);
		oquatf_normalize_me(&updated);
		pose.orient = updated;
		ovec3f_add(&pose.pos, &dt, &pose.pos);

		if (ovec3f_get_length(&dt) < CONVERGE_TRANS &&
		    ovec3f_get_length(&dtheta) < CONVERGE_ROT) {
			iter++;
			break;
		}
	}

	if (!isfinite(pose.pos.x) || !isfinite(pose.pos.y) || !isfinite(pose.pos.z))
		return false;

	if (out_result) {
		double sum_sq = 0.0;
		int n_tot = 0;
		float worst = 0.0f;
		int n_scored = 0;
		for (int v = 0; v < n_views; v++) {
			float rms = rift_joint_pose_rms_px(leds, num_leds, views + v, &pose);
			if (!isfinite(rms))
				continue;
			if (rms > worst)
				worst = rms;
			sum_sq += (double)rms * rms * views[v].n_points;
			n_tot += views[v].n_points;
			n_scored++;
		}
		out_result->rms_px = n_tot ? (float)sqrt(sum_sq / n_tot) : INFINITY;
		out_result->worst_view_px = n_scored ? worst : INFINITY;
		out_result->n_points = total_points;
		out_result->n_views = usable_views;
		out_result->iterations = iter;
	}

	*out_pose = pose;
	return true;
}
