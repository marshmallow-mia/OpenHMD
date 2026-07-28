/*
 * Timing health for the exposure/frame/pose pipeline - see rift-sync-monitor.h
 *
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#include <string.h>

#include "rift-sync-monitor.h"

/* Slow enough that a handful of odd frames cannot move the prediction, fast
 * enough to follow a genuine change in the pipeline within a second or two. */
#define LATENCY_EMA_ALPHA (1.0 / 64.0)

void rift_sync_monitor_init(rift_sync_monitor *m)
{
	memset(m, 0, sizeof(*m));
}

bool rift_sync_monitor_exposure(rift_sync_monitor *m, uint16_t count,
	uint64_t local_ts, int64_t *out_delta_ns)
{
	bool repeated = false;
	int64_t delta = 0;

	if (m->have_last_exposure) {
		delta = (int64_t)(local_ts - m->last_exposure_ts);
		if (count == m->last_count) {
			repeated = true;
			m->repeated_exposures++;
		} else if (rift_sync_monitor_exposure_gap(delta)) {
			m->dropped_exposures++;
		}
	}

	if (out_delta_ns != NULL)
		*out_delta_ns = delta;

	/* A repeat is not a new exposure, so it must not become the reference for
	 * the next gap measurement. */
	if (!repeated) {
		m->last_count = count;
		m->last_exposure_ts = local_ts;
		m->have_last_exposure = true;
	}

	return repeated;
}

bool rift_sync_monitor_exposure_gap(int64_t delta_ns)
{
	return delta_ns > (int64_t)(RIFT_SYNC_NOMINAL_EXPOSURE_NS * RIFT_SYNC_GAP_FACTOR);
}

bool rift_sync_monitor_frame(rift_sync_monitor *m, int64_t latency_ns,
	int64_t *out_predicted_ns)
{
	bool outlier = false;

	if (!m->have_latency) {
		m->latency_mean_ns = (double) latency_ns;
		m->have_latency = true;
		if (out_predicted_ns != NULL)
			*out_predicted_ns = latency_ns;
		return false;
	}

	const int64_t predicted = (int64_t) m->latency_mean_ns;
	int64_t diff = latency_ns - predicted;
	if (diff < 0)
		diff = -diff;

	if (diff > RIFT_SYNC_LATENCY_TOLERANCE_NS) {
		outlier = true;
		m->latency_outliers++;
	} else {
		/* Only well-behaved frames shape the prediction, so a burst of bad
		 * ones cannot drag the reference along with them and hide itself. */
		m->latency_mean_ns += LATENCY_EMA_ALPHA * ((double) latency_ns - m->latency_mean_ns);
	}

	if (out_predicted_ns != NULL)
		*out_predicted_ns = predicted;

	return outlier;
}

bool rift_sync_monitor_pose_age(rift_sync_monitor *m, uint64_t age_ns)
{
	if (age_ns > m->worst_pose_age_ns)
		m->worst_pose_age_ns = age_ns;

	if (age_ns > (uint64_t) RIFT_SYNC_LATE_POSE_NS) {
		m->late_poses++;
		return true;
	}
	return false;
}
