/*
 * Timing health for the exposure/frame/pose pipeline.
 *
 * The two most expensive bugs in this driver's history were both timing: the
 * ~25 ms Touch radio IMU latency, and the exposure-to-device-clock mapping.
 * Neither announced itself - they were found by capturing and scoring offline.
 * The Oculus runtime instead checks its own timing continuously and says when
 * it breaks:
 *
 *   "%s: predicted latency of %.1f ms differed from measured %.1f ms. Check sync cable."
 *   "%s: out of sync. Exposure Delta = %.1f ms, cameraDelta = %.1f ms."
 *   "%s: Repeated exposure time: %.4f %.4f"
 *   "%d: Late Pose time: %.3f ms"
 *
 * This is the same four checks. Kept free of driver types so the logic can be
 * unit-tested directly.
 *
 * Copyright 2026
 * SPDX-License-Identifier: BSL-1.0
 */
#ifndef RIFT_SYNC_MONITOR_H
#define RIFT_SYNC_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

/* CV1 exposures are ~19.2 ms apart (52 Hz). */
#define RIFT_SYNC_NOMINAL_EXPOSURE_NS 19200000LL
/* A gap this far from nominal means an exposure went missing. */
#define RIFT_SYNC_GAP_FACTOR 1.5
/* How far a frame's arrival latency may stray from the running average before
 * it is worth reporting. A quarter of the exposure period: beyond that a frame
 * is closer to its neighbour than to its own exposure. */
#define RIFT_SYNC_LATENCY_TOLERANCE_NS 5000000LL
/* A vision fix older than this has missed its usefulness for the current pose. */
#define RIFT_SYNC_LATE_POSE_NS 50000000LL

typedef struct {
	/* exposure stream */
	bool have_last_exposure;
	uint16_t last_count;
	uint64_t last_exposure_ts;

	/* exposure -> frame arrival latency, as a slow running mean */
	bool have_latency;
	double latency_mean_ns;

	/* counters, for a periodic summary rather than a line per event */
	uint32_t repeated_exposures;
	uint32_t dropped_exposures;
	uint32_t latency_outliers;
	uint32_t late_poses;
	uint64_t worst_pose_age_ns;
} rift_sync_monitor;

void rift_sync_monitor_init(rift_sync_monitor *m);

/* An exposure was announced. `out_delta_ns` receives the gap since the previous
 * one. Returns true if this is a REPEAT of the exposure already seen (same
 * count), which means the HMD's exposure stream is stuttering. */
bool rift_sync_monitor_exposure(rift_sync_monitor *m, uint16_t count,
	uint64_t local_ts, int64_t *out_delta_ns);

/* True if the gap recorded by the last rift_sync_monitor_exposure() call looks
 * like a dropped exposure rather than a normal cadence. */
bool rift_sync_monitor_exposure_gap(int64_t delta_ns);

/* A camera frame arrived `latency_ns` after its matched exposure. Returns true
 * if that is far from what the stream has been doing, and reports what was
 * expected in `out_predicted_ns`. */
bool rift_sync_monitor_frame(rift_sync_monitor *m, int64_t latency_ns,
	int64_t *out_predicted_ns);

/* A vision fix landed `age_ns` after its exposure. Returns true if that is
 * late enough to matter. */
bool rift_sync_monitor_pose_age(rift_sync_monitor *m, uint64_t age_ns);

#endif /* RIFT_SYNC_MONITOR_H */
