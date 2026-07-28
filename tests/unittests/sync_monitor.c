/*
 * Tests for the exposure/frame/pose timing monitor.
 * Distributed under the Boost 1.0 licence, see LICENSE for full text.
 */

#include <string.h>

#include "tests.h"
#include "drv_oculus_rift/rift-sync-monitor.h"

#define NOMINAL RIFT_SYNC_NOMINAL_EXPOSURE_NS

/* A healthy exposure stream must not trip anything. */
void test_rift_sync_monitor_clean_stream()
{
	rift_sync_monitor m;
	rift_sync_monitor_init(&m);

	uint64_t t = 1000000000ULL;
	for (int i = 0; i < 200; i++) {
		int64_t delta = 0;
		TAssert(!rift_sync_monitor_exposure(&m, (uint16_t)i, t, &delta));
		if (i > 0)
			TAssert(!rift_sync_monitor_exposure_gap(delta));
		t += NOMINAL;
	}

	TAssert(m.repeated_exposures == 0);
	TAssert(m.dropped_exposures == 0);
}

/* The HMD re-announcing the same exposure is a stutter, and must not be
 * mistaken for a fresh one - nor allowed to reset the gap reference. */
void test_rift_sync_monitor_repeated_exposure()
{
	rift_sync_monitor m;
	rift_sync_monitor_init(&m);

	uint64_t t = 1000000000ULL;
	rift_sync_monitor_exposure(&m, 7, t, NULL);
	t += NOMINAL;

	TAssert(rift_sync_monitor_exposure(&m, 7, t, NULL));   /* same count */
	TAssert(m.repeated_exposures == 1);

	/* the repeat did not become the reference, so the next real exposure is
	 * still one nominal period away and must not read as a drop */
	int64_t delta = 0;
	TAssert(!rift_sync_monitor_exposure(&m, 8, t, &delta));
	TAssert(delta == NOMINAL);
	TAssert(m.dropped_exposures == 0);
}

/* A missing exposure shows up as a gap. */
void test_rift_sync_monitor_dropped_exposure()
{
	rift_sync_monitor m;
	rift_sync_monitor_init(&m);

	uint64_t t = 1000000000ULL;
	rift_sync_monitor_exposure(&m, 1, t, NULL);
	t += 3 * NOMINAL;                                       /* two went missing */

	int64_t delta = 0;
	rift_sync_monitor_exposure(&m, 4, t, &delta);
	TAssert(rift_sync_monitor_exposure_gap(delta));
	TAssert(m.dropped_exposures == 1);
}

/* Frame latency: a steady stream trains the prediction and stays quiet; a
 * frame that arrives far off is reported, and - importantly - a run of bad
 * frames must not drag the prediction along and silence itself. */
void test_rift_sync_monitor_latency_outlier()
{
	rift_sync_monitor m;
    int64_t predicted = 0;
	rift_sync_monitor_init(&m);

	for (int i = 0; i < 300; i++)
		TAssert(!rift_sync_monitor_frame(&m, 8000000LL, &predicted));   /* 8 ms */

	TAssert(m.latency_outliers == 0);
	TAssert(predicted > 7500000LL && predicted < 8500000LL);

	/* 20 ms when the stream has been doing 8 ms */
	TAssert(rift_sync_monitor_frame(&m, 20000000LL, &predicted));
	TAssert(predicted > 7500000LL && predicted < 8500000LL);

	for (int i = 0; i < 100; i++)
		rift_sync_monitor_frame(&m, 20000000LL, NULL);

	/* every one of them still reported: the reference did not follow */
	TAssert(m.latency_outliers == 101);
}

/* Pose age: ordinary fixes are quiet, a stale one is not. */
void test_rift_sync_monitor_late_pose()
{
	rift_sync_monitor m;
	rift_sync_monitor_init(&m);

	TAssert(!rift_sync_monitor_pose_age(&m, 20000000ULL));   /* 20 ms */
	TAssert(m.late_poses == 0);

	TAssert(rift_sync_monitor_pose_age(&m, 80000000ULL));    /* 80 ms */
	TAssert(m.late_poses == 1);
	TAssert(m.worst_pose_age_ns == 80000000ULL);
}
