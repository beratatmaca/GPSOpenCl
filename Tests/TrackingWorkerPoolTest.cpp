#include "Tracking/GPSOpenClTrackingWorkerPool.hpp"

#include <gtest/gtest.h>

#include <atomic>
#include <vector>

namespace
{
TEST(TrackingWorkerPoolTest, ProcessesEveryItemExactlyOnce)
{
    std::vector<std::atomic<int>> counts(32);
    GPSOpenCl::TrackingWorkerPool pool([&](int item) { counts[static_cast<size_t>(item)]++; }, 32);

    pool.run(32);

    for (const auto &count : counts)
    {
        EXPECT_EQ(count.load(), 1);
    }
}

// Varying small item counts across many runs exercises the wakeup and participation-claim
// paths that a lost notification would deadlock
TEST(TrackingWorkerPoolTest, RepeatedRunsWithVaryingItemCountsComplete)
{
    std::vector<std::atomic<int>> counts(8);
    GPSOpenCl::TrackingWorkerPool pool([&](int item) { counts[static_cast<size_t>(item)]++; }, 8);

    int expectedTotal = 0;
    for (int run = 0; run < 2000; run++)
    {
        const int itemCount = (run % 8) + 1;
        pool.run(itemCount);
        expectedTotal += itemCount;
    }

    int total = 0;
    for (const auto &count : counts)
    {
        total += count.load();
    }
    EXPECT_EQ(total, expectedTotal);
}

TEST(TrackingWorkerPoolTest, ZeroAndSingleItemRunsExecuteInline)
{
    std::atomic<int> total{0};
    GPSOpenCl::TrackingWorkerPool pool([&](int) { total++; }, 4);

    pool.run(0);
    EXPECT_EQ(total.load(), 0);

    pool.run(1);
    EXPECT_EQ(total.load(), 1);
    EXPECT_GE(pool.maxWorkerDurationMs(), 0.0);
}
}
