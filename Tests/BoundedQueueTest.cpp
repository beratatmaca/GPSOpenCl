#include "GPSOpenClBoundedQueue.h"

#include "gtest/gtest.h"
#include <atomic>
#include <thread>
#include <vector>

namespace GPSOpenClTest
{
TEST(BoundedQueueTest, PopReturnsItemsInFifoOrder)
{
    GPSOpenCl::BoundedQueue<int> queue(4);
    EXPECT_TRUE(queue.push(1));
    EXPECT_TRUE(queue.push(2));
    EXPECT_TRUE(queue.push(3));

    int item = 0;
    EXPECT_TRUE(queue.pop(item));
    EXPECT_EQ(item, 1);
    EXPECT_TRUE(queue.pop(item));
    EXPECT_EQ(item, 2);
    EXPECT_TRUE(queue.pop(item));
    EXPECT_EQ(item, 3);
}

TEST(BoundedQueueTest, TryPushDropsWhenFull)
{
    GPSOpenCl::BoundedQueue<int> queue(2);
    EXPECT_TRUE(queue.tryPush(1));
    EXPECT_TRUE(queue.tryPush(2));
    EXPECT_FALSE(queue.tryPush(3));

    int item = 0;
    EXPECT_TRUE(queue.tryPop(item));
    EXPECT_EQ(item, 1);
    EXPECT_TRUE(queue.tryPush(4));
    EXPECT_TRUE(queue.tryPop(item));
    EXPECT_EQ(item, 2);
    EXPECT_TRUE(queue.tryPop(item));
    EXPECT_EQ(item, 4);
}

TEST(BoundedQueueTest, TryPopReturnsFalseOnEmpty)
{
    GPSOpenCl::BoundedQueue<int> queue(2);
    int item = 0;
    EXPECT_FALSE(queue.tryPop(item));
}

TEST(BoundedQueueTest, PushBlocksUntilConsumerFreesASlot)
{
    GPSOpenCl::BoundedQueue<int> queue(1);
    EXPECT_TRUE(queue.push(1));

    std::atomic<bool> pushed{false};
    std::thread producer(
        [&]()
        {
            queue.push(2);
            pushed = true;
        });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(pushed.load());

    int item = 0;
    EXPECT_TRUE(queue.pop(item));
    EXPECT_EQ(item, 1);
    producer.join();
    EXPECT_TRUE(pushed.load());
    EXPECT_TRUE(queue.pop(item));
    EXPECT_EQ(item, 2);
}

TEST(BoundedQueueTest, FinishUnblocksAndDrainsRemainingItems)
{
    GPSOpenCl::BoundedQueue<int> queue(4);
    EXPECT_TRUE(queue.push(1));
    EXPECT_TRUE(queue.push(2));
    queue.finish();

    EXPECT_FALSE(queue.push(3));
    EXPECT_FALSE(queue.tryPush(3));

    int item = 0;
    EXPECT_TRUE(queue.pop(item));
    EXPECT_EQ(item, 1);
    EXPECT_TRUE(queue.pop(item));
    EXPECT_EQ(item, 2);
    EXPECT_FALSE(queue.pop(item));
}

TEST(BoundedQueueTest, FinishUnblocksAWaitingConsumer)
{
    GPSOpenCl::BoundedQueue<int> queue(2);
    std::atomic<bool> returned{false};
    std::thread consumer(
        [&]()
        {
            int item = 0;
            const bool got = queue.pop(item);
            EXPECT_FALSE(got);
            returned = true;
        });

    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    EXPECT_FALSE(returned.load());
    queue.finish();
    consumer.join();
    EXPECT_TRUE(returned.load());
}

TEST(BoundedQueueTest, ManyItemsThroughSmallRingKeepOrderAcrossWrap)
{
    GPSOpenCl::BoundedQueue<int> queue(3);
    std::vector<int> received;
    std::thread consumer(
        [&]()
        {
            int item = 0;
            while (queue.pop(item))
            {
                received.push_back(item);
            }
        });

    for (int i = 0; i < 100; i++)
    {
        EXPECT_TRUE(queue.push(i));
    }
    queue.finish();
    consumer.join();

    ASSERT_EQ(received.size(), 100u);
    for (int i = 0; i < 100; i++)
    {
        EXPECT_EQ(received[static_cast<size_t>(i)], i);
    }
}
}
