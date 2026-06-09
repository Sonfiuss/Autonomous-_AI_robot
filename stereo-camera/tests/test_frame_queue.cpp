/**
 * Unit tests for FrameQueue<T>
 *
 * Run: ./test_frame_queue
 * Expected: all tests PASS, exit code 0.
 */

#include <cassert>
#include <cstdio>
#include <thread>
#include <vector>
#include <chrono>
#include <atomic>

#include "../pipeline/FrameQueue.hpp"

// ── Minimal test harness ──────────────────────────────────────────────────────

static int s_pass = 0;
static int s_fail = 0;

#define CHECK(expr) do { \
    if (expr) { \
        printf("  PASS  %s\n", #expr); \
        ++s_pass; \
    } else { \
        printf("  FAIL  %s  (line %d)\n", #expr, __LINE__); \
        ++s_fail; \
    } \
} while(0)

static void section(const char* name) {
    printf("\n── %s\n", name);
}

// ── Tests ─────────────────────────────────────────────────────────────────────

static void test_fifo_ordering() {
    section("FIFO ordering");
    FrameQueue<int> q(8);

    std::thread producer([&]{
        for (int i = 0; i < 5; ++i) q.push(i);
        q.close();
    });

    int v;
    for (int expected = 0; expected < 5; ++expected) {
        CHECK(q.pop(v) == true);
        CHECK(v == expected);
    }
    CHECK(q.pop(v) == false);   // closed and empty

    producer.join();
}

static void test_pop_returns_false_when_closed_empty() {
    section("pop returns false when closed and empty");
    FrameQueue<int> q;
    q.close();
    int v;
    CHECK(q.pop(v) == false);
}

static void test_push_after_close_is_noop() {
    section("push after close is no-op");
    FrameQueue<int> q(4);
    q.close();
    q.push(99);   // must not block or throw
    CHECK(q.empty() == true);
}

static void test_close_unblocks_waiting_pop() {
    section("close() unblocks waiting pop()");
    FrameQueue<int> q;
    std::atomic<bool> returned{false};

    std::thread consumer([&]{
        int v;
        q.pop(v);        // blocks here
        returned = true;
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK(returned == false);   // still blocking
    q.close();
    consumer.join();
    CHECK(returned == true);    // unblocked by close()
}

static void test_backpressure_blocks_producer() {
    section("backpressure: push blocks when queue full");
    FrameQueue<int> q(2);   // max 2 items
    std::atomic<int> pushed{0};

    std::thread producer([&]{
        q.push(1); ++pushed;   // fills slot 1
        q.push(2); ++pushed;   // fills slot 2
        q.push(3); ++pushed;   // should block until consumer pops
    });

    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    CHECK(pushed == 2);   // third push is blocked

    int v;
    q.pop(v);   // free one slot
    std::this_thread::sleep_for(std::chrono::milliseconds(20));
    CHECK(pushed == 3);   // producer unblocked

    q.close();
    producer.join();
}

static void test_multiple_producers_consumers() {
    section("multiple producers and consumers");
    FrameQueue<int> q(64);
    constexpr int N = 100;
    std::atomic<int> consumed_sum{0};

    std::vector<std::thread> producers;
    for (int t = 0; t < 4; ++t)
        producers.emplace_back([&, t]{
            for (int i = 0; i < N / 4; ++i)
                q.push(1);   // push 1s so we can count
        });

    std::vector<std::thread> consumers;
    for (int t = 0; t < 2; ++t)
        consumers.emplace_back([&]{
            int v;
            while (q.pop(v))
                consumed_sum += v;
        });

    for (auto& p : producers) p.join();
    q.close();
    for (auto& c : consumers) c.join();

    CHECK(consumed_sum == N);
}

static void test_size_and_empty() {
    section("size() and empty()");
    FrameQueue<int> q(8);
    CHECK(q.empty() == true);
    CHECK(q.size()  == 0u);

    q.push(1); q.push(2);
    CHECK(q.size()  == 2u);
    CHECK(q.empty() == false);

    q.close();
    int v;
    q.pop(v); q.pop(v);
    CHECK(q.empty() == true);
}

// ── main ──────────────────────────────────────────────────────────────────────

int main() {
    printf("=== test_frame_queue ===\n");

    test_fifo_ordering();
    test_pop_returns_false_when_closed_empty();
    test_push_after_close_is_noop();
    test_close_unblocks_waiting_pop();
    test_backpressure_blocks_producer();
    test_multiple_producers_consumers();
    test_size_and_empty();

    printf("\n  %d passed  /  %d failed\n", s_pass, s_fail);
    return s_fail > 0 ? 1 : 0;
}
