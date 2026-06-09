#pragma once

#include <queue>
#include <mutex>
#include <condition_variable>

/**
 * Thread-safe blocking queue with back-pressure and close semantics.
 *
 * - push() blocks when the queue is full; no-op after close().
 * - pop()  blocks until an item is available or the queue is closed.
 *          Returns false when closed AND empty (signals consumers to stop).
 * - close() unblocks all waiting callers and prevents future pushes.
 */
template<typename T>
class FrameQueue {
public:
    explicit FrameQueue(size_t max_size = 32) : m_max(max_size) {}

    void push(T item) {
        std::unique_lock<std::mutex> lk(m_mtx);
        m_cv.wait(lk, [this]{ return m_q.size() < m_max || m_closed; });
        if (m_closed) return;
        m_q.push(std::move(item));
        lk.unlock();
        m_cv.notify_one();
    }

    bool pop(T& out) {
        std::unique_lock<std::mutex> lk(m_mtx);
        m_cv.wait(lk, [this]{ return !m_q.empty() || m_closed; });
        if (m_q.empty()) return false;
        out = std::move(m_q.front());
        m_q.pop();
        lk.unlock();
        m_cv.notify_one();
        return true;
    }

    void close() {
        {
            std::lock_guard<std::mutex> lk(m_mtx);
            m_closed = true;
        }
        m_cv.notify_all();
    }

    size_t size() const {
        std::lock_guard<std::mutex> lk(m_mtx);
        return m_q.size();
    }

    bool empty() const {
        std::lock_guard<std::mutex> lk(m_mtx);
        return m_q.empty();
    }

    bool is_closed() const {
        std::lock_guard<std::mutex> lk(m_mtx);
        return m_closed;
    }

private:
    mutable std::mutex      m_mtx;
    std::condition_variable m_cv;
    std::queue<T>           m_q;
    size_t                  m_max;
    bool                    m_closed = false;
};
