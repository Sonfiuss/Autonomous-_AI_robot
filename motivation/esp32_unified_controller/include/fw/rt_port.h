// Thin wrappers over the FreeRTOS primitives the four tasks share, with a
// single-threaded stand-in for host builds so the pure logic can be unit
// tested on a PC (-DFW_HOST_BUILD). Both halves expose the same API and the
// same semantics; only the implementation differs.
//
// The semantics are the whole design, so they are stated once, here:
//   Mailbox  overwrite-on-post, peek-without-remove. For streaming commands
//            where a stale value is worthless and only the newest matters.
//   Queue    bounded FIFO, push NEVER blocks. A full queue fails the push so
//            the caller counts a drop instead of stalling the UART reader.
//   Flag     raised by one task, consumed by another, never waited on.
//   Snapshot one value, one short mutex, written by its owner and read by Status.
#ifndef FW_RT_PORT_H
#define FW_RT_PORT_H

#include <atomic>
#include <cstdint>

#ifdef FW_HOST_BUILD
#include <chrono>
#else
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#endif

namespace fw {

// Milliseconds since boot.
inline uint32_t nowMs() {
#ifdef FW_HOST_BUILD
    using Clock = std::chrono::steady_clock;
    static const Clock::time_point start = Clock::now();
    const auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now() - start);
    return static_cast<uint32_t>(elapsed.count());
#else
    return static_cast<uint32_t>(xTaskGetTickCount()) * portTICK_PERIOD_MS;
#endif
}

// The e-stop path: Comm raises it, Motion consumes it at the top of a tick.
// Atomic on both platforms, so it needs no separate implementation.
class Flag {
public:
    Flag() : raised_(false) {}

    void set() { raised_.store(true); }
    bool isSet() const { return raised_.load(); }

    // True exactly once per set().
    bool testAndClear() { return raised_.exchange(false); }

private:
    std::atomic<bool> raised_;
};

#ifdef FW_HOST_BUILD

template <typename T>
class Mailbox {
public:
    Mailbox() : value_(), full_(false) {}

    bool create() { return true; }

    void post(const T& value) {
        value_ = value;
        full_  = true;
    }

    bool peek(T* out) const {
        if (!full_ || out == nullptr) {
            return false;
        }
        *out = value_;
        return true;
    }

private:
    T    value_;
    bool full_;
};

template <typename T, int CAPACITY>
class Queue {
public:
    Queue() : items_(), head_(0), count_(0) {}

    bool create() { return true; }

    bool push(const T& value) {
        if (count_ >= CAPACITY) {
            return false;
        }
        items_[(head_ + count_) % CAPACITY] = value;
        ++count_;
        return true;
    }

    bool pop(T* out) {
        if (count_ == 0 || out == nullptr) {
            return false;
        }
        *out  = items_[head_];
        head_ = (head_ + 1) % CAPACITY;
        --count_;
        return true;
    }

private:
    T   items_[CAPACITY];
    int head_;
    int count_;
};

template <typename T>
class Snapshot {
public:
    Snapshot() : value_() {}

    bool create() { return true; }
    void set(const T& value) { value_ = value; }
    T    get() const { return value_; }

private:
    T value_;
};

#else  // FreeRTOS

template <typename T>
class Mailbox {
public:
    Mailbox() : handle_(nullptr) {}

    bool create() {
        handle_ = xQueueCreate(1, sizeof(T));
        return handle_ != nullptr;
    }

    void post(const T& value) {
        if (handle_ != nullptr) {
            xQueueOverwrite(handle_, &value);
        }
    }

    bool peek(T* out) const {
        if (handle_ == nullptr || out == nullptr) {
            return false;
        }
        return xQueuePeek(handle_, out, 0) == pdTRUE;
    }

private:
    QueueHandle_t handle_;
};

template <typename T, int CAPACITY>
class Queue {
public:
    Queue() : handle_(nullptr) {}

    bool create() {
        handle_ = xQueueCreate(CAPACITY, sizeof(T));
        return handle_ != nullptr;
    }

    // Never blocks: a full queue is a dropped command the caller must count.
    bool push(const T& value) {
        return handle_ != nullptr && xQueueSend(handle_, &value, 0) == pdTRUE;
    }

    bool pop(T* out) {
        return handle_ != nullptr && out != nullptr && xQueueReceive(handle_, out, 0) == pdTRUE;
    }

private:
    QueueHandle_t handle_;
};

template <typename T>
class Snapshot {
public:
    Snapshot() : mutex_(nullptr), value_() {}

    bool create() {
        mutex_ = xSemaphoreCreateMutex();
        return mutex_ != nullptr;
    }

    void set(const T& value) {
        if (mutex_ == nullptr || xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            return;
        }
        value_ = value;
        xSemaphoreGive(mutex_);
    }

    T get() const {
        T copy = T();
        if (mutex_ == nullptr || xSemaphoreTake(mutex_, portMAX_DELAY) != pdTRUE) {
            return copy;
        }
        copy = value_;
        xSemaphoreGive(mutex_);
        return copy;
    }

private:
    SemaphoreHandle_t mutex_;
    T                 value_;
};

#endif  // FW_HOST_BUILD

}  // namespace fw

#endif  // FW_RT_PORT_H
