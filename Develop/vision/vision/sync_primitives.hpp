#pragma once

#include <condition_variable>
#include <mutex>
#include <optional>
#include <queue>

namespace vision
{

     // Thread-safe bounded FIFO queue.
     // - push() blocks while the queue is full (creates back-pressure on producer).
     // - pop() blocks while the queue is empty.
     // - close() unblocks all waiting threads; pop() then returns std::nullopt.
     template <typename T>
     class BoundedQueue
     {
     public:
          explicit BoundedQueue(std::size_t capacity) : capacity_(capacity) {}
          BoundedQueue(const BoundedQueue &) = delete;
          BoundedQueue &operator=(const BoundedQueue &) = delete;
          BoundedQueue(BoundedQueue &&) = delete;
          BoundedQueue &operator=(BoundedQueue &&) = delete;
          
          bool push(T value)
          {
               std::unique_lock<std::mutex> lk(mu_);
               notFull_.wait(lk, [&]
                             { return closed_ || queue_.size() < capacity_; });
               if (closed_)
                    return false;
               queue_.push(std::move(value));
               notEmpty_.notify_one();
               return true;
          }

          std::optional<T> pop()
          {
               std::unique_lock<std::mutex> lk(mu_);
               notEmpty_.wait(lk, [&]
                              { return closed_ || !queue_.empty(); });
               if (queue_.empty())
                    return std::nullopt;
               T v = std::move(queue_.front());
               queue_.pop();
               notFull_.notify_one();
               return v;
          }

          void close()
          {
               std::lock_guard<std::mutex> lk(mu_);
               closed_ = true;
               notFull_.notify_all();
               notEmpty_.notify_all();
          }

          std::size_t size()
          {
               std::lock_guard<std::mutex> lk(mu_);
               return queue_.size();
          }

     private:
          std::size_t capacity_;
          std::queue<T> queue_;
          std::mutex mu_;
          std::condition_variable notFull_;
          std::condition_variable notEmpty_;
          bool closed_ = false;
     };

     // Thread-safe single-slot register for "latest value wins" semantics.
     // Producers overwrite; consumers read the most recent snapshot without blocking.
     template <typename T>
     class LatestSlot
     {
     public:
          LatestSlot() = default;
          LatestSlot(const LatestSlot &) = delete;
          LatestSlot &operator=(const LatestSlot &) = delete;
          LatestSlot(LatestSlot &&) = delete;
          LatestSlot &operator=(LatestSlot &&) = delete;

          void set(T value)
          {
               std::lock_guard<std::mutex> lk(mu_);
               value_ = std::move(value);
               hasValue_ = true;
          }

          std::optional<T> get()
          {
               std::lock_guard<std::mutex> lk(mu_);
               if (!hasValue_)
                    return std::nullopt;
               return value_;
          }

     private:
          std::mutex mu_;
          T value_{};
          bool hasValue_ = false;
     };
} // namespace vision