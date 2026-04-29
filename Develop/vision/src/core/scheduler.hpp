#pragma once

#include <atomic>
#include <thread>
#include <chrono>

#include "frame_types.hpp"
#include "sync_primitives.hpp"

namespace vision {
     /* Scheduler dispaches interence jons at a targe rate (e.g. 6 Hz) by sampling
     the LatestSlot<StampedFrame> populated by the capture thread

     Key properties:
     - Sampling is driven by the *source timestamp* of the lasst captured frame, not 
     by the frame count. This guarantees corrcect cadence event if the producer rate is jittery
     - It pushes into a *bounded* job queue. If workers are staturated the scheduler simply
     skips that tick instead of accumulating backlog.

     Hence: no unbounned queue, no drift over time
     */

     class Scheduler {
     public:
          Scheduler(double targetFps,
                    LatestSlot<StampedFrame>& latestFrame,
                    BoundedQueue<StampedFrame>& jobQueue,
                    const std::atomic<bool>& captureFinished)
               : interval_(1.0/ std::max(1e-3, targetFps)),
                 latestFrame_(latestFrame),
                 jobQueue_(jobQueue),
                 captureFinished_(captureFinished) {}

          ~Scheduler() {
               stop();
          }

          void start() {
               running_ = true;
               thread_ = std::thread([this] {run();});
          }

          void stop() {
               running_ = false;
               if (thread_.joinable()) {
                    thread_.join();
               }
          }
          
          long dispatchedCount() const {
               return dispatched_.load();
          }
     private:
          void run() {
               using clock = std::chrono::steady_clock;
               double lastTs = -1.0;
               auto nextTick = clock::now();
               
               while (running_) {
                    auto frameOpt = latestFrame_.get();
                    if (frameOpt) {
                         const auto& sf = *frameOpt;
                         if(captureFinished_ && sf.sourceTimestamp < lastTs) {
                              // Capture thread has signaled finished, but we haven't seen any valid frame yet
                              // This can happen if the video is empty or unreadable. In this case we should stop the scheduler
                              break;
                         }
                         // Only dispatch if this frame is "never enough" in source time.
                         if (sf.sourceTimestamp - lastTs >= interval_) {
                              // Try to enqueue without blocking the writer pipeline.
                              // if full just skip this frame
                              if (jobQueue_.tryPush(sf)) {
                                   lastTs = sf.sourceTimestamp;
                                   ++dispatched_;
                              }
                         }
                    }
                    else if(captureFinished_) {
                         // No more frames will come, we can stop the scheduler
                         break;
                    }
                    
                    nextTick += std::chrono::milliseconds(static_cast<int>(interval_ * 1000.0 / 4.0));
                    std::this_thread::sleep_until(nextTick);
               }
               jobQueue_.close(); // signal end-of-jobs to the workers
          }

          double interval_; // in seconds
          LatestSlot<StampedFrame>& latestFrame_;
          BoundedQueue<StampedFrame>& jobQueue_;
          const std::atomic<bool>& captureFinished_;
          std::thread  thread_;
          std::atomic<bool> running_{false};
          std::atomic<long> dispatched_{0};
     };
}