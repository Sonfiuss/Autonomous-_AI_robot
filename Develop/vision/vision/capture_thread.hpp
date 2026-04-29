#pragma once

#include <atomic>
#include <thread>

#include <opencv2/opencv.hpp>

#include "frame_types.hpp"
#include "sync_primitives.hpp"

namespace vision
{

     // CaptureThread reads frames from cv::VideoCapture and pushes them into:
     // - outQueue : ordered, bounded; consumed by OutputWriter (preserves all frames
     // + applies natural back-pressure so we never accumulate frames).
     // - latestSlot : single-slot register for the Scheduler (latest frame wins).
     class CaptureThread
     {
     public:
          CaptureThread(cv::VideoCapture &cap,
                        double sourceFps,
                        BoundedQueue<StampedFrame> &outQueue,
                        LatestSlot<StampedFrame> &latestSlot)
              : cap_(cap), sourceFps_(sourceFps),
                outQueue_(outQueue), latestSlot_(latestSlot) {}

          ~CaptureThread() { stop(); }

          void start()
          {
               running_ = true;
               thread_ = std::thread([this]
                                     { run(); });
          }

          void stop()
          {
               running_ = false;
               if (thread_.joinable())
                    thread_.join();
          }

          bool finished() const { return finished_; }
          long totalFrames() const { return frameCount_; }

     private:
          void run()
          {
               long index = 0;
               while (running_)
               {
                    cv::Mat frame;
                    if (!cap_.read(frame))
                         break;

                    StampedFrame sf;
                    sf.frame = frame;
                    sf.frameIndex = index;
                    sf.sourceTimestamp = (sourceFps_ > 0.0) ? (index / sourceFps_) : 0.0;

                    latestSlot_.set(sf);
                    // push() blocks if queue full -> implicit pacing for the whole pipeline.
                    if (!outQueue_.push(sf))
                         break;

                    ++index;
               }
               frameCount_ = index;
               finished_ = true;
               outQueue_.close(); // signal end-of-stream to the writer
          }

          cv::VideoCapture &cap_;
          double sourceFps_;
          BoundedQueue<StampedFrame> &outQueue_;
          LatestSlot<StampedFrame> &latestSlot_;

          std::thread thread_;
          std::atomic<bool> running_{false};
          std::atomic<bool> finished_{false};
          std::atomic<long> frameCount_{0};
     };

} // namespace vision