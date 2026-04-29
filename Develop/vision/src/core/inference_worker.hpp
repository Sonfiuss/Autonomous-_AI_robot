#pragma once

#include <atomic>
#include <thread>
#include <vector>
#include <memory>
#include <string>
#include <onnxruntime_cxx_api.h>

#include "depth_session.hpp"
#include "frame_types.hpp"
#include "sync_primitives.hpp"

namespace vision
{
     class InferenceWorker
     {
     public:
          InferenceWorker(Ort::Env &env,
                          const std::string &modelPath,
                          int workerCount,
                          BoundedQueue<StampedFrame> &jobQueue,
                          LatestSlot<DepthResult> &latestDepth,
                          int dstHeight,
                          int dstWidth)
              : jobQueue_(jobQueue), latestDepth_(latestDepth), dstH_(dstHeight), dstW_(dstWidth)
          {
               const int safeWorkerCount = std::max(1, workerCount);
               session_.reserve(safeWorkerCount);
               for (int i = 0; i < safeWorkerCount; ++i)
               {
                    session_.emplace_back(std::make_unique<DepthSession>(env, modelPath));
               }
          }
          ~InferenceWorker()
          {
               stop();
          }

          void start()
          {
               running_ = true;
               thread_.reserve(session_.size());
               for (size_t i = 0; i < session_.size(); ++i)
               {
                    thread_.emplace_back([this, i]
                                         { this->runWorker(i); });
               }
          }
          void stop()
          {
               running_ = false;
               for (auto &t : thread_)
               {
                    if (t.joinable())
                    {
                         t.join();
                    }
               }
               thread_.clear();
          }

          long inferenceCount() const
          {
               return inferenceCounter_.load();
          }

     private:
          void runWorker(size_t idx)
          {
               auto &session = *session_[idx];
               while (running_)
               {
                    auto jobOpt = jobQueue_.pop();
                    if (!jobOpt)
                    {
                         break; // Queue is closed and empty
                    }
                    auto &job = *jobOpt;
                    try
                    {
                         cv::Mat depthColor = session.infer(job.frame, dstH_, dstW_);
                         DepthResult result;
                         result.depthColor = std::move(depthColor);
                         result.frameIndex = job.frameIndex;
                         result.sourceTimestamp = job.sourceTimestamp;

                         latestDepth_.set(std::move(result));
                         inferenceCounter_++;
                    }
                    catch (const std::exception &e)
                    {
                         std::cerr << "Inference error on worker " << idx << ": " << e.what() << "\n";
                    }
               }
          }
          BoundedQueue<StampedFrame> &jobQueue_;
          LatestSlot<DepthResult> &latestDepth_;
          int dstH_;
          int dstW_;
          std::vector<std::unique_ptr<DepthSession>> session_;
          std::vector<std::thread> thread_;
          std::atomic<long> inferenceCounter_{0};
          std::atomic<bool> running_{false};
     };

}