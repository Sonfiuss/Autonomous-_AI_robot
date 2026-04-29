#include <algorithm>
#include <array>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <onnxruntime_cxx_api.h>

#include "core/app_config.hpp"
#include "core/capture_thread.hpp"
#include "core/depth_session.hpp" 
#include "core/frame_types.hpp"
#include "core/inference_worker.hpp"
#include "core/output_write.hpp"
#include "core/scheduler.hpp"
#include "core/sync_primitives.hpp"

namespace fs = std::filesystem;

using namespace vision;

int main(int argc, char** argv) {
     try {
          AppConfig cfg = parseArgs(argc, argv);
          
          if(cfg.modelPath.empty()) {
               throw std::runtime_error("Model path is required");
          }

          if (!fs::exists(cfg.videoPath)) {
               throw std::runtime_error("Video not found: " + cfg.videoPath);
          }
          if (!fs::exists(cfg.modelPath)) {
               throw std::runtime_error("Model not found: " + cfg.modelPath);
          }
     
     
          cv::VideoCapture cap(cfg.videoPath);
          if (!cap.isOpened()) {
               throw std::runtime_error("Cannot open video: " + cfg.videoPath);
          }
     
          const int frameW = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_WIDTH));
          const int frameH = static_cast<int>(cap.get(cv::CAP_PROP_FRAME_HEIGHT));
          const double fps = cap.get(cv::CAP_PROP_FPS) > 0.0 ? cap.get(cv::CAP_PROP_FPS) : 30.0;
     
          cv::VideoWriter videoWriter;
          if (!cfg.outputPath.empty()) {
               fs::create_directories(fs::path(cfg.outputPath).parent_path());
               fs::path outPath(cfg.outputPath);
               fs::path outDir = outPath.parent_path();
               if (!outDir.empty()) {
                    fs::create_directories(outDir);
               }
               int fourcc = cv::VideoWriter::fourcc('m', 'p', '4', 'v');
               videoWriter.open(cfg.outputPath, fourcc, fps, cv::Size(frameW * 2, frameH));
               if (!videoWriter.isOpened()) {
                    throw std::runtime_error("Cannot open output file: " + cfg.outputPath);
               }
          }
          Ort::Env env(ORT_LOGGING_LEVEL_WARNING, "DepthAnything");
          const int workerCount = std::max(1, cfg.workerCount);
          BoundedQueue<StampedFrame> orderedFrames(cfg.inputQueueCapacity);
          BoundedQueue<StampedFrame> jobQueue(std::max(1, workerCount));
          LatestSlot<StampedFrame> latestFrame;
          LatestSlot<DepthResult> latestDepth;
          std::atomic<bool> captureFinishedFlag{false};

          CaptureThread capture(cap, fps, orderedFrames, latestFrame, &captureFinishedFlag);
          InferenceWorker worker(env, cfg.modelPath, workerCount, jobQueue, latestDepth, frameH, frameW);
          Scheduler scheduler(cfg.inferenceFps, latestFrame, jobQueue, captureFinishedFlag);
          OutputWriter writer(orderedFrames, latestDepth, videoWriter.isOpened() ? &videoWriter : nullptr,
                              frameW, frameH, cfg.showWindow);
          worker.start();
          scheduler.start();
          capture.start();

          // Writer run on main thread ),
          long written = writer.run();
          // Drain & shutdown sequence
          capture.stop();
          scheduler.stop();
          jobQueue.close(); // Signal workers to stop after finishing current jobs
          worker.stop();

          if(videoWriter.isOpened()) {
               videoWriter.release();
          }
          cap.release();
          if (cfg.showWindow) {
               cv::destroyAllWindows();
          }
          std::cout << "[DONE] Total processed frames: " << written << "\n"
                    << "[DONE] Total inference performed: " << worker.inferenceCount() << "\n"
                    << "[DONE] Dispatched frames: " << scheduler.dispatchedCount() << "\n"
                    << "[DONE] written frames: " << written << "\n";

          if (!cfg.outputPath.empty()) {
               std::cout << "[DONE] Output saved: " << cfg.outputPath << "\n";
          }
          return 0;
     } catch (const Ort::Exception& e) {

          std::cerr << "ONNX Runtime error: " << e.what() << "\n";
          return 1;
     }
     catch (const std::exception& e) {
          std::cerr << "Error: " << e.what() << "\n";
          return 1;
     }
}