#pragma once
#include <atomic>  
#include <iostream>
#include <string>

#include <opencv2/opencv.hpp>

#include "frame_types.hpp"
#include "sync_primitives.hpp"

namespace vision {

     // OutputWriter is cosumes the *ordered* frame queue produced by CaptureThread
     // pairs each frame with the most recent depth result, and writes the side-by-side
     // composite to the output video

     // Because it preserves every original frame and uses the source FPS, the output video
     // duration is idential to the input. Depth refreshes at the inference rate (e.g. 6 Hz) 
     // and is reused for the in-beteween frames.

     class OutputWriter {
     public:
          OutputWriter(BoundedQueue<StampedFrame>& orderedFrames, 
                       LatestSlot<DepthResult>& latestDepth,
                       cv::VideoWriter *writer,
                       int frameWidth,
                       int frameHeight,
                       bool showWindow)
                       :orderedFrames_(orderedFrames),
                        latestDepth_(latestDepth), 
                        writer_(writer),
                        frameH_(frameHeight),
                        frameW_(frameWidth),
                        showWindow_(showWindow),
                        placeholder_(frameHeight, frameWidth, CV_8UC3, cv::Scalar(0,0,0)) {}
                           
          long run(){
               long written = 0;
               while (true) {
                    auto frameOpt = orderedFrames_.pop();
                    if (!frameOpt) {
                         break; // Queue is closed and empty
                    }
                    const auto& sf = *frameOpt;
                    cv::Mat depth = pickDepthOrPlaceholder();
                    cv::Mat combined;
                    cv::hconcat(sf.frame, depth, combined);
                    if(writer_) {
                         *writer_ << combined;
                    }
                    

                    if (showWindow_) {
                         cv::imshow("Deep Anything ONNX | Left: RGB Right: Depth", combined);
                         if ((cv::waitKey(1) & 0xFF) == 27) { // Exit on 'Esc' key
                              hasStopped_ = true;
                              break;
                         }
                    }
                    ++written;
                    if(written % 30 == 0) {
                         std::cout << "[Writer] Written " << written << " frames\n";
                    }
               }
          }        
          ~OutputWriter() = default;               
          bool isStopped() const {
               return hasStopped_.load();
          }

     private:
          cv::Mat pickDepthOrPlaceholder() {
               auto d = latestDepth_.get();
               if(!d) return placeholder_;
               if(d->depthColor.cols != frameW_ || d->depthColor.rows != frameH_) {
                    cv::Mat fixed;
                    cv::resize(d->depthColor, fixed, cv::Size(frameW_, frameH_), 0.0, 0.0, cv::INTER_CUBIC);
                    return fixed;
               }
               return d->depthColor;
          }
          BoundedQueue<StampedFrame>& orderedFrames_;
          LatestSlot<DepthResult>& latestDepth_;
          cv::VideoWriter *writer_;
          int frameH_;
          int frameW_;
          bool showWindow_;
          cv::Mat placeholder_;
          std::atomic<bool> hasStopped_{false};
     };
}    // namespace vision