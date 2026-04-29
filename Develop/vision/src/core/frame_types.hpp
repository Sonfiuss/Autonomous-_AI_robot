#pragma once

#include <opencv2/opencv.hpp>

namespace vision {

     // A frame comming out of the capture stage
     // 'sourceTimestamp' is in seconds, derived form the video stream
     // 'frameIndex' so the rest of the pipeline can reason about timing
     // without relying on wall-clock arrival of frames
     struct StampedFrame {
          cv::Mat frame;
          double sourceTimestamp = 0.0; // Original timestamp from the video source
          long frameIndex = 0; // Index of the frame in the video sequence
     };
     
     // A depth estimation result from the inference worker
     // 'sourceTimestamp' is propagated from the originating StampedFrame, so that 
     // the writer can decide which depth map is "current" at any output time
     struct DepthResult
     {
          cv::Mat depthColor; // Colorized depth map for visualization
          double sourceTimestamp = 0.0; // Timestamp when inference was performed
          long frameIndex = 0; // Index of the corresponding input frame
     };
     
}
