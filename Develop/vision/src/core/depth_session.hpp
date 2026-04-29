#pragma once

#include <array>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <onnxruntime_cxx_api.h>
#include <opencv2/opencv.hpp>

namespace vision {

// Wraps a single ONNX Runtime session for the Depth Anything model.
// One DepthSession per worker thread => no contention inside Run().
class DepthSession {
     public:
          DepthSession(Ort::Env& env, const std::string& modelPath) {
               if(modelPath.empty()) {
                    throw std::runtime_error("Model path is empty");
               }
               Ort::SessionOptions opts;
               opts.SetIntraOpNumThreads(1);
               opts.SetGraphOptimizationLevel(GraphOptimizationLevel::ORT_ENABLE_BASIC);
               opts.DisableMemPattern();

               session_ = std::make_unique<Ort::Session>(env, modelPath.c_str(), opts);

               Ort::AllocatorWithDefaultOptions allocator;
               auto inName = session_->GetInputNameAllocated(0, allocator);
               auto outName = session_->GetOutputNameAllocated(0, allocator);
               inputName_ = inName.get();
               outputName_ = outName.get();

               auto inShape = session_->GetInputTypeInfo(0).GetTensorTypeAndShapeInfo().GetShape();
               std::tie(inputH_, inputW_) = resolveInputSize(inShape);
          }

          int inputHeight() const { return inputH_; }
          int inputWidth() const { return inputW_; }

          const std::string& inputName() const { return inputName_; }
          const std::string& outputName() const { return outputName_; }

          // Runs preprocess + inference + colorisation, producing a depth-colour map
          // resized to (dstH, dstW).
          cv::Mat infer(const cv::Mat& frameBgr, int dstH, int dstW) {
               auto inputData = preprocess(frameBgr);

               std::array<int64_t, 4> shape{1, 3, inputH_, inputW_};
               auto memInfo = Ort::MemoryInfo::CreateCpu(OrtArenaAllocator, OrtMemTypeDefault);

               Ort::Value inputTensor = Ort::Value::CreateTensor<float>(
               memInfo, inputData.data(), inputData.size(), shape.data(), shape.size());

               const char* inNames[] = {inputName_.c_str()};
               const char* outNames[] = {outputName_.c_str()};

               auto outputs = session_->Run(
               Ort::RunOptions{nullptr}, inNames, &inputTensor, 1, outNames, 1);

               if(outputs.empty()|| !outputs[0].IsTensor()) {
                    throw std::runtime_error("Model did not return any outputs");
               }
               auto tensorInfo = outputs[0].GetTensorTypeAndShapeInfo();
               if(tensorInfo.GetElementType() != ONNX_TENSOR_ELEMENT_DATA_TYPE_FLOAT) {
                    throw std::runtime_error("Unexpected output tensor type");
               }

               float* outData = outputs[0].GetTensorMutableData<float>();
               if(outData == nullptr) {
                    throw std::runtime_error("Output tensor data is null");
               }
               auto outShape = tensorInfo.GetShape();

               int outH = inputH_;
               int outW = inputW_;
               if (outShape.size() >= 2) {
                    outH = static_cast<int>(outShape[outShape.size() - 2]);
                    outW = static_cast<int>(outShape[outShape.size() - 1]);
               }
               return colorize(outData, outH, outW, dstH, dstW);
     }
     private:
          static std::pair<int, int> resolveInputSize(const std::vector<int64_t>& shape) {
               constexpr int kDefault = 518;
               if (shape.size() != 4) return {kDefault, kDefault};
               int h = (shape[2] > 0) ? static_cast<int>(shape[2]) : kDefault;
               int w = (shape[3] > 0) ? static_cast<int>(shape[3]) : kDefault;
               return {h, w};
          }

          std::vector<float> preprocess(const cv::Mat& frameBgr) const {
               cv::Mat rgb;
               cv::cvtColor(frameBgr, rgb, cv::COLOR_BGR2RGB);
               
               cv::Mat resized;
               cv::resize(rgb, resized, cv::Size(inputW_, inputH_), 0.0, 0.0, cv::INTER_CUBIC);
               
               cv::Mat floatImg;
               resized.convertTo(floatImg, CV_32F, 1.0 / 255.0);
               
               std::vector<cv::Mat> channels(3);
               cv::split(floatImg, channels);
               
               const std::array<float, 3> mean{0.485f, 0.456f, 0.406f};
               const std::array<float, 3> stdv{0.229f, 0.224f, 0.225f};
               for (int c = 0; c < 3; ++c) {
                    channels[c] = (channels[c] - mean[c]) / stdv[c];
               }

               std::vector<float> tensor(static_cast<std::size_t>(3) * inputH_ * inputW_);
               const std::size_t plane = static_cast<std::size_t>(inputH_) * inputW_;
               for (int c = 0; c < 3; ++c) {
                    const float* src = reinterpret_cast<float*>(channels[c].data);
                    std::copy(src, src + plane, tensor.begin() + c * plane);
               }
               return tensor;
          }

          static cv::Mat colorize(const float* data, int outH, int outW, int dstH, int dstW) {
               cv::Mat depth(outH, outW, CV_32F, const_cast<float*>(data));
               cv::Mat depthCopy = depth.clone();

               cv::Mat depthResized;
               cv::resize(depthCopy, depthResized, cv::Size(dstW, dstH), 0.0, 0.0, cv::INTER_CUBIC);

               double minVal = 0.0;
               double maxVal = 0.0;
               cv::minMaxLoc(depthResized, &minVal, &maxVal);

               cv::Mat depthNorm;
               depthResized.convertTo(
               depthNorm, CV_32F,
               1.0 / (maxVal - minVal + 1e-8),
               -minVal / (maxVal - minVal + 1e-8));
               
               cv::Mat depthU8;
               depthNorm.convertTo(depthU8, CV_8U, 255.0);
               
               cv::Mat depthColor;
               cv::applyColorMap(depthU8, depthColor, cv::COLORMAP_INFERNO);
               return depthColor;
          }

          std::unique_ptr<Ort::Session> session_;
          std::string inputName_;
          std::string outputName_;
          int inputH_ = 0;
          int inputW_ = 0;
     };

} // namespace vision