#pragma once

#include <string>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>

/**
 * Stereo camera wrapper.
 *
 * Captures synchronised left/right frames, optionally rectifies them with a
 * pre-computed calibration, and returns a metric depth map (float32, metres)
 * via SGBM disparity matching.
 *
 * Calibration file is an OpenCV FileStorage .yml/.xml containing:
 *   M1, D1  – left  camera matrix + distortion
 *   M2, D2  – right camera matrix + distortion
 *   R, T    – stereo extrinsics
 *   (R1, R2, P1, P2, Q are computed on load if absent)
 */
class StereoCamera {
public:
    struct Config {
        int left_idx  = 0;
        int right_idx = 1;
        int width     = 640;
        int height    = 480;
        int fps       = 30;
        // SGBM parameters
        int num_disparities = 128;
        int block_size      = 5;
    };

    StereoCamera();
    explicit StereoCamera(const Config& cfg);
    ~StereoCamera();

    // Non-copyable
    StereoCamera(const StereoCamera&) = delete;
    StereoCamera& operator=(const StereoCamera&) = delete;

    /** Open both cameras. Returns false if either fails. */
    bool open();

    /** Load stereo calibration from OpenCV FileStorage file. */
    bool loadCalibration(const std::string& path);

    /**
     * Capture one synchronised stereo pair and compute a depth map.
     *
     * @param depth_out  Output depth map (CV_32F, metres). NaN where invalid.
     * @param left_out   Optional rectified left image for display.
     * @return true on success.
     */
    bool captureDepth(cv::Mat& depth_out, cv::Mat* left_out = nullptr);

    void release();
    bool isOpened() const;

    int width()  const { return m_cfg.width;  }
    int height() const { return m_cfg.height; }

private:
    Config         m_cfg;
    cv::VideoCapture m_cap_l, m_cap_r;
    cv::Ptr<cv::StereoSGBM> m_matcher;

    // Rectification maps
    cv::Mat m_map_lx, m_map_ly;
    cv::Mat m_map_rx, m_map_ry;
    cv::Mat m_Q;          // disparity-to-depth (4×4)
    bool    m_calibrated = false;

    void buildMatcher();
};
