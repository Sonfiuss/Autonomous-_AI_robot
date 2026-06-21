#include "StereoCamera.hpp"
#include "../core/Depth.hpp"

#include <cmath>
#include <cstdio>
#include <chrono>
#include <stdexcept>

// ── Constructor / Destructor ──────────────────────────────────────────────────

StereoCamera::StereoCamera() : m_cfg(Config{}) {
    buildMatcher();
}

StereoCamera::StereoCamera(const Config& cfg) : m_cfg(cfg) {
    buildMatcher();
}

StereoCamera::~StereoCamera() {
    release();
}

void StereoCamera::buildMatcher() {
    const int bs = m_cfg.block_size;
    const int nd = m_cfg.num_disparities;
    m_matcher = cv::StereoSGBM::create(
        /*minDisparity=*/   0,
        /*numDisparities=*/ nd,
        /*blockSize=*/      bs,
        /*P1=*/             8  * 3 * bs * bs,
        /*P2=*/             32 * 3 * bs * bs,
        /*disp12MaxDiff=*/  1,
        /*preFilterCap=*/   63,
        /*uniquenessRatio=*/10,
        /*speckleWindowSize=*/100,
        /*speckleRange=*/   32,
        /*mode=*/           cv::StereoSGBM::MODE_SGBM_3WAY
    );
}

// ── Open / Close ─────────────────────────────────────────────────────────────

bool StereoCamera::open() {
    auto openCap = [&](cv::VideoCapture& cap, int idx) {
        if (!cap.open(idx)) return false;
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  m_cfg.width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, m_cfg.height);
        cap.set(cv::CAP_PROP_FPS,          m_cfg.fps);
        return true;
    };
    if (!openCap(m_cap_l, m_cfg.left_idx)) {
        fprintf(stderr, "[StereoCamera] Cannot open left camera (idx %d)\n",
                m_cfg.left_idx);
        return false;
    }
    if (!openCap(m_cap_r, m_cfg.right_idx)) {
        fprintf(stderr, "[StereoCamera] Cannot open right camera (idx %d)\n",
                m_cfg.right_idx);
        return false;
    }
    return true;
}

bool StereoCamera::isOpened() const {
    return m_cap_l.isOpened() && m_cap_r.isOpened();
}

void StereoCamera::release() {
    m_cap_l.release();
    m_cap_r.release();
}

// ── Calibration ───────────────────────────────────────────────────────────────

bool StereoCamera::loadCalibration(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        fprintf(stderr, "[StereoCamera] Cannot open calibration file: %s\n",
                path.c_str());
        return false;
    }

    cv::Mat M1, D1, M2, D2, R, T;
    cv::Mat R1, R2, P1, P2;

    fs["M1"] >> M1; fs["D1"] >> D1;
    fs["M2"] >> M2; fs["D2"] >> D2;
    fs["R"]  >> R;  fs["T"]  >> T;

    // Prefer pre-computed rectification if stored
    fs["R1"] >> R1; fs["R2"] >> R2;
    fs["P1"] >> P1; fs["P2"] >> P2;
    fs["Q"]  >> m_Q;

    const cv::Size img_sz(m_cfg.width, m_cfg.height);

    if (R1.empty()) {
        cv::stereoRectify(M1, D1, M2, D2, img_sz, R, T,
                          R1, R2, P1, P2, m_Q,
                          cv::CALIB_ZERO_DISPARITY, 0, img_sz);
    }

    cv::initUndistortRectifyMap(M1, D1, R1, P1, img_sz,
                                CV_32FC1, m_map_lx, m_map_ly);
    cv::initUndistortRectifyMap(M2, D2, R2, P2, img_sz,
                                CV_32FC1, m_map_rx, m_map_ry);

    // Capture rectified intrinsics for Reconstruct (must match the depth's K).
    if (!P1.empty()) {
        cv::Mat P1d; P1.convertTo(P1d, CV_64F);
        m_fx = (float)P1d.at<double>(0, 0);
        m_fy = (float)P1d.at<double>(1, 1);
        m_cx = (float)P1d.at<double>(0, 2);
        m_cy = (float)P1d.at<double>(1, 2);
        if (!P2.empty()) {
            cv::Mat P2d; P2.convertTo(P2d, CV_64F);
            // baseline = -P2(0,3)/fx  (sign per OpenCV convention; |·| in mm)
            m_baseline = std::fabs((float)P2d.at<double>(0, 3) / m_fx);
        }
    }

    m_calibrated = true;
    printf("[StereoCamera] Calibration loaded from %s\n", path.c_str());
    return true;
}

bool StereoCamera::applyIntrinsicsTo(::Config& cfg) const {
    if (!m_calibrated || m_fx <= 0.f) return false;
    cfg.fx = m_fx; cfg.fy = m_fy; cfg.cx = m_cx; cfg.cy = m_cy;
    if (m_baseline > 0.f) cfg.baseline = m_baseline;
    cfg.z_min = m_cfg.z_min; cfg.z_max = m_cfg.z_max;
    return true;
}

// ── Capture ───────────────────────────────────────────────────────────────────

bool StereoCamera::captureDepth(cv::Mat& depth_out, cv::Mat* left_out) {
    double t_ms;
    return captureDepth(depth_out, t_ms, left_out);
}

bool StereoCamera::captureDepth(cv::Mat& depth_out, double& t_ms_out,
                               cv::Mat* left_out) {
    // Fix stereo skew: grab() both first (near-simultaneous), then retrieve().
    // Sequential read() could add up to one frame period (~33 ms) of skew.
    if (!m_cap_l.grab() || !m_cap_r.grab()) return false;
    t_ms_out = std::chrono::duration<double, std::milli>(
                   std::chrono::steady_clock::now().time_since_epoch()).count();

    cv::Mat frame_l, frame_r;
    if (!m_cap_l.retrieve(frame_l) || !m_cap_r.retrieve(frame_r)) return false;

    cv::Mat gray_l, gray_r;
    cv::cvtColor(frame_l, gray_l, cv::COLOR_BGR2GRAY);
    cv::cvtColor(frame_r, gray_r, cv::COLOR_BGR2GRAY);

    if (m_calibrated) {
        cv::remap(gray_l, gray_l, m_map_lx, m_map_ly, cv::INTER_LINEAR);
        cv::remap(gray_r, gray_r, m_map_rx, m_map_ry, cv::INTER_LINEAR);
        if (left_out) {
            cv::Mat cl;
            cv::remap(frame_l, cl, m_map_lx, m_map_ly, cv::INTER_LINEAR);
            *left_out = cl;
        }
    } else if (left_out) {
        *left_out = frame_l;
    }

    // Compute disparity (fixed-point, 16-bit, scale factor 16)
    cv::Mat disp16;
    m_matcher->compute(gray_l, gray_r, disp16);

    cv::Mat disp;
    disp16.convertTo(disp, CV_32F, 1.0 / 16.0);

    if (m_calibrated && !m_Q.empty()) {
        // Reproject to 3D and extract Z channel. Q from a calibration in mm
        // yields depth in mm (plan units).
        cv::Mat pts3d;
        cv::reprojectImageTo3D(disp, pts3d, m_Q, true);
        cv::extractChannel(pts3d, depth_out, 2);          // Z component (mm)
        cv::Mat mask = (disp <= 0);
        depth_out.setTo(std::numeric_limits<float>::quiet_NaN(), mask);
        // Validity clamp: drop SGBM outliers outside [z_min, z_max] (mm).
        cv::Mat valid = depthutil::validityMask(depth_out, m_cfg.z_min, m_cfg.z_max);
        depthutil::applyMask(depth_out, valid);
    } else {
        // No calibration: derive a PLAUSIBLE (non-metric) depth from disparity via
        // Z = fx·baseline / disp, using the same defaults the reconstruct pipeline
        // unprojects with (Config fx=600 px, baseline=60 mm) so the cloud is
        // self-consistent and lands inside [z_min, z_max]. NOT metrically accurate
        // — calibrate (tools/stereo_calibrate.py) + --calib for true scale.
        constexpr float kFxPx = 600.f, kBaselineMm = 60.f;
        depth_out.create(disp.size(), CV_32F);
        const float fb = kFxPx * kBaselineMm;
        for (int y = 0; y < disp.rows; ++y) {
            const float* d = disp.ptr<float>(y);
            float* z = depth_out.ptr<float>(y);
            for (int x = 0; x < disp.cols; ++x)
                z[x] = (d[x] > 1.f) ? fb / d[x]
                                    : std::numeric_limits<float>::quiet_NaN();
        }
        cv::Mat valid = depthutil::validityMask(depth_out, m_cfg.z_min, m_cfg.z_max);
        depthutil::applyMask(depth_out, valid);
    }

    return true;
}
