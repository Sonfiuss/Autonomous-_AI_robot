/* AD-Census depth tool — Jetson/Linux port.
 *
 * Two input modes:
 *
 *  FILE MODE (default):
 *    adcensus_depth <left.png> <right.png> [out_prefix] [min_disp] [max_disp]
 *    Defaults: out_prefix = "adcensus_out", min_disp = 0, max_disp = 64.
 *
 *  CAMERA MODE:
 *    adcensus_depth --camera [--left N] [--right N] [--width W] [--height H]
 *                            [--fps F] [--out prefix] [--min-disp D] [--max-disp D]
 *                            [--save]
 *    Defaults match stereo-camera/vision/StereoCamera::Config:
 *      left=0, right=1, width=640, height=480, fps=30, out=adcensus_out
 *    --save  also writes captures/live_left.png + captures/live_right.png.
 *
 * Output (both modes):
 *   <out>_disp.png        normalized 8-bit grayscale disparity
 *   <out>_disp_color.png  JET colormap of the same
 */
#include <iostream>
#include <chrono>
#include <cmath>
#include <cstring>
#include <cstdlib>
#include <opencv2/opencv.hpp>

#include "adcensus/ADCensusStereo.h"

using namespace std::chrono;

// ── Disparity output ─────────────────────────────────────────────────────────

static void SaveDisparityMap(const float32* disp_map, sint32 width, sint32 height,
                             const std::string& out_prefix)
{
    cv::Mat disp_mat(height, width, CV_8UC1);
    float32 min_disp = static_cast<float32>(width);
    float32 max_disp = -static_cast<float32>(width);
    for (sint32 i = 0; i < height; ++i) {
        for (sint32 j = 0; j < width; ++j) {
            const float32 disp = std::abs(disp_map[i * width + j]);
            if (disp != Invalid_Float) {
                min_disp = std::min(min_disp, disp);
                max_disp = std::max(max_disp, disp);
            }
        }
    }
    const float32 range = (max_disp - min_disp);
    for (sint32 i = 0; i < height; ++i) {
        for (sint32 j = 0; j < width; ++j) {
            const float32 disp = std::abs(disp_map[i * width + j]);
            if (disp == Invalid_Float || range <= 0.f) {
                disp_mat.data[i * width + j] = 0;
            } else {
                disp_mat.data[i * width + j] =
                    static_cast<uchar>((disp - min_disp) / range * 255);
            }
        }
    }

    const std::string p_gray  = out_prefix + "_disp.png";
    const std::string p_color = out_prefix + "_disp_color.png";
    cv::imwrite(p_gray, disp_mat);
    cv::Mat disp_color;
    cv::applyColorMap(disp_mat, disp_color, cv::COLORMAP_JET);
    cv::imwrite(p_color, disp_color);
    std::cout << "Saved: " << p_gray << "\n       " << p_color << std::endl;
    std::cout << "Disparity range: [" << min_disp << ", " << max_disp << "] px"
              << std::endl;

    // 16-bit raw disparity (disparity × 16, 0 = invalid) — readable by object_detector.py
    cv::Mat disp_raw(height, width, CV_16UC1);
    for (sint32 i = 0; i < height; ++i) {
        for (sint32 j = 0; j < width; ++j) {
            const float32 d = disp_map[i * width + j];
            if (!std::isfinite(d) || d <= 0.f) {
                disp_raw.at<uint16_t>(i, j) = 0;
            } else {
                const float32 v = std::abs(d) * 16.f;
                disp_raw.at<uint16_t>(i, j) =
                    static_cast<uint16_t>(std::min(v, 65535.f));
            }
        }
    }
    const std::string p_raw = out_prefix + "_disp_raw.png";
    cv::imwrite(p_raw, disp_raw);
    std::cout << "Saved: " << p_raw << " (16-bit raw)" << std::endl;
}

// ── Pack cv::Mat BGR into contiguous byte buffer ──────────────────────────────

static uint8* PackBGR(const cv::Mat& img) {
    const int w = img.cols, h = img.rows;
    auto* buf = new uint8[static_cast<size_t>(w) * h * 3];
    for (int i = 0; i < h; ++i) {
        for (int j = 0; j < w; ++j) {
            const cv::Vec3b& p = img.at<cv::Vec3b>(i, j);
            const size_t idx = (static_cast<size_t>(i) * w + j) * 3;
            buf[idx + 0] = p[0]; buf[idx + 1] = p[1]; buf[idx + 2] = p[2];
        }
    }
    return buf;
}

// ── Run AD-Census on a pair of cv::Mat BGR images ────────────────────────────

static int RunADCensus(const cv::Mat& img_left, const cv::Mat& img_right,
                       const std::string& out_prefix, int min_disp, int max_disp)
{
    if (img_left.empty() || img_right.empty()) {
        std::cerr << "Input images are empty." << std::endl;
        return -1;
    }
    if (img_left.rows != img_right.rows || img_left.cols != img_right.cols) {
        std::cerr << "Left/right image sizes differ." << std::endl;
        return -1;
    }

    const sint32 width  = static_cast<sint32>(img_left.cols);
    const sint32 height = static_cast<sint32>(img_left.rows);

    auto bytes_left  = PackBGR(img_left);
    auto bytes_right = PackBGR(img_right);

    ADCensusOption ad_option;
    ad_option.min_disparity = min_disp;
    ad_option.max_disparity = max_disp;
    ad_option.lrcheck_thres = 1.0f;
    ad_option.do_lr_check   = true;
    ad_option.do_filling    = true;

    std::cout << "w=" << width << " h=" << height
              << " disp=[" << min_disp << "," << max_disp << "]" << std::endl;

    ADCensusStereo ad_census;
    auto t0 = steady_clock::now();
    if (!ad_census.Initialize(width, height, ad_option)) {
        std::cerr << "AD-Census Initialize failed." << std::endl;
        delete[] bytes_left; delete[] bytes_right;
        return -2;
    }
    std::cout << "Initialized in "
              << duration_cast<milliseconds>(steady_clock::now() - t0).count() / 1000.0
              << " s" << std::endl;

    auto disparity = new float32[static_cast<size_t>(width) * height]();
    t0 = steady_clock::now();
    if (!ad_census.Match(bytes_left, bytes_right, disparity)) {
        std::cerr << "AD-Census Match failed." << std::endl;
        delete[] disparity; delete[] bytes_left; delete[] bytes_right;
        return -2;
    }
    std::cout << "Matched in "
              << duration_cast<milliseconds>(steady_clock::now() - t0).count() / 1000.0
              << " s" << std::endl;

    SaveDisparityMap(disparity, width, height, out_prefix);

    delete[] disparity;
    delete[] bytes_left;
    delete[] bytes_right;
    return 0;
}

// ── Camera capture ────────────────────────────────────────────────────────────

static int CameraMode(int argc, char** argv)
{
    // Defaults matching stereo-camera/vision/StereoCamera::Config
    int left_idx  = 0;
    int right_idx = 2;   // Jetson UVC: each camera occupies 2 nodes (capture+metadata)
    int width     = 640;
    int height    = 480;
    int fps       = 30;
    int min_disp  = 0;
    int max_disp  = 128;
    std::string out_prefix = "captures/adcensus_live";
    bool save_frames = false;

    for (int i = 2; i < argc; ++i) {
        auto eq = [&](const char* name) {
            return strcmp(argv[i], name) == 0 && i + 1 < argc;
        };
        if      (eq("--left"))      left_idx  = std::atoi(argv[++i]);
        else if (eq("--right"))     right_idx = std::atoi(argv[++i]);
        else if (eq("--width"))     width     = std::atoi(argv[++i]);
        else if (eq("--height"))    height    = std::atoi(argv[++i]);
        else if (eq("--fps"))       fps       = std::atoi(argv[++i]);
        else if (eq("--out"))       out_prefix = argv[++i];
        else if (eq("--min-disp"))  min_disp  = std::atoi(argv[++i]);
        else if (eq("--max-disp"))  max_disp  = std::atoi(argv[++i]);
        else if (strcmp(argv[i], "--save") == 0) save_frames = true;
        else {
            std::cerr << "Unknown camera option: " << argv[i] << std::endl;
            return -1;
        }
    }

    std::cout << "Camera mode: left=" << left_idx << " right=" << right_idx
              << " " << width << "x" << height << " @" << fps << "fps" << std::endl;

    cv::VideoCapture cap_l(left_idx), cap_r(right_idx);
    if (!cap_l.isOpened()) {
        std::cerr << "Cannot open left camera (index " << left_idx << ")" << std::endl;
        return -1;
    }
    if (!cap_r.isOpened()) {
        std::cerr << "Cannot open right camera (index " << right_idx << ")" << std::endl;
        return -1;
    }

    auto setCap = [&](cv::VideoCapture& cap) {
        cap.set(cv::CAP_PROP_FRAME_WIDTH,  width);
        cap.set(cv::CAP_PROP_FRAME_HEIGHT, height);
        cap.set(cv::CAP_PROP_FPS,          fps);
    };
    setCap(cap_l);
    setCap(cap_r);

    // Discard first few frames to let auto-exposure settle (same as StereoCamera practice).
    std::cout << "Warming up cameras..." << std::endl;
    for (int i = 0; i < 5; ++i) {
        cap_l.grab();
        cap_r.grab();
    }

    // Synchronized grab — both cameras grab before either retrieves.
    if (!cap_l.grab() || !cap_r.grab()) {
        std::cerr << "Failed to grab frames from cameras." << std::endl;
        return -1;
    }

    cv::Mat frame_l, frame_r;
    if (!cap_l.retrieve(frame_l) || !cap_r.retrieve(frame_r)) {
        std::cerr << "Failed to retrieve frames from cameras." << std::endl;
        return -1;
    }

    cap_l.release();
    cap_r.release();

    std::cout << "Captured: " << frame_l.cols << "x" << frame_l.rows << std::endl;

    if (save_frames) {
        const std::string pl = "captures/live_left.png";
        const std::string pr = "captures/live_right.png";
        cv::imwrite(pl, frame_l);
        cv::imwrite(pr, frame_r);
        std::cout << "Saved frames: " << pl << "  " << pr << std::endl;
    }

    return RunADCensus(frame_l, frame_r, out_prefix, min_disp, max_disp);
}

// ── File mode ─────────────────────────────────────────────────────────────────

static int FileMode(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage:\n"
                  << "  File mode:   " << argv[0]
                  << " <left> <right> [out_prefix] [min_disp] [max_disp]\n"
                  << "  Camera mode: " << argv[0]
                  << " --camera [--left N] [--right N] [--width W] [--height H]\n"
                  << "                        [--fps F] [--out prefix] "
                     "[--min-disp D] [--max-disp D] [--save]\n";
        return -1;
    }

    const std::string path_left  = argv[1];
    const std::string path_right = argv[2];
    const std::string out_prefix = (argc >= 4) ? argv[3] : "adcensus_out";
    const int min_disp = (argc >= 5) ? std::atoi(argv[4]) : 0;
    const int max_disp = (argc >= 6) ? std::atoi(argv[5]) : 64;

    cv::Mat img_left  = cv::imread(path_left,  cv::IMREAD_COLOR);
    cv::Mat img_right = cv::imread(path_right, cv::IMREAD_COLOR);
    if (img_left.empty() || img_right.empty()) {
        std::cerr << "Failed to read input images." << std::endl;
        return -1;
    }

    return RunADCensus(img_left, img_right, out_prefix, min_disp, max_disp);
}

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char** argv)
{
    if (argc >= 2 && strcmp(argv[1], "--camera") == 0)
        return CameraMode(argc, argv);
    return FileMode(argc, argv);
}
