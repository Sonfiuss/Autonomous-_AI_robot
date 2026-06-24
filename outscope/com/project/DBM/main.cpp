/* AD-Census depth tool — Jetson/Linux port.
 *
 * Keeps IMAGE-FILE input (a left/right capture pair) and produces an
 * upstream-style disparity visualization:
 *   <out>_disp.png        normalized 8-bit grayscale disparity
 *   <out>_disp_color.png  JET colormap of the same
 *
 * Algorithm is the unmodified vendored AD-Census in adcensus/
 * (Xing Mei et al., impl. by Yingsong Li / ethan-li-coding).
 * This entry point replaces upstream main.cpp so it builds with GCC and runs
 * headless (no imshow / system("pause") / Windows fopen_s).
 *
 * Usage:
 *   adcensus_depth <left.png> <right.png> [out_prefix] [min_disp] [max_disp]
 * Defaults: out_prefix = "adcensus_out", min_disp = 0, max_disp = 64.
 */
#include <iostream>
#include <chrono>
#include <cmath>
#include <opencv2/opencv.hpp>

#include "adcensus/ADCensusStereo.h"

using namespace std::chrono;

// Normalize a float disparity map to 8-bit, then save grayscale + JET color.
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
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cout << "Usage: " << argv[0]
                  << " <left> <right> [out_prefix] [min_disp] [max_disp]\n"
                  << "  e.g. " << argv[0]
                  << " left.png right.png adcensus_out 0 128" << std::endl;
        return -1;
    }

    const std::string path_left  = argv[1];
    const std::string path_right = argv[2];
    const std::string out_prefix = (argc >= 4) ? argv[3] : "adcensus_out";

    cv::Mat img_left  = cv::imread(path_left,  cv::IMREAD_COLOR);
    cv::Mat img_right = cv::imread(path_right, cv::IMREAD_COLOR);
    if (img_left.data == nullptr || img_right.data == nullptr) {
        std::cout << "Failed to read input images." << std::endl;
        return -1;
    }
    if (img_left.rows != img_right.rows || img_left.cols != img_right.cols) {
        std::cout << "Left/right image sizes differ." << std::endl;
        return -1;
    }

    const sint32 width  = static_cast<sint32>(img_left.cols);
    const sint32 height = static_cast<sint32>(img_left.rows);

    // Pack BGR into contiguous 3-channel byte buffers (matcher expects this).
    auto bytes_left  = new uint8[static_cast<size_t>(width) * height * 3];
    auto bytes_right = new uint8[static_cast<size_t>(width) * height * 3];
    for (sint32 i = 0; i < height; ++i) {
        for (sint32 j = 0; j < width; ++j) {
            const cv::Vec3b& l = img_left.at<cv::Vec3b>(i, j);
            const cv::Vec3b& r = img_right.at<cv::Vec3b>(i, j);
            const size_t idx = (static_cast<size_t>(i) * width + j) * 3;
            bytes_left[idx + 0]  = l[0]; bytes_left[idx + 1]  = l[1]; bytes_left[idx + 2]  = l[2];
            bytes_right[idx + 0] = r[0]; bytes_right[idx + 1] = r[1]; bytes_right[idx + 2] = r[2];
        }
    }

    ADCensusOption ad_option;
    ad_option.min_disparity = (argc >= 5) ? atoi(argv[4]) : 0;
    ad_option.max_disparity = (argc >= 6) ? atoi(argv[5]) : 64;
    ad_option.lrcheck_thres = 1.0f;
    ad_option.do_lr_check   = true;
    ad_option.do_filling    = true;

    std::cout << "w=" << width << " h=" << height
              << " disp=[" << ad_option.min_disparity << ","
              << ad_option.max_disparity << "]" << std::endl;

    ADCensusStereo ad_census;
    auto t0 = steady_clock::now();
    if (!ad_census.Initialize(width, height, ad_option)) {
        std::cout << "AD-Census Initialize failed." << std::endl;
        delete[] bytes_left; delete[] bytes_right;
        return -2;
    }
    std::cout << "Initialized in "
              << duration_cast<milliseconds>(steady_clock::now() - t0).count() / 1000.0
              << " s" << std::endl;

    auto disparity = new float32[static_cast<size_t>(width) * height]();
    t0 = steady_clock::now();
    if (!ad_census.Match(bytes_left, bytes_right, disparity)) {
        std::cout << "AD-Census Match failed." << std::endl;
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
