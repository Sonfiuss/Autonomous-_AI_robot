/**
 * check_camera.cpp — Headless camera sanity check for deepmap on Jetson.
 *
 * Default mode: captures frames for a few seconds, measures real FPS,
 * saves the last frame as frame.jpg. No display required.
 *
 * Compile:
 *   g++ check_camera.cpp -o check_camera \
 *       $(pkg-config --cflags --libs opencv4) -std=c++17
 *
 * Usage:
 *   ./check_camera                  # check camera 0, save frame.jpg
 *   ./check_camera 1                # check camera index 1
 *   ./check_camera 0 --frames 60    # capture 60 frames then report
 *   ./check_camera 0 --out snap.jpg # custom output filename
 *   ./check_camera 0 --preview      # live window (needs X11/display)
 */

#include <chrono>
#include <cstdio>
#include <cstring>
#include <string>
#include <opencv2/opencv.hpp>

static void usage(const char* prog) {
    printf("Usage: %s [cam_index] [--frames N] [--out FILE] [--preview]\n", prog);
}

int main(int argc, char* argv[])
{
    int         cam_idx    = 0;
    int         n_frames   = 30;
    std::string out_file   = "frame.jpg";
    bool        preview    = false;

    for (int i = 1; i < argc; ++i) {
        if (std::strcmp(argv[i], "--frames") == 0 && i + 1 < argc)
            n_frames = std::atoi(argv[++i]);
        else if (std::strcmp(argv[i], "--out") == 0 && i + 1 < argc)
            out_file = argv[++i];
        else if (std::strcmp(argv[i], "--preview") == 0)
            preview = true;
        else if (std::strcmp(argv[i], "--help") == 0) {
            usage(argv[0]); return 0;
        } else if (argv[i][0] != '-') {
            cam_idx = std::atoi(argv[i]);
        }
    }

    printf("Opening camera %d ...\n", cam_idx);
    cv::VideoCapture cap(cam_idx, cv::CAP_V4L2);

    if (!cap.isOpened()) {
        fprintf(stderr, "ERROR: Cannot open camera %d\n", cam_idx);
        return 1;
    }

    // Report properties reported by driver
    double w   = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    double h   = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    double fps = cap.get(cv::CAP_PROP_FPS);
    printf("Driver reports: %.0f x %.0f @ %.1f FPS\n", w, h, fps);

    // Warm up (first frames often dark/incomplete on USB cameras)
    cv::Mat frame;
    for (int i = 0; i < 5; ++i) cap.read(frame);

    if (frame.empty()) {
        fprintf(stderr, "ERROR: Camera opened but produced no frames\n");
        return 1;
    }

    // Measure real FPS over n_frames
    printf("Capturing %d frames to measure real FPS...\n", n_frames);
    auto t0 = std::chrono::steady_clock::now();
    int good = 0;
    for (int i = 0; i < n_frames; ++i) {
        if (cap.read(frame) && !frame.empty()) ++good;
    }
    auto t1 = std::chrono::steady_clock::now();
    double elapsed = std::chrono::duration<double>(t1 - t0).count();
    double real_fps = good / elapsed;

    printf("Result : %d/%d frames OK  |  real FPS = %.1f  |  %.0f x %.0f px\n",
           good, n_frames, real_fps, (double)frame.cols, (double)frame.rows);

    // Save last frame
    if (!frame.empty()) {
        cv::imwrite(out_file, frame);
        printf("Saved  : %s\n", out_file.c_str());
    }

    // Optional live preview — only if --preview passed and display available
    if (preview) {
        const char* disp = std::getenv("DISPLAY");
        if (!disp || disp[0] == '\0') {
            fprintf(stderr, "WARNING: DISPLAY not set — cannot open preview window. "
                            "Use SSH -X or omit --preview.\n");
        } else {
            const std::string win = "Camera " + std::to_string(cam_idx)
                                  + "  |  q/ESC quit  |  s save";
            cv::namedWindow(win, cv::WINDOW_NORMAL);
            printf("Preview open — press q/ESC to quit, s to save frame\n");
            int count = 0;
            while (true) {
                if (!cap.read(frame) || frame.empty()) continue;
                ++count;
                char info[64];
                std::snprintf(info, sizeof(info), "#%d  %dx%d", count, frame.cols, frame.rows);
                cv::putText(frame, info, {8, 28},
                            cv::FONT_HERSHEY_SIMPLEX, 0.8, {0, 255, 0}, 2);
                cv::imshow(win, frame);
                int key = cv::waitKey(1) & 0xFF;
                if (key == 'q' || key == 27) break;
                if (key == 's') {
                    cv::imwrite(out_file, frame);
                    printf("Saved: %s\n", out_file.c_str());
                }
            }
            cv::destroyAllWindows();
        }
    }

    cap.release();
    return 0;
}
