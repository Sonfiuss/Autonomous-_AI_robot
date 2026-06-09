/**
 * Stereo-camera 3D scanner — step-and-hold pipeline mode.
 *
 * Usage:
 *   ./stereo_scan [options]
 *
 * Options:
 *   --port       /dev/ttyUSB0   Serial port connected to ESP32
 *   --left       0              Left  camera index
 *   --right      1              Right camera index
 *   --pan-min   -80.0           Pan  range minimum (deg)
 *   --pan-max    80.0           Pan  range maximum (deg)
 *   --tilt-min  -70.0           Tilt range minimum (deg)
 *   --tilt-max   30.0           Tilt range maximum (deg)
 *   --step-pan   30.0           Pan  step size  (deg)
 *   --step-tilt  20.0           Tilt step size  (deg)
 *   --calib      ""             Calibration .yml path (optional)
 *   --output     scan.ply       Output point cloud path
 *   --config     scan.config    Checkpoint file (enables resume on restart)
 */

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <csignal>

#include "control/ServoController.hpp"
#include "vision/StereoCamera.hpp"
#include "vision/MapBuilder.hpp"
#include "pipeline/Scanner.hpp"

// ── Args ──────────────────────────────────────────────────────────────────────

struct Args {
    std::string port        = "/dev/ttyUSB0";
    int         left_cam    = 0;
    int         right_cam   = 1;  // Jetson UVC: each camera occupies 2 nodes (capture+metadata)
    float       pan_min     = -80.f;
    float       pan_max     =  80.f;
    float       tilt_min    = -70.f;
    float       tilt_max    =  30.f;
    float       step_pan    =  30.f;
    float       step_tilt   =  20.f;
    std::string calib_path;
    std::string output      = "scan.ply";
    std::string config_path = "scan.config";
};

static bool parseArgs(int argc, char* argv[], Args& out) {
    for (int i = 1; i < argc; ++i) {
        auto eq = [&](const char* name) {
            return strcmp(argv[i], name) == 0 && i + 1 < argc;
        };
        if      (eq("--port"))       out.port        = argv[++i];
        else if (eq("--left"))       out.left_cam    = std::atoi(argv[++i]);
        else if (eq("--right"))      out.right_cam   = std::atoi(argv[++i]);
        else if (eq("--pan-min"))    out.pan_min     = std::atof(argv[++i]);
        else if (eq("--pan-max"))    out.pan_max     = std::atof(argv[++i]);
        else if (eq("--tilt-min"))   out.tilt_min    = std::atof(argv[++i]);
        else if (eq("--tilt-max"))   out.tilt_max    = std::atof(argv[++i]);
        else if (eq("--step-pan"))   out.step_pan    = std::atof(argv[++i]);
        else if (eq("--step-tilt"))  out.step_tilt   = std::atof(argv[++i]);
        else if (eq("--calib"))      out.calib_path  = argv[++i];
        else if (eq("--output"))     out.output      = argv[++i];
        else if (eq("--config"))     out.config_path = argv[++i];
        else if (strcmp(argv[i], "--help") == 0) return false;
        else { fprintf(stderr, "Unknown argument: %s\n", argv[i]); return false; }
    }
    return true;
}

static void printUsage(const char* prog) {
    fprintf(stderr,
        "Usage: %s [--port dev] [--left N] [--right N]\n"
        "          [--pan-min deg] [--pan-max deg]\n"
        "          [--tilt-min deg] [--tilt-max deg]\n"
        "          [--step-pan deg] [--step-tilt deg]\n"
        "          [--calib file.yml] [--output file.ply]\n"
        "          [--config scan.config]\n", prog);
}

// ── Main ──────────────────────────────────────────────────────────────────────

int main(int argc, char* argv[]) {
    Args args;
    if (!parseArgs(argc, argv, args)) {
        printUsage(argv[0]);
        return 1;
    }

    // Build scan plan and print summary
    Scanner::Config scan_cfg;
    scan_cfg.pan_min    = args.pan_min;
    scan_cfg.pan_max    = args.pan_max;
    scan_cfg.tilt_min   = args.tilt_min;
    scan_cfg.tilt_max   = args.tilt_max;
    scan_cfg.step_pan   = args.step_pan;
    scan_cfg.step_tilt  = args.step_tilt;
    scan_cfg.config_path = args.config_path;
    scan_cfg.output_path = args.output;

    auto raster = Scanner::buildRaster(scan_cfg);

    size_t resume_idx = 0;
    bool resuming = Scanner::loadConfig(args.config_path, resume_idx)
                    && resume_idx > 0
                    && resume_idx < raster.size();

    printf("=== Stereo Scan (step-and-hold) ===\n");
    printf("  Port:       %s\n",   args.port.c_str());
    printf("  Cameras:    %d / %d\n", args.left_cam, args.right_cam);
    printf("  Pan:        %.0f° → %.0f°  step %.0f°\n",
           args.pan_min, args.pan_max, args.step_pan);
    printf("  Tilt:       %.0f° → %.0f°  step %.0f°\n",
           args.tilt_min, args.tilt_max, args.step_tilt);
    printf("  Total steps:%zu  (~%.0f s)\n", raster.size(), raster.size() * 0.5);
    if (resuming)
        printf("  Resuming:   from step %zu\n", resume_idx);
    printf("  Output:     %s\n\n", args.output.c_str());

    // Open hardware
    ServoController servo(args.port);
    if (!servo.connect()) {
        fprintf(stderr, "Failed to connect to ESP32 on %s\n", args.port.c_str());
        return 1;
    }

    StereoCamera::Config cam_cfg;
    cam_cfg.left_idx  = args.left_cam;
    cam_cfg.right_idx = args.right_cam;
    StereoCamera camera(cam_cfg);

    if (!camera.open()) {
        fprintf(stderr, "Failed to open cameras\n");
        servo.disconnect();
        return 1;
    }
    if (!args.calib_path.empty())
        camera.loadCalibration(args.calib_path);

    MapBuilder builder;

    // Run 3-thread pipeline
    Scanner scanner(servo, camera, builder);
    scanner.run(scan_cfg);

    // Return servos to home
    printf("Resetting servos to home…\n");
    servo.reset();
    servo.disconnect();
    camera.release();

    return 0;
}
