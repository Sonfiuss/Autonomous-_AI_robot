// astra_probe.cpp — smoke test: open Astra Pro depth via OpenNI2, grab N frames,
// print stats, save raw 16-bit depth (mm) + a colorized preview. Headless-friendly.
//
// Build:
//   g++ astra_probe.cpp -o astra_probe \
//       -I/usr/include/openni2 -lOpenNI2 `pkg-config --cflags --libs opencv4`
// Run:
//   ./astra_probe            # saves to ./ (astra_depth_raw.png, astra_depth_vis.png)
//   ./astra_probe 30 out/    # warm up 30 frames, save into out/

#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <cstdio>
#include <string>

using namespace openni;

static const char* fmtName(PixelFormat f) {
    switch (f) {
        case PIXEL_FORMAT_DEPTH_1_MM:   return "DEPTH_1_MM";
        case PIXEL_FORMAT_DEPTH_100_UM: return "DEPTH_100_UM";
        default:                        return "OTHER";
    }
}

int main(int argc, char** argv) {
    int warmup = (argc > 1) ? atoi(argv[1]) : 10;
    std::string outdir = (argc > 2) ? argv[2] : "./";
    if (!outdir.empty() && outdir.back() != '/') outdir += '/';

    if (OpenNI::initialize() != STATUS_OK) {
        fprintf(stderr, "OpenNI init failed: %s\n", OpenNI::getExtendedError());
        return 1;
    }

    // List devices.
    openni::Array<DeviceInfo> devs;
    OpenNI::enumerateDevices(&devs);
    printf("OpenNI2 devices found: %d\n", devs.getSize());
    for (int i = 0; i < devs.getSize(); ++i)
        printf("  [%d] %s | %s | uri=%s\n", i,
               devs[i].getName(), devs[i].getVendor(), devs[i].getUri());
    if (devs.getSize() == 0) {
        fprintf(stderr, "No OpenNI2 device. Is the Astra depth (2bc5:0501) plugged in "
                        "and are you allowed to access it (udev/root)?\n");
        OpenNI::shutdown();
        return 2;
    }

    Device dev;
    if (dev.open(ANY_DEVICE) != STATUS_OK) {
        fprintf(stderr, "Device open failed: %s\n", OpenNI::getExtendedError());
        OpenNI::shutdown();
        return 3;
    }

    VideoStream depth;
    if (depth.create(dev, SENSOR_DEPTH) != STATUS_OK) {
        fprintf(stderr, "Depth stream create failed: %s\n", OpenNI::getExtendedError());
        dev.close(); OpenNI::shutdown();
        return 4;
    }

    VideoMode vm = depth.getVideoMode();
    printf("depth mode: %dx%d @ %d fps, pixfmt=%s\n",
           vm.getResolutionX(), vm.getResolutionY(), vm.getFps(), fmtName(vm.getPixelFormat()));
    printf("HFOV=%.1f deg  VFOV=%.1f deg\n",
           depth.getHorizontalFieldOfView() * 180.0 / M_PI,
           depth.getVerticalFieldOfView()   * 180.0 / M_PI);

    if (depth.start() != STATUS_OK) {
        fprintf(stderr, "Depth start failed: %s\n", OpenNI::getExtendedError());
        depth.destroy(); dev.close(); OpenNI::shutdown();
        return 5;
    }

    VideoFrameRef frame;
    int valid = 0;
    for (int n = 0; n < warmup + 1; ++n) {
        int idx = -1;
        VideoStream* ps = &depth;
        if (OpenNI::waitForAnyStream(&ps, 1, &idx, 2000) != STATUS_OK) {
            fprintf(stderr, "waitForAnyStream timeout at frame %d\n", n);
            continue;
        }
        depth.readFrame(&frame);
    }

    if (!frame.isValid()) {
        fprintf(stderr, "No valid frame captured.\n");
        depth.stop(); depth.destroy(); dev.close(); OpenNI::shutdown();
        return 6;
    }

    const int W = frame.getWidth(), H = frame.getHeight();
    cv::Mat raw(H, W, CV_16UC1, (void*)frame.getData());
    cv::Mat raw16 = raw.clone();  // own the data before frame is reused/destroyed

    // Stats over valid (non-zero) pixels.
    double mn = 1e9, mx = 0; long long sum = 0;
    for (int y = 0; y < H; ++y)
        for (int x = 0; x < W; ++x) {
            uint16_t d = raw16.at<uint16_t>(y, x);
            if (d == 0) continue;
            valid++; sum += d;
            if (d < mn) mn = d; if (d > mx) mx = d;
        }
    uint16_t center = raw16.at<uint16_t>(H / 2, W / 2);
    printf("frame %dx%d | valid=%d (%.1f%%) | min=%.0fmm max=%.0fmm mean=%.0fmm | center=%dmm\n",
           W, H, valid, 100.0 * valid / (W * H),
           valid ? mn : 0, mx, valid ? (double)sum / valid : 0, center);

    // Save raw 16-bit (mm) + colorized 8-bit preview.
    std::string rawpath = outdir + "astra_depth_raw.png";
    std::string vispath = outdir + "astra_depth_vis.png";
    cv::imwrite(rawpath, raw16);
    cv::Mat vis8, viscol;
    raw16.convertTo(vis8, CV_8UC1, 255.0 / (mx > 0 ? mx : 1));
    cv::applyColorMap(vis8, viscol, cv::COLORMAP_JET);
    viscol.setTo(cv::Scalar(0, 0, 0), raw16 == 0);  // invalid = black
    cv::imwrite(vispath, viscol);
    printf("saved: %s (raw mm)\n       %s (colorized)\n", rawpath.c_str(), vispath.c_str());

    depth.stop(); depth.destroy(); dev.close(); OpenNI::shutdown();
    return 0;
}
