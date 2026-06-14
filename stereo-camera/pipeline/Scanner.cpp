#include "Scanner.hpp"

#include <cstdio>
#include <cmath>
#include <thread>
#include <fstream>
#include <algorithm>

#include "../control/ServoController.hpp"
#include "../vision/StereoCamera.hpp"
#include "../vision/MapBuilder.hpp"

Scanner::Scanner(ServoController& servo, StereoCamera& camera, MapBuilder& builder)
    : m_servo(servo), m_camera(camera), m_builder(builder) {}

// ── Raster ────────────────────────────────────────────────────────────────────

std::vector<std::pair<float,float>> Scanner::buildRaster(const Config& cfg) {
    std::vector<std::pair<float,float>> pts;
    bool forward = true;

    for (float tilt = cfg.tilt_min;
         tilt <= cfg.tilt_max + 0.01f;
         tilt += cfg.step_tilt)
    {
        const float t = std::min(tilt, cfg.tilt_max);

        std::vector<float> pans;
        if (cfg.pan_tiling) {
            // Fixed FOV-tiling centres (plan §6).
            for (float p : Scanner::kPanTiles) pans.push_back(p);
        } else {
            for (float pan = cfg.pan_min;
                 pan <= cfg.pan_max + 0.01f;
                 pan += cfg.step_pan)
                pans.push_back(std::min(pan, cfg.pan_max));
        }

        if (!forward)
            std::reverse(pans.begin(), pans.end());

        for (float p : pans)
            pts.push_back({p, t});

        forward = !forward;
    }
    return pts;
}

// ── Config I/O ────────────────────────────────────────────────────────────────

bool Scanner::loadConfig(const std::string& path, size_t& resume_idx) {
    std::ifstream f(path);
    if (!f.is_open()) return false;

    std::string line;
    while (std::getline(f, line)) {
        if (line.rfind("step_idx=", 0) == 0) {
            try {
                resume_idx = static_cast<size_t>(std::stoul(line.substr(9)));
                return true;
            } catch (...) {}
        }
    }
    return false;
}

void Scanner::saveConfig(const std::string& path, size_t idx,
                         float pan, float tilt) {
    std::ofstream f(path);
    f << "step_idx=" << idx  << "\n"
      << "pan="      << pan  << "\n"
      << "tilt="     << tilt << "\n";
}

// ── Thread A ──────────────────────────────────────────────────────────────────

void Scanner::threadA(const Config& cfg, CaptureSignal& sig,
                      FrameQueue<FramePacket>& q) {
    auto raster = buildRaster(cfg);
    if (raster.empty()) {
        std::lock_guard<std::mutex> lk(sig.mtx);
        sig.done = true;
        sig.cv.notify_one();
        return;
    }

    size_t start = 0;
    if (loadConfig(cfg.config_path, start)) {
        if (start >= raster.size()) start = 0;
        printf("[Scanner] Resuming from step %zu / %zu\n", start, raster.size());
    }

    for (size_t i = start; i < raster.size(); ++i) {
        auto [pan, tilt] = raster[i];
        printf("[Scanner] Step %zu/%zu  pan=%+.1f°  tilt=%+.1f°\n",
               i + 1, raster.size(), pan, tilt);

        if (!m_servo.moveTo(pan, tilt)) {
            fprintf(stderr, "[Scanner] moveTo failed at step %zu — aborting\n", i);
            break;
        }

        // Checkpoint: next step to run if we resume
        saveConfig(cfg.config_path, i + 1, pan, tilt);

        // Signal Thread B immediately — don't wait for capture to finish
        {
            std::lock_guard<std::mutex> lk(sig.mtx);
            sig.pan     = pan;
            sig.tilt    = tilt;
            sig.pending = true;
        }
        sig.cv.notify_one();
    }

    // Tell Thread B to stop
    {
        std::lock_guard<std::mutex> lk(sig.mtx);
        sig.done = true;
    }
    sig.cv.notify_one();
}

// ── Thread B ──────────────────────────────────────────────────────────────────

void Scanner::threadB(CaptureSignal& sig, FrameQueue<FramePacket>& q) {
    while (true) {
        float pan, tilt;
        {
            std::unique_lock<std::mutex> lk(sig.mtx);
            sig.cv.wait(lk, [&sig]{ return sig.pending || sig.done; });

            if (sig.done && !sig.pending) break;

            pan  = sig.pan;
            tilt = sig.tilt;
            sig.pending = false;
        }

        cv::Mat depth;
        if (m_camera.captureDepth(depth)) {
            q.push({pan, tilt, std::move(depth)});
        } else {
            fprintf(stderr,
                    "[Scanner] captureDepth failed  pan=%+.1f° tilt=%+.1f°\n",
                    pan, tilt);
        }
    }
}

// ── Thread C ──────────────────────────────────────────────────────────────────

void Scanner::threadC(FrameQueue<FramePacket>& q,
                      const std::string& output_path) {
    FramePacket fp;
    while (q.pop(fp)) {
        m_builder.addFrame(fp.pan, fp.tilt, fp.depth);
        printf("[Scanner] Map: %zu pts\r", m_builder.pointCount());
        fflush(stdout);
    }
    printf("\n[Scanner] Map done: %zu points\n", m_builder.pointCount());

    if (m_builder.pointCount() > 0) {
        if (output_path.size() >= 4 &&
            output_path.substr(output_path.size() - 4) == ".npy")
            m_builder.saveNPY(output_path);
        else
            m_builder.savePLY(output_path);
    } else {
        fprintf(stderr, "[Scanner] No points captured — output not written\n");
    }
}

// ── run ───────────────────────────────────────────────────────────────────────

void Scanner::run(const Config& cfg) {
    CaptureSignal sig;
    FrameQueue<FramePacket> q(32);

    std::thread tb([&]{ threadB(sig, q); });
    std::thread tc([&]{ threadC(q, cfg.output_path); });

    threadA(cfg, sig, q);   // blocks until all steps complete

    tb.join();
    q.close();              // drain: Thread C saves PLY then exits
    tc.join();
}

// ── Continuous-sweep mode (plan §6/§8) ─────────────────────────────────────────

size_t Scanner::runContinuous(const ::Config& core, AngleBuffer& angles,
                              size_t n_frames, const std::string& output_path) {
    if (!core.shutter_synced) {
        fprintf(stderr,
            "[Scanner] runContinuous refused: core.shutter_synced is false.\n"
            "  Continuous mode needs a global-shutter + genlocked stereo rig\n"
            "  (plan §6). Use stop-and-shoot run() instead.\n");
        return 0;
    }

    m_builder.setConfig(core);

    size_t accepted = 0, discarded = 0;
    for (size_t i = 0; i < n_frames; ++i) {
        cv::Mat depth;
        double t_ms = 0.0;
        if (!m_camera.captureDepth(depth, t_ms)) {
            fprintf(stderr, "[Scanner] continuous: capture failed at frame %zu\n", i);
            continue;
        }
        // Pair with interpolated angle at the calibrated capture instant.
        auto q = angles.angleAt(t_ms + core.t_offset_ms);
        if (!q.valid || q.gap_ms > core.angle_gap_threshold_ms) {
            ++discarded;     // no trustworthy angle for this frame
            continue;
        }
        m_builder.addFrame(q.pan, q.tilt, depth);
        ++accepted;
    }

    printf("[Scanner] continuous: %zu accepted, %zu discarded (angle gap)\n",
           accepted, discarded);
    if (m_builder.pointCount() > 0) {
        if (output_path.size() >= 4 &&
            output_path.substr(output_path.size() - 4) == ".npy")
            m_builder.saveNPY(output_path);
        else
            m_builder.savePLY(output_path);
    }
    return accepted;
}
