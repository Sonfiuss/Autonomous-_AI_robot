#pragma once

#include <string>
#include <vector>
#include <utility>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>

#include "FrameQueue.hpp"
#include "../core/Config.hpp"
#include "../core/AngleBuffer.hpp"

class ServoController;
class StereoCamera;
class MapBuilder;

/**
 * 3-thread pipelined scanner.
 *
 * Thread A (caller thread): snake-raster servo control.
 *   - moveTo(pan, tilt) — blocks ~400 ms for servo settle + OK from ESP32
 *   - writes scan.config checkpoint after each step
 *   - signals Thread B immediately on OK
 *
 * Thread B: depth capture.
 *   - fires on Thread A's signal; does NOT wait for Thread A's next step
 *   - captureDepth() (~150 ms SGBM) overlaps with next moveTo()
 *   - pushes FramePacket to FrameQueue; no sleep
 *
 * Thread C: map building.
 *   - independent consumer of the FrameQueue
 *   - addFrame() accumulates 3-D points; saves PLY on queue close
 *
 * Timing: effective scan rate ≈ 1 frame / 400 ms  (servo-limited).
 */
class Scanner {
public:
    struct Config {
        float pan_min    = -80.f;
        float pan_max    =  80.f;
        float tilt_min   = -70.f;
        float tilt_max   =  30.f;
        float step_pan   =  30.f;
        float step_tilt  =  20.f;
        // Stop-and-shoot 3-pan tiling (plan §6): when true, pan positions are the
        // fixed FOV-tiling centres {-44, 0, +44}° instead of stepping by step_pan.
        bool  pan_tiling = false;
        std::string config_path = "scan.config";
        std::string output_path = "scan.ply";
    };

    // Fixed pan-tiling centres (plan §6) for ~±80° horizontal coverage.
    static constexpr float kPanTiles[3] = {-44.f, 0.f, 44.f};

    Scanner(ServoController& servo, StereoCamera& camera, MapBuilder& builder);

    /** Run full stop-and-shoot scan; block until the point cloud is saved. */
    void run(const Config& cfg);

    /**
     * Continuous-sweep mode (plan §6/§8, optional). Cameras free-run while the
     * head sweeps; each frame is paired with the INTERPOLATED angle at
     * (t_frame + t_offset) from `angles`, gap-rejected, then reconstructed.
     *
     * Gated on core.shutter_synced (requires global-shutter + genlock). The
     * `angles` buffer must be fed concurrently by a firmware reader thread
     * (P-feedback -> AngleBuffer::push). NOTE: the current ServoController is
     * step-and-hold only and does not stream P-feedback; wiring that reader is a
     * pending firmware task. Returns number of frames accumulated.
     *
     * @param n_frames  number of free-running frames to capture.
     */
    size_t runContinuous(const ::Config& core, AngleBuffer& angles,
                         size_t n_frames, const std::string& output_path);

    // ── Helpers exposed for unit tests ────────────────────────────────────────

    /** Build ordered (pan°, tilt°) snake-raster positions. */
    static std::vector<std::pair<float,float>> buildRaster(const Config& cfg);

    /**
     * Load checkpoint. Returns false if the file is absent or malformed.
     * resume_idx is the raster index to start from on the next run.
     */
    static bool loadConfig(const std::string& path, size_t& resume_idx);

    /** Write checkpoint file. idx is the NEXT index to execute. */
    static void saveConfig(const std::string& path, size_t idx,
                           float pan, float tilt);

private:
    ServoController& m_servo;
    StereoCamera&    m_camera;
    MapBuilder&      m_builder;

    struct FramePacket {
        float   pan, tilt;
        cv::Mat depth;
    };

    struct CaptureSignal {
        std::mutex              mtx;
        std::condition_variable cv;
        bool  pending = false;   // Thread B has a frame to capture
        bool  done    = false;   // Thread A finished all steps
        float pan     = 0.f;
        float tilt    = 0.f;
    };

    // Thread A runs in the caller's thread via run()
    void threadA(const Config& cfg, CaptureSignal& sig,
                 FrameQueue<FramePacket>& q);
    void threadB(CaptureSignal& sig, FrameQueue<FramePacket>& q);
    void threadC(FrameQueue<FramePacket>& q, const std::string& output_path);
};
