#pragma once

#include <vector>
#include <mutex>
#include <string>
#include <unordered_set>
#include <cstdint>
#include <opencv2/opencv.hpp>

#include "../core/Config.hpp"

/**
 * Accumulates depth frames captured at known (pan, tilt) angles into a unified
 * 3-D point cloud.
 *
 * Geometry (plan §3, via core/Reconstruct + core/Geometry):
 *   Body/world axes:  X = forward, Y = right, Z = up.  Units: millimetres.
 *   Per pixel: unproject through K, remap axes, add lever arm, rotate by
 *   R(pan,tilt). This REPLACES the old spherical projection (which double-counted
 *   the angle on a depth map) and the FOV-as-half-FOV bug.
 *
 * Accumulation: a voxel grid (config.voxel_size) deduplicates overlapping points
 * so repeated views of the same surface don't pile up (plan §6/§7).
 *
 * Usage:
 *   MapBuilder builder(cfg);
 *   builder.addFrame(pan_deg, tilt_deg, depth_mm);   // CV_32F mm
 *   builder.savePLY("scan.ply");
 */
class MapBuilder {
public:
    struct Point3f { float x, y, z; };

    MapBuilder() : MapBuilder(Config{}) {}
    explicit MapBuilder(const Config& cfg) : m_cfg(cfg) {}

    void setConfig(const Config& cfg);

    /**
     * Reconstruct a depth map taken at the given pan/tilt and append the new
     * (voxel-deduplicated) points to the cloud. Thread-safe.
     *
     * @param pan_deg   Horizontal servo angle.
     * @param tilt_deg  Vertical   servo angle.
     * @param depth_mm  CV_32F depth image (mm). NaN/out-of-range pixels skipped.
     * @return number of NEW points added (after voxel dedup).
     */
    size_t addFrame(float pan_deg, float tilt_deg, const cv::Mat& depth_mm);

    size_t pointCount() const;
    cv::Mat getCloud() const;                 // (N×3) CV_32F, mm

    bool savePLY(const std::string& path) const;
    bool saveNPY(const std::string& path) const;
    void clear();

private:
    mutable std::mutex            m_mutex;
    std::vector<Point3f>          m_points;
    std::unordered_set<uint64_t>  m_occupied;   // occupied voxel keys (dedup)
    Config                        m_cfg;

    // Quantize a point to a voxel key. Caller holds the lock.
    uint64_t voxelKey(float x, float y, float z) const;
};
