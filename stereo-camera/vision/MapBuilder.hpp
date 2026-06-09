#pragma once

#include <vector>
#include <mutex>
#include <string>
#include <cmath>
#include <opencv2/opencv.hpp>

/**
 * Accumulates depth frames captured at known (pan, tilt) angles and projects
 * them into a unified 3-D point cloud using spherical-to-Cartesian conversion.
 *
 * Coordinate system (right-hand, Z forward):
 *   X  =  right
 *   Y  =  up
 *   Z  =  forward (away from camera at pan=0, tilt=0)
 *
 * Usage:
 *   MapBuilder builder;
 *   builder.setLensFOV(70.f, 50.f);       // horizontal / vertical half-FOV
 *   builder.addFrame(pan_deg, tilt_deg, depth_map);
 *   builder.savePLY("scan.ply");
 */
class MapBuilder {
public:
    struct Point3f { float x, y, z; };

    MapBuilder() = default;

    /** Set the camera lens half-FOV in degrees (default: 70° H, 50° V). */
    void setLensFOV(float half_fov_h_deg, float half_fov_v_deg);

    /**
     * Project a depth map taken at the given pan/tilt angles and append to the
     * internal cloud. Thread-safe.
     *
     * @param pan_deg   Horizontal servo angle  (positive = right).
     * @param tilt_deg  Vertical   servo angle  (positive = up).
     * @param depth_map CV_32F depth image (metres). NaN pixels are skipped.
     */
    void addFrame(float pan_deg, float tilt_deg, const cv::Mat& depth_map);

    /** Total number of valid points accumulated so far. */
    size_t pointCount() const;

    /** Copy accumulated cloud as (N×3) float matrix. */
    cv::Mat getCloud() const;

    /**
     * Save cloud as ASCII PLY.
     * Falls back to XYZ if path has neither .ply nor .pcd extension.
     */
    bool savePLY(const std::string& path) const;

    /** Save raw float32 binary: 4 bytes × 3 channels × N points (row-major). */
    bool saveNPY(const std::string& path) const;

    void clear();

private:
    mutable std::mutex          m_mutex;
    std::vector<Point3f>        m_points;

    float m_fov_h = 70.f;   // half-FOV degrees
    float m_fov_v = 50.f;
};
