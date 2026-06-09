#include "MapBuilder.hpp"

#include <fstream>
#include <cstring>
#include <cstdio>
#include <stdexcept>

static constexpr float DEG2RAD = static_cast<float>(M_PI / 180.0);

// ── Configuration ─────────────────────────────────────────────────────────────

void MapBuilder::setLensFOV(float half_fov_h_deg, float half_fov_v_deg) {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_fov_h = half_fov_h_deg;
    m_fov_v = half_fov_v_deg;
}

// ── Add frame ─────────────────────────────────────────────────────────────────

void MapBuilder::addFrame(float pan_deg, float tilt_deg, const cv::Mat& depth_map) {
    CV_Assert(depth_map.type() == CV_32F);

    const int H = depth_map.rows;
    const int W = depth_map.cols;

    float fov_h, fov_v;
    {
        std::lock_guard<std::mutex> lk(m_mutex);
        fov_h = m_fov_h * DEG2RAD;
        fov_v = m_fov_v * DEG2RAD;
    }

    const float pan_r  = pan_deg  * DEG2RAD;
    const float tilt_r = tilt_deg * DEG2RAD;

    std::vector<Point3f> local;
    local.reserve(H * W / 4);

    for (int v = 0; v < H; ++v) {
        const float* row = depth_map.ptr<float>(v);

        // Vertical angular offset of this pixel row (centre = 0)
        // Guard against single-row images where (H-1) == 0.
        const float beta = (H > 1)
            ? ((float)v / (H - 1) - 0.5f) * 2.f * fov_v
            : 0.f;

        for (int u = 0; u < W; ++u) {
            const float r = row[u];
            if (!std::isfinite(r) || r <= 0.f) continue;

            // Horizontal angular offset (guard single-column images)
            const float alpha = (W > 1)
                ? ((float)u / (W - 1) - 0.5f) * 2.f * fov_h
                : 0.f;

            // World-frame angles
            const float theta = pan_r  + alpha;   // azimuth
            const float phi   = tilt_r + beta;    // elevation

            local.push_back({
                r * std::cos(phi) * std::sin(theta),   // X
                r * std::sin(phi),                     // Y
                r * std::cos(phi) * std::cos(theta)    // Z
            });
        }
    }

    std::lock_guard<std::mutex> lk(m_mutex);
    m_points.insert(m_points.end(), local.begin(), local.end());
}

// ── Query ─────────────────────────────────────────────────────────────────────

size_t MapBuilder::pointCount() const {
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_points.size();
}

cv::Mat MapBuilder::getCloud() const {
    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_points.empty())
        return cv::Mat();

    // Return as (N, 3) CV_32F
    cv::Mat out((int)m_points.size(), 3, CV_32F);
    std::memcpy(out.data, m_points.data(), m_points.size() * sizeof(Point3f));
    return out;
}

void MapBuilder::clear() {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_points.clear();
}

// ── Save ──────────────────────────────────────────────────────────────────────

bool MapBuilder::savePLY(const std::string& path) const {
    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_points.empty()) {
        fprintf(stderr, "[MapBuilder] No points to save.\n");
        return false;
    }

    std::ofstream ofs(path);
    if (!ofs) {
        fprintf(stderr, "[MapBuilder] Cannot open %s for writing\n", path.c_str());
        return false;
    }

    const size_t N = m_points.size();
    ofs << "ply\n"
           "format ascii 1.0\n"
           "element vertex " << N << "\n"
           "property float x\n"
           "property float y\n"
           "property float z\n"
           "end_header\n";

    for (const auto& p : m_points)
        ofs << p.x << ' ' << p.y << ' ' << p.z << '\n';

    printf("[MapBuilder] Saved %zu points to %s\n", N, path.c_str());
    return true;
}

bool MapBuilder::saveNPY(const std::string& path) const {
    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_points.empty()) return false;

    // Minimal .npy header (float32, shape [N, 3], C-order)
    const size_t N = m_points.size();
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) return false;

    // Magic + version
    const char magic[] = "\x93NUMPY\x01\x00";
    ofs.write(magic, 8);

    char header[128];
    int hlen = snprintf(header, sizeof(header),
        "{'descr': '<f4', 'fortran_order': False, 'shape': (%zu, 3), }",
        N);
    // Pad to multiple of 64 with spaces, terminated by newline
    int total = ((hlen + 10 + 63) / 64) * 64;
    std::string hdr(header, hlen);
    hdr.append(total - 10 - hlen, ' ');
    hdr += '\n';

    uint16_t hdr_len = (uint16_t)hdr.size();
    ofs.write(reinterpret_cast<const char*>(&hdr_len), 2);
    ofs.write(hdr.data(), hdr.size());
    ofs.write(reinterpret_cast<const char*>(m_points.data()),
              N * sizeof(Point3f));

    printf("[MapBuilder] Saved %zu points to %s\n", N, path.c_str());
    return true;
}
