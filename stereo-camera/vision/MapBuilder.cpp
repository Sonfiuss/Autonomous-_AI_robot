#include "MapBuilder.hpp"
#include "../core/Reconstruct.hpp"

#include <fstream>
#include <cstring>
#include <cstdio>
#include <cmath>

// ── Configuration ─────────────────────────────────────────────────────────────

void MapBuilder::setConfig(const Config& cfg) {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_cfg = cfg;
}

// ── Voxel key ─────────────────────────────────────────────────────────────────
// Quantize to voxel index and pack three 21-bit signed indices into 63 bits.
uint64_t MapBuilder::voxelKey(float x, float y, float z) const {
    const float vs = (m_cfg.voxel_size > 0.f) ? m_cfg.voxel_size : 1.f;
    auto q = [vs](float c) -> int64_t {
        int64_t i = (int64_t)std::llround(std::floor(c / vs));
        // bias into unsigned 21-bit range [0, 2^21)
        return (i + (1 << 20)) & 0x1FFFFF;
    };
    const uint64_t ix = (uint64_t)q(x);
    const uint64_t iy = (uint64_t)q(y);
    const uint64_t iz = (uint64_t)q(z);
    return (ix << 42) | (iy << 21) | iz;
}

// ── Add frame ─────────────────────────────────────────────────────────────────

size_t MapBuilder::addFrame(float pan_deg, float tilt_deg, const cv::Mat& depth_mm) {
    CV_Assert(depth_mm.type() == CV_32F);

    Config cfg;
    { std::lock_guard<std::mutex> lk(m_mutex); cfg = m_cfg; }

    // Reconstruct outside the lock (the expensive part).
    auto world = recon::reconstruct(depth_mm, pan_deg, tilt_deg, cfg);

    std::lock_guard<std::mutex> lk(m_mutex);
    size_t added = 0;
    for (const auto& p : world) {
        if (m_points.size() >= m_cfg.max_points) {
            fprintf(stderr, "[MapBuilder] max_points (%zu) reached — capping cloud\n",
                    m_cfg.max_points);
            break;
        }
        const uint64_t key = voxelKey(p[0], p[1], p[2]);
        if (m_occupied.insert(key).second) {       // new voxel -> keep point
            m_points.push_back({p[0], p[1], p[2]});
            ++added;
        }
    }
    return added;
}

// ── Query ─────────────────────────────────────────────────────────────────────

size_t MapBuilder::pointCount() const {
    std::lock_guard<std::mutex> lk(m_mutex);
    return m_points.size();
}

cv::Mat MapBuilder::getCloud() const {
    std::lock_guard<std::mutex> lk(m_mutex);
    if (m_points.empty()) return cv::Mat();
    cv::Mat out((int)m_points.size(), 3, CV_32F);
    std::memcpy(out.data, m_points.data(), m_points.size() * sizeof(Point3f));
    return out;
}

void MapBuilder::clear() {
    std::lock_guard<std::mutex> lk(m_mutex);
    m_points.clear();
    m_occupied.clear();
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

    const size_t N = m_points.size();
    std::ofstream ofs(path, std::ios::binary);
    if (!ofs) return false;

    const char magic[] = "\x93NUMPY\x01\x00";
    ofs.write(magic, 8);

    char header[128];
    int hlen = snprintf(header, sizeof(header),
        "{'descr': '<f4', 'fortran_order': False, 'shape': (%zu, 3), }", N);
    int total = ((hlen + 10 + 63) / 64) * 64;
    std::string hdr(header, hlen);
    hdr.append(total - 10 - hlen, ' ');
    hdr += '\n';

    uint16_t hdr_len = (uint16_t)hdr.size();
    ofs.write(reinterpret_cast<const char*>(&hdr_len), 2);
    ofs.write(hdr.data(), hdr.size());
    ofs.write(reinterpret_cast<const char*>(m_points.data()), N * sizeof(Point3f));

    printf("[MapBuilder] Saved %zu points to %s\n", N, path.c_str());
    return true;
}
