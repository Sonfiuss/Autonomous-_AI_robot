#include "AngleBuffer.hpp"
#include <cmath>
#include <algorithm>

void AngleBuffer::push(double t_ms, float pan, float tilt) {
    std::lock_guard<std::mutex> lk(m_mtx);
    if (m_buf.size() < m_cap) {
        m_buf.push_back({t_ms, pan, tilt});
        m_head = m_buf.size() % m_cap;
        m_count = m_buf.size();
    } else {
        m_buf[m_head] = {t_ms, pan, tilt};
        m_head = (m_head + 1) % m_cap;
        // m_count stays at m_cap (oldest evicted)
    }
}

size_t AngleBuffer::size() const {
    std::lock_guard<std::mutex> lk(m_mtx);
    return m_count;
}

void AngleBuffer::clear() {
    std::lock_guard<std::mutex> lk(m_mtx);
    m_buf.clear();
    m_head = 0;
    m_count = 0;
}

AngleBuffer::Query AngleBuffer::angleAt(double t_ms) const {
    std::lock_guard<std::mutex> lk(m_mtx);
    Query q;
    if (m_count == 0) return q;            // invalid: empty

    const Sample& oldest = m_buf[at(0)];
    const Sample& newest = m_buf[at(m_count - 1)];

    // Single sample: return it.
    if (m_count == 1) {
        q.pan = oldest.pan; q.tilt = oldest.tilt;
        q.gap_ms = std::fabs(t_ms - oldest.t_ms);
        q.valid = true;
        q.extrapolated = true;
        return q;
    }

    // Clamp to range -> extrapolation (nearest end).
    if (t_ms <= oldest.t_ms) {
        q.pan = oldest.pan; q.tilt = oldest.tilt;
        q.gap_ms = oldest.t_ms - t_ms;
        q.valid = true; q.extrapolated = true;
        return q;
    }
    if (t_ms >= newest.t_ms) {
        q.pan = newest.pan; q.tilt = newest.tilt;
        q.gap_ms = t_ms - newest.t_ms;
        q.valid = true; q.extrapolated = true;
        return q;
    }

    // Binary search for the bracketing pair [lo, lo+1] with t in between.
    // Samples are time-ordered by construction (monotonic clock).
    size_t lo = 0, hi = m_count - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (m_buf[at(mid)].t_ms <= t_ms) lo = mid; else hi = mid;
    }
    const Sample& a = m_buf[at(lo)];
    const Sample& b = m_buf[at(hi)];

    const double span = b.t_ms - a.t_ms;
    const double w = (span > 0.0) ? (t_ms - a.t_ms) / span : 0.0;

    q.pan  = (float)(a.pan  + w * (b.pan  - a.pan));
    q.tilt = (float)(a.tilt + w * (b.tilt - a.tilt));
    q.gap_ms = std::min(t_ms - a.t_ms, b.t_ms - t_ms);
    q.valid = true;
    q.extrapolated = false;
    return q;
}
