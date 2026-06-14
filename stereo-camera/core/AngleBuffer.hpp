#pragma once

#include <vector>
#include <cstddef>
#include <mutex>

/**
 * Fixed-size ring buffer of timestamped (pan, tilt) samples pushed by the
 * firmware reader (~50–100 Hz). Provides angle_at(t) via linear interpolation
 * between the two bracketing samples, plus the time gap to the nearest sample
 * so callers can reject frames captured during an angle gap.
 *
 * Timestamps are milliseconds on a single monotonic clock (see Config contract:
 * stamp frames and angles from the SAME clock; apply config.t_offset_ms before
 * querying if they differ).
 *
 * Thread-safe: one producer (push) + many readers (angleAt).
 */
class AngleBuffer {
public:
    struct Sample { double t_ms; float pan; float tilt; };

    struct Query {
        float  pan      = 0.f;
        float  tilt     = 0.f;
        double gap_ms   = 0.0;     // |t - nearest sample time|
        bool   valid    = false;   // false if buffer empty or t out of range with no bracket
        bool   extrapolated = false; // t outside [oldest, newest]: clamped to nearest
    };

    explicit AngleBuffer(size_t capacity = 4096)
        : m_cap(capacity ? capacity : 1) { m_buf.reserve(m_cap); }

    /** Insert a sample. O(1). Evicts the oldest when full (ring wrap). */
    void push(double t_ms, float pan, float tilt);

    /** Linear-interpolated angle at time t_ms. */
    Query angleAt(double t_ms) const;

    size_t size() const;
    void   clear();

private:
    mutable std::mutex   m_mtx;
    std::vector<Sample>  m_buf;     // logical ring; physical size grows to m_cap
    size_t               m_cap;
    size_t               m_head = 0;   // index of next write
    size_t               m_count = 0;  // number of valid samples

    // physical index of the i-th oldest logical sample
    size_t at(size_t i) const { return (m_head + m_cap - m_count + i) % m_cap; }
};
