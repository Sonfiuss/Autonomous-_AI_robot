#include "Sync.hpp"
#include <algorithm>
#include <cmath>
#include <limits>

namespace sync {

static size_t lowerBracket(const std::vector<TimedValue>& s, double t) {
    // returns index i such that s[i].t <= t < s[i+1].t (clamped)
    if (t <= s.front().t_ms) return 0;
    if (t >= s.back().t_ms)  return s.size() - 1;
    size_t lo = 0, hi = s.size() - 1;
    while (hi - lo > 1) {
        size_t mid = (lo + hi) / 2;
        if (s[mid].t_ms <= t) lo = mid; else hi = mid;
    }
    return lo;
}

float interpAt(const std::vector<TimedValue>& s, double t) {
    if (s.empty()) return 0.f;
    if (s.size() == 1) return s[0].value;
    if (t <= s.front().t_ms) return s.front().value;
    if (t >= s.back().t_ms)  return s.back().value;
    size_t i = lowerBracket(s, t);
    const auto& a = s[i];
    const auto& b = s[i + 1];
    double span = b.t_ms - a.t_ms;
    double w = (span > 0) ? (t - a.t_ms) / span : 0.0;
    return (float)(a.value + w * (b.value - a.value));
}

float nearestAt(const std::vector<TimedValue>& s, double t) {
    if (s.empty()) return 0.f;
    size_t i = lowerBracket(s, t);
    size_t j = std::min(i + 1, s.size() - 1);
    return (std::fabs(t - s[i].t_ms) <= std::fabs(s[j].t_ms - t)) ? s[i].value
                                                                  : s[j].value;
}

double gapTo(const std::vector<TimedValue>& s, double t) {
    if (s.empty()) return std::numeric_limits<double>::infinity();
    size_t i = lowerBracket(s, t);
    size_t j = std::min(i + 1, s.size() - 1);
    return std::min(std::fabs(t - s[i].t_ms), std::fabs(s[j].t_ms - t));
}

double estimateTOffset(const std::vector<TimedValue>& frames,
                       const std::vector<TimedValue>& angles,
                       double search_ms, double step_ms) {
    double best_off = 0.0, best_err = std::numeric_limits<double>::infinity();
    for (double off = -search_ms; off <= search_ms + 1e-9; off += step_ms) {
        double err = 0.0;
        for (const auto& f : frames) {
            double pred = interpAt(angles, f.t_ms + off);
            double d = pred - f.value;
            err += d * d;
        }
        if (err < best_err) { best_err = err; best_off = off; }
    }
    return best_off;
}

size_t countDiscarded(const std::vector<TimedValue>& frames,
                      const std::vector<TimedValue>& angles,
                      double threshold_ms) {
    size_t n = 0;
    for (const auto& f : frames)
        if (gapTo(angles, f.t_ms) > threshold_ms) ++n;
    return n;
}

} // namespace sync
