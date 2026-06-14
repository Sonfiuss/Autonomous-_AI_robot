#pragma once

#include <vector>
#include <cstddef>

/**
 * Sync utilities (plan §6) — pairing frames with angles and estimating the
 * fixed temporal offset between the camera clock and the angle clock.
 *
 * All times in milliseconds. Pure / hardware-free, exercised by test_sync.
 */
namespace sync {

struct TimedValue { double t_ms; float value; };

// Linear interpolation of a time-ordered series at time t (clamped at ends).
float interpAt(const std::vector<TimedValue>& series, double t_ms);

// Nearest-sample value (no interpolation).
float nearestAt(const std::vector<TimedValue>& series, double t_ms);

// Time gap to the nearest sample at time t.
double gapTo(const std::vector<TimedValue>& series, double t_ms);

/**
 * Estimate t_offset: the value δ (ms) such that interpAt(angles, frame.t + δ)
 * best matches the per-frame observed value. Sweeps candidate offsets in
 * [-search, +search] at `step` and returns the residual-minimizing offset.
 * (Plan §9 "sharpen a moving edge" reduces to this once each frame yields a
 * scalar observation of the swept quantity.)
 */
double estimateTOffset(const std::vector<TimedValue>& frames,
                       const std::vector<TimedValue>& angles,
                       double search_ms = 50.0, double step_ms = 0.5);

// Count frames whose nearest angle gap exceeds the threshold (to be discarded).
size_t countDiscarded(const std::vector<TimedValue>& frames,
                      const std::vector<TimedValue>& angles,
                      double threshold_ms);

} // namespace sync
