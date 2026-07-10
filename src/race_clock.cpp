#include "race_clock.h"
#include <cmath>
#include <limits>

namespace {
constexpr double kUnset = std::numeric_limits<double>::quiet_NaN();

double lerp_t(double p0, double t0, double p1, double t1, double p) {
  return t0 + (t1 - t0) * (p - p0) / (p1 - p0);
}
} // namespace

RaceClock::RaceClock(double course_length, std::vector<Checkpoint> checkpoints,
                     double grid_spacing)
    : spacing_(grid_spacing), course_len_(course_length),
      checkpoints_(std::move(checkpoints)) {}

void RaceClock::reset() { traces_.clear(); }

void RaceClock::record(RiderId id, double pos, double t) {
  auto [it, inserted] = traces_.try_emplace(id);
  Trace& tr = it->second;

  if (inserted) {
    tr.anchor_pos = tr.latest_pos = pos;
    tr.anchor_t = tr.latest_t = t;
    tr.grid_t.assign(static_cast<size_t>(course_len_ / spacing_) + 2, kUnset);
    tr.cp_t.assign(checkpoints_.size(), kUnset);
    // Checkpoints already behind the spawn position stay uncrossed.
    while (tr.next_cp < checkpoints_.size() &&
           checkpoints_[tr.next_cp].pos <= pos)
      ++tr.next_cp;
    return;
  }

  // No forward progress: ignore, so every stored time is a first crossing.
  if (pos <= tr.latest_pos)
    return;

  const double p0 = tr.latest_pos;
  const double t0 = tr.latest_t;

  // Gridlines strictly above the previous sample, up to (and including) pos.
  for (size_t i = static_cast<size_t>(std::floor(p0 / spacing_)) + 1;
       i < tr.grid_t.size() && static_cast<double>(i) * spacing_ <= pos; ++i)
    tr.grid_t[i] = lerp_t(p0, t0, pos, t, static_cast<double>(i) * spacing_);

  // Checkpoints crossed this step: exact capture, once each.
  while (tr.next_cp < checkpoints_.size() &&
         checkpoints_[tr.next_cp].pos <= pos) {
    const double x = checkpoints_[tr.next_cp].pos;
    if (x > p0)
      tr.cp_t[tr.next_cp] = lerp_t(p0, t0, pos, t, x);
    ++tr.next_cp;
  }

  tr.latest_pos = pos;
  tr.latest_t = t;
}

std::optional<double> RaceClock::crossing_time(RiderId id, double pos) const {
  auto it = traces_.find(id);
  if (it == traces_.end())
    return std::nullopt;
  const Trace& tr = it->second;

  if (pos < tr.anchor_pos || pos > tr.latest_pos)
    return std::nullopt;
  if (pos == tr.latest_pos)
    return tr.latest_t; // also covers anchor == latest

  // Every gridline in (anchor, latest] is recorded, so the nearest known
  // points bracketing pos are its cell's gridlines — or the trace endpoints
  // where the cell sticks out past them.
  const size_t lo_i = static_cast<size_t>(std::floor(pos / spacing_));
  double lo_pos = tr.anchor_pos, lo_t = tr.anchor_t;
  if (lo_i < tr.grid_t.size() && !std::isnan(tr.grid_t[lo_i])) {
    lo_pos = static_cast<double>(lo_i) * spacing_;
    lo_t = tr.grid_t[lo_i];
  }
  const size_t hi_i = lo_i + 1;
  double hi_pos = tr.latest_pos, hi_t = tr.latest_t;
  if (hi_i < tr.grid_t.size() && !std::isnan(tr.grid_t[hi_i])) {
    hi_pos = static_cast<double>(hi_i) * spacing_;
    hi_t = tr.grid_t[hi_i];
  }

  if (hi_pos <= lo_pos)
    return lo_t; // degenerate cell (shouldn't happen with monotone samples)
  return lerp_t(lo_pos, lo_t, hi_pos, hi_t, pos);
}

std::optional<double> RaceClock::time_gap(RiderId ahead, double behind_pos,
                                          double now) const {
  const auto ct = crossing_time(ahead, behind_pos);
  if (!ct)
    return std::nullopt;
  return now - *ct;
}

std::optional<double> RaceClock::checkpoint_time(RiderId id, size_t k) const {
  auto it = traces_.find(id);
  if (it == traces_.end() || k >= it->second.cp_t.size() ||
      std::isnan(it->second.cp_t[k]))
    return std::nullopt;
  return it->second.cp_t[k];
}
