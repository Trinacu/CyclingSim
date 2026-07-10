// race_clock.h — C0 race-style time gaps & checkpoint times.
//
// Trace-based measurement, the way real races do it: the time gap from a
// rider to the one ahead is *now minus when the rider ahead crossed the point
// where I am now* — robust on gradients, where the instantaneous
// gap_metres / speed estimate balloons with every slope change.
//
// Per rider, a uniform position grid of crossing times (grid_spacing metres
// apart): each entry is interpolated *within the physics step that crossed
// it*, so entries are accurate to milliseconds; memory is bounded by course
// length and the whole-course history is kept.  Queries interpolate between
// the two nearest known points — adjacent gridlines, or the trace's live
// endpoints (first/latest sample) near its edges — in O(1).
//
// Accepted roughness: interpolation assumes constant speed within a cell, so
// a query error concentrates where speed changes sharply inside one cell
// (hitting the base of a climb); at the default 100 m spacing that is ~±1 s —
// race-radio precision.  Named checkpoints (course data — timechecks, KOM
// lines, the finish) never go through the grid: they are captured exactly,
// per rider, at the step that crosses them (finish ordering can be decided
// by hundredths).
//
// Pure and engine-free, same isolation philosophy as drafting/follow/
// rotation: the DecisionSystem feeds record() every physics step (per rider
// it is one index comparison unless something was crossed).  Positions are
// assumed monotone; a sample with no forward progress is ignored, so every
// stored time is a *first* crossing.

#ifndef RACE_CLOCK_H
#define RACE_CLOCK_H

#include "course.h" // Checkpoint
#include "mytypes.h"
#include <optional>
#include <unordered_map>
#include <vector>

class RaceClock {
public:
  RaceClock(double course_length, std::vector<Checkpoint> checkpoints,
            double grid_spacing = 100.0);

  // Feed one rider's position sample at sim time t (monotone in both).
  // Crossings since the previous sample are interpolated linearly between
  // the two — at physics rate that is within-one-step exact.
  void record(RiderId id, double pos, double t);

  // Drop every trace (Simulation::reset).
  void reset();

  // When `id` first crossed `pos`.  nullopt if the rider has not reached
  // pos yet, was never seen, or spawned past it.
  std::optional<double> crossing_time(RiderId id, double pos) const;

  // now - crossing_time(ahead, behind_pos): the chase reaches the point
  // where `ahead` was gap seconds ago.  nullopt when crossing_time is.
  std::optional<double> time_gap(RiderId ahead, double behind_pos,
                                 double now) const;

  const std::vector<Checkpoint>& checkpoints() const { return checkpoints_; }

  // Exact captured crossing time of checkpoint k for this rider; nullopt
  // until crossed.
  std::optional<double> checkpoint_time(RiderId id, size_t k) const;

private:
  struct Trace {
    double anchor_pos = 0.0, anchor_t = 0.0; // first sample
    double latest_pos = 0.0, latest_t = 0.0; // most recent sample
    std::vector<double> grid_t;              // NaN = gridline not crossed
    std::vector<double> cp_t;                // NaN = checkpoint not crossed
    size_t next_cp = 0; // checkpoints are sorted: next one ahead
  };

  double spacing_;
  double course_len_;
  std::vector<Checkpoint> checkpoints_; // sorted by pos
  std::unordered_map<RiderId, Trace> traces_;
};

#endif
