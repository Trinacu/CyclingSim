// decision.h — perception & decision layer (workstream C).
//
// C0 skeleton: DecisionSystem owns the perception components and their
// per-step feed.  For now that is only the RaceClock; C2 adds decide() — the
// 1 Hz decision cadence, per-rider policies and team directors — on top.
// Owned by Simulation (not PhysicsEngine): the engine stays pure mechanics,
// the decision layer reads it through its public API.  Everything runs on
// the physics thread (whichever thread calls Simulation::step_fixed) —
// decision *cadence* is lower, the thread is the same.

#ifndef DECISION_H
#define DECISION_H

#include "course.h"
#include "race_clock.h"

class PhysicsEngine;

class DecisionSystem {
public:
  explicit DecisionSystem(const Course* course, double grid_spacing = 100.0);

  // Perception feed — call every physics step, after the engine stepped, with
  // the post-step sim time.  Per rider this is one gridline-index comparison
  // unless a gridline/checkpoint was actually crossed.
  void observe(const PhysicsEngine& engine, double t);

  // Drop all perception state (Simulation::reset).
  void reset();

  const RaceClock& race_clock() const { return clock_; }

private:
  RaceClock clock_;
};

#endif
