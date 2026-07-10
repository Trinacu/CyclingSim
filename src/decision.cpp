#include "decision.h"
#include "sim.h"

DecisionSystem::DecisionSystem(const Course* course, double grid_spacing)
    : clock_(course->get_total_length(), course->get_checkpoints(),
             grid_spacing) {}

void DecisionSystem::observe(const PhysicsEngine& engine, double t) {
  // Per-rider traces are independent — iteration order is irrelevant here
  // (unlike the C2 decision phase, which must iterate in sorted id order).
  for (const auto& [id, r] : engine.get_riders())
    clock_.record(id, r->get_pos(), t);
}

void DecisionSystem::reset() { clock_.reset(); }
