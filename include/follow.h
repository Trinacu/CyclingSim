// follow.h — D2 gap-holding follow controller.
//
// A PI(D) controller on wheel-to-wheel gap error whose output IS the rider's
// target_effort (single-writer ownership — see sim.h EffortSource).  Pure and
// engine-free, same isolation philosophy as drafting.h: the engine builds a
// FollowInput per following rider and owns the per-rider FollowState.
//
// Design constraints (PLAN.md D2):
//   - The gap uses D1's definition verbatim: lon_sep - leader bike_len, so
//     the setpoint and the drafting falloff agree on what "0.25 m" means.
//   - The integral term holds the entire cruise effort at converged steady
//     state, so it is never zeroed; windup is prevented purely by hard
//     clamping it to [0, max_effort].  The >= 0 clamp is what makes recovery
//     from an overrun immediate (a negative integral would only delay the
//     response when the leader accelerates again — effort can't go below 0
//     anyway).
//   - max_effort is the rider's *static* ceiling (RiderConfig).  The dynamic,
//     fatigue-dependent effort limit belongs to the energy model, which clamps
//     realized effort downstream; the controller never learns why a request
//     wasn't met.  A dying rider getting dropped is emergent.
//   - No braking: output floors at 0, deceleration is drag-only, wheel
//     overlap (negative gap) is tolerated.

#ifndef FOLLOW_H
#define FOLLOW_H

#include "follow_params.h"
#include "mytypes.h"

// Per-rider controller state, owned by the engine for the lifetime of the
// follow target.
struct FollowState {
  RiderId leader = -1;
  // Integral term in effort units, hard-clamped to [0, max_effort].
  // Bootstrapped from the rider's current target_effort on assignment so a
  // mode switch never steps effort discontinuously.
  double integrator = 0.0;
};

// Per-tick input, built by the engine from one-tick-stale positions (fine at
// 100 Hz, same as drafting).
struct FollowInput {
  double gap;        // wheel-to-wheel to the leader: lon_sep - leader bike_len (m)
  double rel_speed;  // leader speed - own speed = d(gap)/dt (m/s)
  double own_speed;  // m/s; sets the h*v part of the target gap
  double max_effort; // rider's static ceiling (RiderConfig::max_effort)
};

// Advance the integrator by dt and return the commanded effort, clamped to
// [0, in.max_effort].
double follow_effort(const FollowInput& in, double dt, double& integrator,
                     const FollowParams& p);

#endif
