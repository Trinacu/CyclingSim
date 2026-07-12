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
  // The reference rider: the leader ahead (Behind) or the ward behind
  // (Ahead/protect, C4).
  RiderId leader = -1;
  FollowRelation relation = FollowRelation::Behind;
  // Integral term in effort units, hard-clamped to [0, max_effort].
  // Bootstrapped from the rider's current target_effort on assignment so a
  // mode switch never steps effort discontinuously.
  double integrator = 0.0;

  // Merge state (rotation, D3).  side != 0 means the rider is off the line
  // merging back: the swing offset applies to its lat_target (full while the
  // wheel gap to the followed rider is <= 0, fading to 0 at the setpoint)
  // and the drift speed-hold competes with the gap controller via max().
  // Set on swing-off; cleared by the engine the first time the gap reaches
  // the setpoint (the offset has already faded to 0 there — no lateral step).
  double side = 0.0;             // +1 / -1 swing side; 0 = not merging
  double drift_integrator = 0.0; // speed-hold I, clamped [0, max_effort]

  // Move-up transit (sitter promotion C-pre-b; the C4 join reuses this).
  // approach_side != 0 means the rider is closing on the tail from behind:
  // it rides offset to the advance side (full beyond approach_fade_len above
  // the setpoint, fading to 0 at the setpoint) with commanded effort clamped
  // to effort_cap.  Refreshed each tick by the rotation directive; cleared by
  // the engine the first time the gap closes to the setpoint (the offset has
  // already faded to 0 there — no lateral step).
  double approach_side = 0.0; // +1 / -1 advance side; 0 = not in transit
  double effort_cap = -1.0;   // effort units; < 0 = no cap
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

// Protect (C4): the same regulator with the reference swapped — the ward is
// *behind*.  The caller builds the input from the protector's side of the
// pair: gap = own_pos - ward_pos - own bike_len (the identical quantity the
// ward's follow controller sees, so both ends agree on the setpoint),
// rel_speed = ward speed - own speed.  Error is sign-mirrored (gap too big →
// ease and let the ward back on; ward closing → push), position gain is the
// softer protect_kp, and the rel_speed term speed-matches the ward.  Same
// clamping discipline as follow_effort; no braking — a ward that eases is
// re-collected by drag alone, same as an overrun.
double protect_effort(const FollowInput& in, double dt, double& integrator,
                      const FollowParams& p);

// Drift speed-hold PI (rotation, D3): v_err = (v_leader - drift_delta) -
// v_own.  Same clamping discipline as the gap controller.  The caller
// max()es this with follow_effort for a merging rider — far from the tail
// the gap controller wants 0 and this holds the drift speed; approaching
// the wheel the gap controller rises through it and takes over smoothly.
double drift_effort(double v_err, double dt, double& integrator,
                    double max_effort, const FollowParams& p);

#endif
