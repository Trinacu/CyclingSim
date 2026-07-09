#ifndef ROTATION_PARAMS_H
#define ROTATION_PARAMS_H

// Tunables for the D3 paceline rotation coordinator (rotation.h).
// Dependency-free — sibling of follow_params.h / drafting_params.h.
typedef struct RotationParams {
  double pull_time = 20.0; // s at the front before swinging off

  // Swing side when crosswind is ~0 (with wind the drifter always takes the
  // windward side, sheltering the line while the extra wind slows him).
  double default_side = 1.0;

  // The pull timer only advances while the puller's immediate follower is
  // attached within this wheel gap — an unformed or disrupted line doesn't
  // rotate.
  double engage_gap = 1.0; // m

  // A member whose wheel gap to the rider ahead exceeds detach_gap for
  // longer than detach_time is removed from the roster (physics already
  // dropped them; smarter role handling is C4-era).  detach_gap defaults to
  // the draft cutoff.
  double detach_gap = 8.0;  // m
  double detach_time = 3.0; // s
} RotationParams;

#endif
