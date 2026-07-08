#ifndef FOLLOW_PARAMS_H
#define FOLLOW_PARAMS_H

// Tunables for the D2 gap-holding follow controller (follow.h).
// Dependency-free so it can be included anywhere without cycles — sibling of
// drafting_params.h / grouping_params.h.
typedef struct FollowParams {
  // Setpoint: target wheel-to-wheel gap = d0 + h * own_speed.
  // h is the string-stability escape hatch: a tiny headway (~0.02-0.05 s)
  // damps oscillation travelling down a chain if the accordion test ever
  // shows it growing; at race speed it costs well under a metre of gap,
  // which the drafting falloff barely notices.
  double d0 = 0.25; // m
  double h = 0.0;   // s

  // PI(D) gains, effort units per metre / metre-second / (m/s).
  // Defaults placed via linearisation around cruise for a ~73 kg, 250 W
  // rider (poles ~ -0.8, -0.8, -0.2 rad/s) and validated empirically by the
  // accordion test in tests/test_follow.cpp.
  double kp = 2.8;
  double ki = 0.35;
  double kd = 5.0;
} FollowParams;

#endif
