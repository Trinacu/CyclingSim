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

  // Drift-back (rotation, D3): while merging, the rider holds
  // v_leader - drift_delta via a speed-hold PI, max-combined with the gap
  // controller, and rides offset from the wake axis by swing_offset_radii *
  // own radius (fading to 0 as the wheel gap opens to the setpoint).
  double drift_delta = 0.4; // m/s backward relative to the line
  double drift_kp = 2.0;    // effort per (m/s) of speed error
  double drift_ki = 0.4;    // effort per (m/s · s)
  double swing_offset_radii = 3.0;

  // Move-up transit (C-pre-b): the advance-side offset (swing_offset_radii,
  // shared with the drift merge) fades to 0 over the last approach_fade_len
  // metres of wheel gap above the setpoint, so the cut-in ends exactly on
  // the wake axis.
  double approach_fade_len = 2.0; // m

  // Protect (C4): the swapped-reference regulator (ward behind).  Same d0/h
  // setpoint as the follow side — the natural mutual pairing (ward follows
  // protector) must agree on the gap from both ends.  The position gain is
  // deliberately softer than kp: the ward's follow controller does the tight
  // gap-keeping, the protector mostly speed-matches (kd term) and corrects
  // position slowly — hard protect_kp against a following ward is how the
  // pair oscillates.  ⚖️ Validated by the mutual-pair test
  // (tests/test_follow.cpp); retune there.
  double protect_kp = 1.4;
  double protect_ki = 0.35;
  double protect_kd = 5.0;
} FollowParams;

#endif
