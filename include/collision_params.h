// collision_params.h
#ifndef COLLISION_PARAMS_H
#define COLLISION_PARAMS_H

// CollisionParams was previously defined inline in sim.h.  It has been moved
// to its own header so that lateral_solver.h can include it without creating
// a circular dependency (sim.h → lateral_solver.h → sim.h).
//
// Dependency direction after this change:
//   collision_params.h   (no sim deps)
//   lateral_solver.h     → collision_params.h
//   sim.h                → collision_params.h, lateral_solver.h   (step 4.1)

struct CollisionParams {
  // --- geometry ---
  double rider_radius = 0.5; // m — used as a uniform radius for all riders

  // --- longitudinal proximity window ---
  double x_lookahead = 2.2; // m, how far ahead counts as "in the way"
  double x_contact = 1.2;   // m, threshold for 2D overlap / projection

  // --- blockade force model ---
  double v_min = 1.0;   // m/s — safe lower bound for divisions involving speed
  double F_max = 300.0; // N   — clamp on blocking resistance force
  double k_t = 1.0;     // scaling: tightness → lateral resistance force
  double tight_gamma =
      2.0; // exponent; higher = sharper resistance ramp as gap closes

  // --- shove impulse model ---
  // double shove_kJ = 0.002; // (W·s) → lateral impulse proxy conversion factor
  double shove_kJ = 0.002; // (W·s) → lateral impulse proxy conversion factor
  double J_max = 30.0;     // N·s  — clamp on shove impulse per step

  // Overlap-proportional baseline separation rate.
  // Guarantees divergence even at zero surplus power and equal parameters.
  // full overlap -> k_contact m/s separation.
  double k_contact = 1.0; // m/s — overlap_frac × k_contact = floor displacement

  // --- lateral integration ---
  double lat_damping = 8.0; // 1/s  — exponential velocity damping coefficient

  // Per-contact total separation rate cap.
  double max_lat_correction = 10.0; // m/s
  double lat_spring_k = 4.0;        // 1/s² — spring constant toward lat_target
};

#endif
