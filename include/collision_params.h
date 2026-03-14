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

class Bike;
struct RiderConfig;

struct CollisionParams {
  // Overlap-proportional baseline separation rate.
  // Guarantees divergence even at zero surplus power and equal parameters.
  // full overlap -> k_contact m/s separation.
  double k_contact = 0.07;

  // --- shove impulse model ---
  // shove_kJ determines additional push due to power output difference
  double shove_kJ = 1e-8; // (W·s) → lateral impulse proxy conversion factor
  double J_max = 30.0;    // N·s  — clamp on shove impulse per step

  // --- lateral integration ---
  double lat_damping = 8.0; // 1/s  — exponential velocity damping coefficient

  // Per-contact total separation rate cap.
  double max_lat_correction = 10.0; // m/s
  double lat_spring_k = 4.0;        // 1/s² — spring constant toward lat_target

  static CollisionParams from_config(const Bike& bike, const RiderConfig& cfg);
};

#endif
