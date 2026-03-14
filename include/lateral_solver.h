// lateral_solver.h
#ifndef LATERAL_SOLVER_H
#define LATERAL_SOLVER_H

// lateral_solver.h defines the data contracts and solver interface for the
// lateral impulse model.
//
// Dependency policy:
//   lateral_solver.h → collision_params.h, mytypes.h
//   No dependency on sim.h, rider.h, SDL, or Eigen.  This makes LateralSolver
//   independently testable without any simulation infrastructure.
//
// Three types are defined here:
//
//   LateralRiderState  — input per rider: a flat, copyable snapshot built by
//                        PhysicsEngine before calling solve().
//
//   LateralUpdate      — output per rider: what the solver wants written back
//                        into each Rider object.
//
//   LateralSolver      — stateless algorithm class.  Holds only CollisionParams
//                        (injected once at construction).  solve() is const.

#include "collision_params.h"
#include "mytypes.h" // RiderId
#include <optional>
#include <vector>

// ---------------------------------------------------------------------------
// LateralRiderState
//
// A flat, POD-like snapshot of everything the solver needs to know about one
// rider.  Built by PhysicsEngine::step_lateral_solve() from live Rider state
// immediately before calling LateralSolver::solve().
//
// Design notes:
//   - road_width is per-rider (queried from the course at lon_pos) so the
//     solver handles variable-width segments without needing a course
//     reference.
//   - surplus_power approximates the power budget available for a shove:
//     typically (rider power) minus (power consumed by longitudinal resistance
//     at current speed).  The engine populates this; the ShoveModel consumes
//     it.
//   - rider_radius is taken from CollisionParams (uniform for all riders for
//     now), so it is NOT duplicated here.
// ---------------------------------------------------------------------------
struct LateralRiderState {
  RiderId id;

  // --- longitudinal ---
  double lon_pos; // metres along course (state.pos)
  double speed;   // longitudinal speed m/s

  // --- lateral kinematics ---
  double lat_pos;                   // metres from road centre
  double lat_vel;                   // m/s lateral
  std::optional<double> lat_target; // desired lat_pos from behavior phase

  // --- energy ---
  double w_prime_frac;  // W'bal / W'_total in [0, 1]; 0 = depleted
  double surplus_power; // W — power above longitudinal resistance at speed

  // --- physical ---
  double mass; // kg — rider + bike combined
  double rider_radius;
  double bike_length;
  double road_width; // m — full driveable width at lon_pos
};

// ---------------------------------------------------------------------------
// LateralUpdate
//
// Output of LateralSolver::solve(), one entry per rider in the input vector.
// PhysicsEngine::step_lateral_apply() calls Rider::apply_lateral_update()
// with these values.
//
// speed_penalty is a multiplier in [0, 1] applied to state.speed post-step.
// A value of 1.0 means no penalty (normal case when no contact is occurring).
// ---------------------------------------------------------------------------
struct LateralUpdate {
  RiderId id;

  double new_lat_pos; // metres from road centre, clamped to road bounds
  double new_lat_vel; // m/s — derived from actual displacement / dt

  // Longitudinal speed multiplier accounting for energy spent on lateral
  // contact.  Applied post-longitudinal-solve so Newton convergence is not
  // disturbed.  Must be in [0, 1].
  double speed_penalty;
};

// ---------------------------------------------------------------------------
// LateralSolver
//
// Stateless algorithm class — solve() is const and has no side effects beyond
// its return value.  The engine holds one instance for the lifetime of the
// simulation.
//
// CollisionParams is injected at construction and stored by value.  This means
// the engine can re-construct or reconfigure the solver at any point without
// touching in-flight state (there is none).
//
// Implementation plan:
//   Phase 3.1  — free movement (spring-damper toward lat_target, no contacts)
//   Phase 3.2  — proximity detection, build contact pair list
//   Phase 3.3  — blockade detection
//   Phase 3.4  — ShoveModel (nested private class or free function)
//   Phase 3.5  — full resolution: integrate contacts into free-movement output
//
// Until Phase 3.1 is implemented, solve() returns identity updates:
// each rider keeps its current lat_pos and lat_vel, speed_penalty = 1.0.
// The project stays fully functional in this state.
// ---------------------------------------------------------------------------
class LateralSolver {
public:
  explicit LateralSolver(const CollisionParams& params);

  // Core entry point.  Called once per physics step by PhysicsEngine.
  // Input:  flat vector of per-rider state snapshots (order irrelevant)
  // Output: flat vector of per-rider update instructions, same length and
  //         same order as the input vector.
  std::vector<LateralUpdate> solve(const std::vector<LateralRiderState>& riders,
                                   double dt) const;

private:
  CollisionParams params_;
  // --- 3.1: single-rider free integration ---
  LateralUpdate free_movement(const LateralRiderState& r, double dt) const;

  // --- 3.2: proximity pair detection ---
  struct ContactPair {
    int a_idx; // index into the riders vector
    int b_idx;
    double lon_sep; // |A.lon_pos - B.lon_pos|, always >= 0
    double
        lat_sep; // B.lat_pos - A.lat_pos  (signed: +ve means B is to the right)
  };
  std::vector<ContactPair>
  find_proximity_pairs(const std::vector<LateralRiderState>& riders) const;

  // --- 3.3: blockade detection ---
  // Returns true if every lateral gap ahead of riders[rider_idx] within
  // x_lookahead is narrower than 2*rider_radius (no passable lane exists).
  bool is_blocked(int rider_idx,
                  const std::vector<LateralRiderState>& riders) const;

  // --- 3.4: shove model ---
  // Applied to one contact pair.  Both riders push each other away
  // symmetrically; the stronger/fresher rider displaces the other more.
  struct ShoveOutcome {
    double a_lat_delta;     // displacement for rider A (toward more space)
    double b_lat_delta;     // displacement for rider B
    double a_speed_penalty; // [0,1]
    double b_speed_penalty; // [0,1]
  };
  ShoveOutcome compute_shove(const LateralRiderState& a,
                             const LateralRiderState& b,
                             const ContactPair& pair, double dt) const;

  // --- 3.4 helper: tiebreak direction when lat_sep ~ 0 ---
  // Returns +1 or -1: the direction rider A should move.
  // Picks the side with more available space; falls back to id parity.
  int tiebreak_direction(const LateralRiderState& a,
                         const LateralRiderState& b) const;
};

#endif
