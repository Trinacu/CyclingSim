#include "lateral_solver.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <numeric>

LateralSolver::LateralSolver(const CollisionParams& params) : params_(params) {}

// ============================================================================
// 3.1  free_movement
//
// Integrates one rider's spring-damper toward an optional lat_target.
//
// Damping term  — EXACT exponential decay:
//   vel *= exp(-lat_damping * dt)
//
//   Euler would use vel *= (1 - lat_damping * dt), which diverges at large dt
//   (unstable when dt > 2/lat_damping) and introduces O(dt) error at smaller
//   dt.  The exponential is exact for the pure-damping ODE and costs one
//   std::exp() per rider per step — negligible.
//
// Spring term   — Euler integration:
//   vel += spring_k * w_prime_frac * (target - pos) * dt
//
//   The system is heavily overdamped (lat_damping^2 >> 4*spring_k), so the
//   spring contribution is small relative to damping.  Euler error on the
//   spring term is second-order in dt and is acceptable.
//
// Result: free_movement behaviour is dt-independent for any practically used
//         update frequency.
// ============================================================================
LateralUpdate LateralSolver::free_movement(const LateralRiderState& r,
                                           double dt) const {
  double spring_a = 0.0;
  if (r.lat_target.has_value()) {
    const double error = r.lat_target.value() - r.lat_pos;
    const double strength = params_.lat_spring_k * r.w_prime_frac;
    spring_a += strength * error;
  }

  // Exact exponential decay of existing velocity (dt-independent damping),
  // then add spring impulse via Euler.
  const double decay = std::exp(-params_.lat_damping * dt);
  const double new_vel = r.lat_vel * decay + spring_a * dt;
  const double new_pos = std::clamp(r.lat_pos + new_vel * dt,
                                    -r.road_width / 2.0, +r.road_width / 2.0);

  return LateralUpdate{
      .id = r.id,
      .new_lat_pos = new_pos,
      .new_lat_vel = new_vel, // overwritten in step 3.5 if contacts present
      .speed_penalty = 1.0,
  };
}

// ============================================================================
// 3.2  find_proximity_pairs
//
// Returns all pairs (a, b) where:
//   |lon_sep| < x_lookahead   — longitudinally close enough to interact
//   |lat_sep| < 2*rider_radius — laterally overlapping (in contact)
//
// The list is built from CURRENT (pre-step) positions, not free-movement
// candidates, so we do not miss contacts that close during integration.
//
// Algorithm:
//   Sort a copy of rider indices by lon_pos (O(N log N)).
//   Walk a sliding window: for each rider A, advance a pointer forward
//   collecting riders B within x_lookahead.  This gives O(N·k) pair
//   candidates where k is average window occupancy — negligible at N ≤ 20.
// ============================================================================
std::vector<LateralSolver::ContactPair> LateralSolver::find_proximity_pairs(
    const std::vector<LateralRiderState>& riders) const {

  const int N = static_cast<int>(riders.size());
  if (N < 2)
    return {};

  // Sort indices by longitudinal position
  std::vector<int> idx(N);
  std::iota(idx.begin(), idx.end(), 0);
  std::sort(idx.begin(), idx.end(), [&](int i, int j) {
    return riders[i].lon_pos < riders[j].lon_pos;
  });

  std::vector<ContactPair> pairs;

  for (int si = 0; si < N; ++si) {
    const int ai = idx[si];
    const double lon_a = riders[ai].lon_pos;

    for (int sj = si + 1; sj < N; ++sj) {
      const int bi = idx[sj];
      const double lon_b = riders[bi].lon_pos;
      const double lon_sep = lon_b - lon_a; // always >= 0 due to sort

      // this is not 100% correct, because there might be another rider
      // b at a higher pos but with larger bike_length - for now good enuf
      if (lon_sep >= riders[bi].bike_length)
        break; // window exhausted for this A

      const double lat_sep = riders[bi].lat_pos - riders[ai].lat_pos;

      double threshold_lat = riders[ai].rider_radius + riders[bi].rider_radius;
      if (std::fabs(lat_sep) < threshold_lat) {
        pairs.push_back(ContactPair{
            .a_idx = ai,
            .b_idx = bi,
            .lon_sep = lon_sep,
            .lat_sep = lat_sep, // signed: +ve means B is to the right of A
        });
      }
    }
  }

  return pairs;
}

// ============================================================================
// 3.3  is_blocked
//
// Returns true if rider at riders[rider_idx] has no passable gap ahead.
//
// A gap is passable if its width >= 2*rider_radius.
// Gaps checked:
//   left road edge  → leftmost ahead-rider
//   between adjacent ahead-riders (sorted by lat_pos)
//   rightmost ahead-rider → right road edge
//
// Used by compute_shove to modulate tightness: when fully blocked, more of
// the contact impulse is absorbed rather than penetrating the blockade.
// ============================================================================
bool LateralSolver::is_blocked(
    int rider_idx, const std::vector<LateralRiderState>& riders) const {
  const LateralRiderState& own = riders[rider_idx];
  const double min_gap = 2.0 * own.rider_radius;
  const double half_road = own.road_width / 2.0;

  // Collect riders ahead within x_lookahead
  std::vector<double> ahead_lat;
  for (int i = 0; i < static_cast<int>(riders.size()); ++i) {
    if (i == rider_idx)
      continue;
    const double lon_offset = riders[i].lon_pos - own.lon_pos;
    if (lon_offset > 0.0 && lon_offset < riders[i].bike_length)
      ahead_lat.push_back(riders[i].lat_pos);
  }

  if (ahead_lat.empty())
    return false; // clear road

  std::sort(ahead_lat.begin(), ahead_lat.end());

  // Left edge to first rider
  if ((ahead_lat.front() - own.rider_radius) - (-half_road) >= min_gap)
    return false;

  // Between adjacent riders
  for (size_t i = 0; i + 1 < ahead_lat.size(); ++i) {
    const double gap = (ahead_lat[i + 1] - own.rider_radius) -
                       (ahead_lat[i] + own.rider_radius);
    if (gap >= min_gap)
      return false;
  }

  // Last rider to right edge
  if (half_road - (ahead_lat.back() + own.rider_radius) >= min_gap)
    return false;

  return true; // every gap is too narrow
}

// ============================================================================
// 3.4  tiebreak_direction
//
// Used when two riders are at nearly identical lateral positions (|lat_sep|
// below a small epsilon) and we cannot determine which way to push them from
// geometry alone.
//
// Strategy:
//   Prefer the direction toward the side with more available space.
//   Space is measured as (road_edge - nearest_occupied_position) on each side.
//   If space is equal (within epsilon), fall back to a deterministic parity
//   on the lower rider id — same result every step, no oscillation.
//
// Returns +1 or -1: the direction rider A should be pushed.
// ============================================================================
int LateralSolver::tiebreak_direction(const LateralRiderState& a,
                                      const LateralRiderState& b) const {
  const double half = std::min(a.road_width, b.road_width) / 2.0;
  const double mid = (a.lat_pos + b.lat_pos) / 2.0;

  const double space_left = mid - (-half); // distance to left edge
  const double space_right = half - mid;   // distance to right edge

  constexpr double kEpsilon = 1e-4;

  if (space_left - space_right > kEpsilon)
    return -1; // more room left: A goes left
  if (space_right - space_left > kEpsilon)
    return +1; // more room right: A goes right

  // Equal space: deterministic tiebreak on id
  return (a.id < b.id) ? -1 : +1;
}

// ============================================================================
// 3.4  compute_shove
//
// Computes bilateral lateral displacement for one contact pair.
//
// == Why dt is passed here ==
//
// All physical quantities that represent a RATE must be multiplied by dt to
// produce a per-step displacement.  Centralising this in compute_shove (rather
// than pre-scaling surplus_power in solve()) keeps the intent explicit at each
// use site and ensures nothing is accidentally scaled twice or not at all.
//
// surplus_power is received in Watts (NOT pre-multiplied by dt).
// =============================================================================
LateralSolver::ShoveOutcome
LateralSolver::compute_shove(const LateralRiderState& a,
                             const LateralRiderState& b,
                             const ContactPair& pair, double dt) const {
  // --- direction ---
  constexpr double kDirEpsilon = 1e-3; // m

  // direction_a: +1 means A moves right, -1 means A moves left
  int direction_a;
  if (pair.lat_sep > +kDirEpsilon)
    direction_a = -1; // A is left of B → push A further left
  else if (pair.lat_sep < -kDirEpsilon)
    direction_a = +1; // A is right of B → push A further right
  else
    direction_a = tiebreak_direction(a, b);

  const int direction_b = -direction_a; // B goes the opposite way
                                        //
  // --- resistance fractions (no dt dependence) ---
  const double resist_a = a.mass * (0.5 + 0.5 * a.surplus_power);
  const double resist_b = b.mass * (0.5 + 0.5 * b.surplus_power);
  const double resist_total = resist_a + resist_b;

  const double frac_a = resist_b / resist_total;
  const double frac_b = resist_a / resist_total;

  // --- contact floor: rate * dt ---
  const double contact_dist = a.rider_radius + b.rider_radius;
  const double overlap_frac =
      std::clamp((contact_dist - fabs(pair.lat_sep)) / contact_dist, 0.0, 1.0);
  const double contact_floor = overlap_frac * params_.k_contact * dt;

  // --- active budget: clamp on RATE, then * dt ---
  // surplus_power is in Watts (not pre-scaled by dt).
  // J_max caps the rate; * dt converts to per-step displacement.
  const double active_a =
      std::clamp(a.surplus_power * params_.shove_kJ, 0.0, params_.J_max) *
      a.w_prime_frac * dt;
  const double active_b =
      std::clamp(b.surplus_power * params_.shove_kJ, 0.0, params_.J_max) *
      b.w_prime_frac * dt;

  // --- total separation: rate cap * dt ---
  const double total_sep = std::clamp(contact_floor + active_a + active_b, 0.0,
                                      params_.max_lat_correction * dt);

  // --- bilateral displacements ---
  const double delta_a = static_cast<double>(direction_a) * frac_a * total_sep;
  const double delta_b = static_cast<double>(direction_b) * frac_b * total_sep;

  // --- speed penalty ---
  // Threshold and max both scale with dt so compounded penalty/s is constant.
  constexpr double kPenaltyThresholdRate =
      0.1;                                // m/s — fire if sep rate exceeds this
  constexpr double kMaxPenaltyRate = 5.0; // /s  — max penalty rate
  constexpr double kPenaltyScale =
      0.5; // m^-1 — displacement -> fraction (no dt)

  const double threshold = kPenaltyThresholdRate * dt;
  const double max_pen = kMaxPenaltyRate * dt;

  const double penalty_a =
      (std::fabs(delta_a) > threshold)
          ? std::clamp(std::fabs(delta_a) * kPenaltyScale, 0.0, max_pen)
          : 0.0;
  const double penalty_b =
      (std::fabs(delta_b) > threshold)
          ? std::clamp(std::fabs(delta_b) * kPenaltyScale, 0.0, max_pen)
          : 0.0;

  return ShoveOutcome{
      .a_lat_delta = delta_a,
      .b_lat_delta = delta_b,
      .a_speed_penalty = 1.0 - penalty_a,
      .b_speed_penalty = 1.0 - penalty_b,
  }; //
}

// ============================================================================
// 3.5  solve  — full pipeline
//
// Execution order per step:
//
//   [1] Free movement  — integrate all riders independently toward their
//                        optional lat_target using the spring-damper model.
//                        Produces candidate positions.
//
//   [2] Contact pairs  — detect overlapping pairs from CURRENT (pre-step)
//                        positions.  Using pre-step positions ensures we
//                        never miss a contact that closes during integration.
//
//   [3] Shove          — for each contact pair, compute bilateral displacement
//                        deltas and speed penalties.  Deltas are accumulated
//                        per rider (multiple simultaneous contacts stack).
//                        Speed penalties are multiplied (conservative: worst
//                        case when a rider is in contact with several others).
//
//   [4] Compose        — add accumulated deltas to the free-movement
//   candidates.
//                        Clamp to road bounds.
//
//   [5] Velocity       — recompute lat_vel from TOTAL displacement / dt, not
//                        from the spring-damper term.  This is the critical
//                        velocity pitfall: if we kept the spring-damper vel,
//                        the damping term would fight the contact correction
//                        on the very next step, producing jitter.
//
// Speed penalty note:
//   Penalties are 1.0 (no effect) when no displacement occurs.  This prevents
//   compounding penalty for riders who are merely adjacent without shoving.
// ============================================================================
std::vector<LateralUpdate>
LateralSolver::solve(const std::vector<LateralRiderState>& riders,
                     double dt) const {

  const int N = static_cast<int>(riders.size());
  if (N == 0)
    return {};

  // --- [1] Free movement (spring-damper, optional steering) ---
  std::vector<LateralUpdate> updates;
  updates.reserve(N);
  for (const auto& r : riders)
    updates.push_back(free_movement(r, dt));

  if (N == 1)
    return updates; // no contacts possible

  // --- [2] Contact pair detection on current positions ---
  std::vector<ContactPair> pairs = find_proximity_pairs(riders);

  if (pairs.empty())
    return updates; // clean run — free movement only

  // --- [3] Shove model — accumulate deltas and multiply penalties ---

  // Working accumulators indexed by position in the riders vector
  std::vector<double> delta_acc(N, 0.0);   // accumulated lat_pos deltas
  std::vector<double> penalty_acc(N, 1.0); // accumulated speed multipliers

  for (const auto& pair : pairs) {
    const ShoveOutcome out =
        compute_shove(riders[pair.a_idx], riders[pair.b_idx], pair, dt);
    delta_acc[pair.a_idx] += out.a_lat_delta;
    delta_acc[pair.b_idx] += out.b_lat_delta;
    penalty_acc[pair.a_idx] *= out.a_speed_penalty;
    penalty_acc[pair.b_idx] *= out.b_speed_penalty;
    // SDL_Log("SHOVE %d vs %d deltaA= %.4f deltaB= %.4f",
    // riders[pair.a_idx].id,
    //         riders[pair.b_idx].id, out.a_lat_delta, out.b_lat_delta);
  }

  // --- [4] Compose: free-movement candidate + contact delta, clamped ---
  for (int i = 0; i < N; ++i) {
    const double half_road = riders[i].road_width / 2.0;
    updates[i].new_lat_pos = std::clamp(updates[i].new_lat_pos + delta_acc[i],
                                        -half_road, +half_road);
    updates[i].speed_penalty = penalty_acc[i];
  }

  // --- [5] Recompute lat_vel from total actual displacement ---
  // This overwrites the spring-damper velocity so damping doesn't fight
  // contact corrections on the next step.
  for (int i = 0; i < N; ++i) {
    updates[i].new_lat_vel = (updates[i].new_lat_pos - riders[i].lat_pos) / dt;
  }

  return updates;
}
