#define private public
#include "lateral_solver.h"
#undef private

#include "collision_params.h"
#include <cassert>
#include <iostream>

// Wrapper for testing
struct TestableShoveOutcome {
  const LateralSolver::ShoveOutcome& q;

  explicit TestableShoveOutcome(const LateralSolver::ShoveOutcome& quad)
      : q(quad) {}

  bool isApproxEqual(const TestableShoveOutcome other,
                     double epsilon = 1e-10) const {
    return (std::abs(q.a_lat_rate - other.q.a_lat_rate) <= epsilon) &&
           (std::abs(q.b_lat_rate - other.q.b_lat_rate) <= epsilon) &&
           (std::abs(q.a_penalty_rate - other.q.a_penalty_rate) <= epsilon) &&
           (std::abs(q.b_penalty_rate - other.q.b_penalty_rate) <= epsilon);
  }

  void print() const {
    std::cout << "(" << q.a_lat_rate << ", " << q.b_lat_rate << ", "
              << q.a_penalty_rate << ", " << q.b_penalty_rate << ")";
  }
};

// Test utilities
namespace ShoveOutcomeTest {
void printAll(const std::vector<LateralSolver::ShoveOutcome>& results) {
  for (const auto& q : results) {
    TestableShoveOutcome(q).print();
    std::cout << std::endl;
  }
}

bool allEqual(const std::vector<LateralSolver::ShoveOutcome>& results,
              double epsilon = 1e-10) {
  if (results.empty())
    return true;

  TestableShoveOutcome first(results[0]);
  for (size_t i = 1; i < results.size(); ++i) {
    if (!first.isApproxEqual(TestableShoveOutcome(results[i]), epsilon)) {
      ShoveOutcomeTest::printAll(results);
      return false;
    }
  }
  return true;
}

} // namespace ShoveOutcomeTest

CollisionParams default_params() {
  CollisionParams p{};
  p.lat_damping = 0.0;
  p.lat_spring_k = 0.0;
  // Keep the ambient spring out of shove/dt tests: its Euler spring term has a
  // dt-dependent implied rate (like the behavior spring), which would break
  // test_dt_independence for any rider not at lat 0.
  p.ambient_center_k = 0.0;

  p.shove_kJ = 1.0;
  p.J_max = 100.0;

  p.max_lat_correction = 10.0;

  return p;
}

LateralRiderState rider(int id, double lon, double lat, double power = 100,
                        double wprime = 1.0) {
  LateralRiderState r{};
  r.id = id;
  r.lon_pos = lon;
  r.lat_pos = lat;
  r.lat_vel = 0;
  r.mass = 75;
  r.rider_radius = 0.5;
  // Proximity window: solver pairs riders within one bike_length
  // longitudinally.  ~1.53 m for a road bike (wheelbase + 2*wheel_r).
  r.bike_length = 1.5;

  r.surplus_power = power;
  r.w_prime_frac = wprime;

  r.road_width = 10.0;

  return r;
}

void test_contact_detection() {
  auto params = default_params();
  LateralSolver solver(params);

  std::vector<LateralRiderState> riders{rider(1, 0, 0), rider(2, 0.5, 0.2)};

  auto pairs = solver.find_proximity_pairs(riders);

  assert(pairs.size() == 1);
}

void test_stronger_rider_pushes() {
  auto params = default_params();
  LateralSolver solver(params);

  auto strong = rider(1, 0, 0, 200);
  auto weak = rider(2, 0, 0.4, 50);

  LateralSolver::ContactPair pair{
      .a_idx = 0, .b_idx = 1, .lon_sep = 0, .lat_sep = 0.4};

  auto out = solver.compute_shove(strong, weak, pair, false);

  assert(out.b_lat_rate > 0); // weak rider pushed right
}

void test_solver_moves_weaker_rider() {
  auto params = default_params();
  LateralSolver solver(params);

  std::vector<LateralRiderState> riders{rider(1, 0, 0, 200),
                                        rider(2, 0, 0.4, 50)};

  auto updates = solver.solve(riders, 1.0);

  assert(updates[1].new_lat_pos != riders[1].lat_pos);
}

void test_tiebreak_direction() {
  auto params = default_params();
  LateralSolver solver(params);

  auto a = rider(1, 0, 0);
  auto b = rider(2, 0, 0);

  int dir = solver.tiebreak_direction(a, b);

  assert(dir == -1); // id 1 < id 2 → left
}

void test_lat_target_steering() {
  auto params = default_params();
  params.lat_spring_k = 1.0;

  LateralSolver solver(params);

  auto r = rider(1, 0, 0);
  r.lat_target = 2.0;

  auto update = solver.free_movement(r, 1.0);

  assert(update.new_lat_pos > 0);
}

void test_dt_independence() {
  // compute_shove returns pure rates and takes no dt — dt-independence is
  // enforced by its signature.  What remains to verify is that solve()
  // performs exactly one rate -> per-step conversion: the implied rates
  // (displacement / dt, (1 - penalty) / dt) must be identical across
  // integration step sizes.
  auto params = default_params();
  LateralSolver solver(params);

  std::vector<LateralRiderState> riders{rider(1, 0, 0, 200),
                                        rider(2, 0, 0.4, 50)};

  std::vector<double> dts = {0.01, 0.05, 0.1, 0.2};
  std::vector<LateralSolver::ShoveOutcome> res;
  res.reserve(dts.size());

  for (double dt : dts) {
    auto updates = solver.solve(riders, dt);
    res.push_back(LateralSolver::ShoveOutcome{
        .a_lat_rate = (updates[0].new_lat_pos - riders[0].lat_pos) / dt,
        .b_lat_rate = (updates[1].new_lat_pos - riders[1].lat_pos) / dt,
        .a_penalty_rate = (1.0 - updates[0].speed_penalty) / dt,
        .b_penalty_rate = (1.0 - updates[1].speed_penalty) / dt,
    });
  }

  assert(ShoveOutcomeTest::allEqual(res, 1e-8));
  ShoveOutcomeTest::printAll(res);
}

// --- penalty targeting (A1) ---

// A pair with A meaningfully behind B (lon_sep past the squeeze ramp) and a
// strong power imbalance so |rate_a| clears the penalty threshold.
LateralSolver::ContactPair squeeze_pair() {
  return LateralSolver::ContactPair{
      .a_idx = 0, .b_idx = 1, .lon_sep = 1.0, .lat_sep = 0.2};
}

void test_front_rider_never_penalized() {
  LateralSolver solver(default_params());

  auto a = rider(1, 0, 0, 200);
  auto b = rider(2, 1.0, 0.2, 50);

  auto out = solver.compute_shove(a, b, squeeze_pair(), true);

  assert(out.b_penalty_rate == 0.0);
}

void test_blocked_squeezer_penalized() {
  LateralSolver solver(default_params());

  auto a = rider(1, 0, 0, 200);
  auto b = rider(2, 1.0, 0.2, 50);

  auto out = solver.compute_shove(a, b, squeeze_pair(), true);

  assert(out.a_penalty_rate > 0.0);
}

void test_unblocked_overtaker_free() {
  LateralSolver solver(default_params());

  auto a = rider(1, 0, 0, 200);
  auto b = rider(2, 1.0, 0.2, 50);

  auto out = solver.compute_shove(a, b, squeeze_pair(), false);

  assert(out.a_penalty_rate == 0.0);
}

void test_side_by_side_no_penalty() {
  LateralSolver solver(default_params());

  auto a = rider(1, 0, 0, 200);
  auto b = rider(2, 0, 0.2, 50);

  LateralSolver::ContactPair pair{
      .a_idx = 0, .b_idx = 1, .lon_sep = 0, .lat_sep = 0.2};

  auto out = solver.compute_shove(a, b, pair, true);

  assert(out.a_penalty_rate == 0.0);
  assert(out.b_penalty_rate == 0.0);
}

void test_squeeze_ramp_monotonic() {
  LateralSolver solver(default_params());

  auto a = rider(1, 0, 0, 200);
  auto b = rider(2, 0, 0.2, 50); // lon irrelevant; pair carries lon_sep

  // Ramp reaches full at 0.5 * bike_length = 0.75; probe the midpoint.
  auto pair_at = [](double lon_sep) {
    return LateralSolver::ContactPair{
        .a_idx = 0, .b_idx = 1, .lon_sep = lon_sep, .lat_sep = 0.2};
  };

  const double mid =
      solver.compute_shove(a, b, pair_at(0.375), true).a_penalty_rate;
  const double full =
      solver.compute_shove(a, b, pair_at(0.75), true).a_penalty_rate;

  assert(full > 0.0);
  assert(mid > 0.0 && mid < full);
}

// solve()-level: a squeezer behind a wall of riders spanning the road is the
// only one slowed.
void test_wall_penalizes_only_squeezer() {
  LateralSolver solver(default_params());

  // Wall at lon 1.0 spaced 1.5 m apart: every lateral gap (including the road
  // edges, half_road = 5) is narrower than 2 * rider_radius = 1.0 → blocked.
  std::vector<LateralRiderState> riders_v{rider(1, 0, 0, 500, 1.0)};
  const double wall_lats[] = {-4.5, -3.0, -1.5, 0.0, 1.5, 3.0, 4.5};
  int id = 2;
  for (double lat : wall_lats)
    riders_v.push_back(rider(id++, 1.0, lat, 50));

  auto updates = solver.solve(riders_v, 0.01);

  assert(updates[0].speed_penalty < 1.0); // squeezer slowed
  for (size_t i = 1; i < updates.size(); ++i)
    assert(updates[i].speed_penalty == 1.0); // wall untouched
}

void test_shove_symmetry_blend() {
  auto a = rider(1, 0, 0, 500);
  auto b = rider(2, 0, 0.2, 10);
  LateralSolver::ContactPair pair{
      .a_idx = 0, .b_idx = 1, .lon_sep = 0, .lat_sep = 0.2};

  auto params = default_params();

  params.shove_asymmetry = 0.0; // both riders move equally
  auto even = LateralSolver(params).compute_shove(a, b, pair, false);
  assert(std::abs(std::abs(even.a_lat_rate) - std::abs(even.b_lat_rate)) <
         1e-12);

  params.shove_asymmetry = 1.0; // split fully by resistance ratio
  auto full = LateralSolver(params).compute_shove(a, b, pair, false);
  assert(std::abs(full.a_lat_rate) < std::abs(full.b_lat_rate));

  params.shove_asymmetry = 0.4; // weaker rider still gives way more, but the
                                // stronger one moves more than at full ratio
  auto blended = LateralSolver(params).compute_shove(a, b, pair, false);
  assert(std::abs(blended.a_lat_rate) < std::abs(blended.b_lat_rate));
  assert(std::abs(blended.a_lat_rate) > std::abs(full.a_lat_rate));
}

// --- ambient centering (A2) ---

void test_ambient_centering() {
  auto params = default_params();
  params.lat_damping = 8.0; // realistic damping so the approach is monotonic
  params.ambient_center_k = 0.2;

  LateralSolver solver(params);

  auto r = rider(1, 0, 3.0);
  assert(!r.lat_target.has_value());

  double prev = r.lat_pos;
  for (int i = 0; i < 1000; ++i) {
    auto upd = solver.solve({r}, 0.1);
    r.lat_pos = upd[0].new_lat_pos;
    r.lat_vel = upd[0].new_lat_vel;
    assert(r.lat_pos <= prev + 1e-12);
    prev = r.lat_pos;
  }
  assert(r.lat_pos < 0.5); // converged most of the way to centre

  // With the ambient spring off, an offset rider stays put.
  LateralSolver off(default_params());
  auto s = rider(1, 0, 3.0);
  auto upd = off.solve({s}, 0.1);
  assert(upd[0].new_lat_pos == s.lat_pos);
}

int main() {
  test_contact_detection();
  test_stronger_rider_pushes();
  test_solver_moves_weaker_rider();
  test_tiebreak_direction();
  test_lat_target_steering();
  test_dt_independence();
  test_front_rider_never_penalized();
  test_blocked_squeezer_penalized();
  test_unblocked_overtaker_free();
  test_side_by_side_no_penalty();
  test_squeeze_ramp_monotonic();
  test_wall_penalizes_only_squeezer();
  test_shove_symmetry_blend();
  test_ambient_centering();

  std::cout << "All tests passed\n";
}
