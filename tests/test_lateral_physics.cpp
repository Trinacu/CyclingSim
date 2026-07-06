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

  auto out = solver.compute_shove(strong, weak, pair);

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

int main() {
  test_contact_detection();
  test_stronger_rider_pushes();
  test_solver_moves_weaker_rider();
  test_tiebreak_direction();
  test_lat_target_steering();
  test_dt_independence();

  std::cout << "All tests passed\n";
}
