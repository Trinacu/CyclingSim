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
    return (std::abs(q.a_lat_delta - other.q.a_lat_delta) <= epsilon) &&
           (std::abs(q.b_lat_delta - other.q.b_lat_delta) <= epsilon) &&
           (std::abs(q.a_speed_penalty - other.q.a_speed_penalty) <= epsilon) &&
           (std::abs(q.b_speed_penalty - other.q.b_speed_penalty) <= epsilon);
  }

  void print() const {
    std::cout << "(" << q.a_lat_delta << ", " << q.b_lat_delta << ", "
              << q.a_speed_penalty << ", " << q.b_speed_penalty << ")";
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

  double dt = 0.01;
  auto out = solver.compute_shove(strong, weak, pair, dt);

  assert(out.b_lat_delta > 0); // weak rider pushed right
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
  auto params = default_params();
  LateralSolver solver(params);

  auto strong = rider(1, 0, 0, 200);
  auto weak = rider(2, 0, 0.4, 50);

  LateralSolver::ContactPair pair{
      .a_idx = 0, .b_idx = 1, .lon_sep = 0, .lat_sep = 0.4};

  std::vector<double> dts = {0.01, 0.05, 0.1, 0.2};
  std::vector<LateralSolver::ShoveOutcome> res;
  res.reserve(dts.size());

  for (double dt : dts) {
    res.emplace_back(solver.compute_shove(strong, weak, pair, dt));
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
