// Tests for workstream B — wind.
//
// B1: wind field on Course, per-segment rider heading, and the longitudinal
// projection into the core's scalar env->headwind (the sign-fix proof lives
// in tests/core/test_wind_core.c).  Mirrors test_drafting / test_follow.
//
// B2: crosswind -> yaw-dependent longitudinal drag via the cda_factor split
// (draft x yaw), including the echelon integration payoff: shelter on the
// rotated wake axis vs. yaw-exposed riding at matched speed.

#include "course.h"
#include "lateral_behavior.h"
#include "rider.h"
#include "sim.h"

#include <cmath>
#include <iostream>
#include <string>
#include <vector>

static int checks_failed = 0;

static void check(bool ok, const std::string& label) {
  if (ok) {
    std::cout << "  ok: " << label << "\n";
  } else {
    std::cout << "FAIL: " << label << "\n";
    ++checks_failed;
  }
}

static bool approx(double a, double b, double eps = 1e-9) {
  return std::fabs(a - b) <= eps;
}

static RiderConfig cfg(int id, double ftp = 250) {
  return RiderConfig{id,  "R" + std::to_string(id),
                     ftp, 6,
                     2,   0.05,
                     700, 3.5,
                     65,  0.3,
                     24000, Bike::create_road(),
                     Team("T")};
}

// Three equal flat legs pointing 0, pi/2 and pi — with a wind blowing from
// heading 0 that is a headwind leg, a pure-crosswind leg and a tailwind leg.
static Course three_leg_course() {
  return Course::from_segments({{2000, 0, 0, 0, 8},
                                {2000, 0, 0, M_PI / 2.0, 8},
                                {2000, 0, 0, M_PI, 8}});
}

// --- B1: wind field + heading ---

static void test_default_wind_is_zero() {
  const Course flat = Course::create_flat();
  const Wind w = flat.get_wind(100.0);
  check(approx(w.speed, 0.0) && approx(w.heading, 0.0),
        "factory course wind defaults to 0 (phantom 1 m/s gone)");
}

static void test_set_wind_roundtrip() {
  Course flat = Course::create_flat();
  flat.set_wind({1.25, 4.0});
  const Wind w = flat.get_wind(500.0);
  check(approx(w.heading, 1.25) && approx(w.speed, 4.0),
        "set_wind round-trips through get_wind");
}

static void test_get_heading_per_segment() {
  const Course c = three_leg_course();
  check(approx(c.get_heading(100.0), 0.0), "heading of leg 1 is 0");
  check(approx(c.get_heading(2500.0), M_PI / 2.0), "heading of leg 2 is pi/2");
  check(approx(c.get_heading(4500.0), M_PI), "heading of leg 3 is pi");
}

// Rider heading follows the course, and the cos projection makes the same
// wind a headwind, a crosswind and a tailwind on the three legs: end-of-leg
// speeds must order headwind < crosswind < tailwind.
static void test_projection_ordering() {
  const double dt = 0.01;
  Course course = three_leg_course();
  course.set_wind({0.0, 3.0}); // blows from heading 0
  PhysicsEngine eng(&course);
  eng.add_rider(cfg(1));
  eng.set_rider_effort(1, 1.0);

  const Rider* r = eng.get_rider_by_id(1);
  double v_end[3] = {0.0, 0.0, 0.0};
  bool heading_tracked = true;
  for (int i = 0; i < 200000 && r->get_pos() < 5900.0; ++i) {
    const double pre_pos = r->get_pos(); // heading is set pre-integration
    eng.update(dt);
    if (!approx(r->get_heading(), course.get_heading(pre_pos)))
      heading_tracked = false;
    const int leg = static_cast<int>(r->get_pos() / 2000.0);
    if (leg >= 0 && leg < 3)
      v_end[leg] = r->get_speed();
  }

  std::cout << "  [projection] end-of-leg speeds: head " << v_end[0]
            << ", cross " << v_end[1] << ", tail " << v_end[2] << " m/s\n";
  check(heading_tracked, "rider heading tracks the course segment heading");
  check(r->get_pos() >= 5900.0, "rider traversed all three legs");
  check(v_end[0] < v_end[1] - 0.3, "headwind leg slower than crosswind leg");
  check(v_end[1] < v_end[2] - 0.3, "crosswind leg slower than tailwind leg");
}

// --- B2: crosswind -> yaw drag ---

// Terminal speed of a lone rider at effort 1.0 on a flat course under `wind`.
static double terminal_speed(Wind wind, double seconds = 300.0) {
  const double dt = 0.01;
  Course course = Course::create_flat();
  course.set_wind(wind);
  PhysicsEngine eng(&course);
  eng.add_rider(cfg(1));
  eng.set_rider_effort(1, 1.0);
  const int steps = static_cast<int>(seconds / dt);
  for (int i = 0; i < steps; ++i)
    eng.update(dt);
  return eng.get_rider_by_id(1)->get_speed();
}

static void test_crosswind_slows_symmetrically() {
  const double v_still = terminal_speed({0.0, 0.0});
  const double v_left = terminal_speed({M_PI / 2.0, 5.0});
  const double v_right = terminal_speed({-M_PI / 2.0, 5.0});

  std::cout << "  [crosswind] still " << v_still << ", cross " << v_left
            << " m/s\n";
  check(v_left < v_still - 0.3, "pure crosswind lowers terminal speed");
  check(approx(v_left, v_right), "crosswind cost is symmetric in +-c");
}

// Guard for every pre-wind suite: in still air the yaw factor is exactly 1
// and a lone rider's total cda_factor stays exactly 1.
static void test_still_air_yaw_factor_is_one() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  PhysicsEngine eng(&course);
  eng.add_rider(cfg(1));
  eng.set_rider_effort(1, 1.0);
  bool yaw_one = true, cda_one = true;
  for (int i = 0; i < 2000; ++i) {
    eng.update(dt);
    const Rider* r = eng.get_rider_by_id(1);
    yaw_one = yaw_one && r->get_yaw_factor() == 1.0;
    cda_one = cda_one && r->get_cda_factor() == 1.0;
  }
  check(yaw_one, "still air: yaw_factor == 1 exactly");
  check(cda_one, "still air: lone rider's cda_factor == 1 exactly");
}

static void test_standing_start_crosswind_capped() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  course.set_wind({M_PI / 2.0, 10.0}); // strong pure crosswind
  PhysicsEngine eng(&course);
  eng.add_rider(cfg(1));
  eng.set_rider_effort(1, 1.0);

  double max_yaw = 0.0;
  for (int i = 0; i < 6000; ++i) { // 60 s from rest
    eng.update(dt);
    max_yaw = std::max(max_yaw, eng.get_rider_by_id(1)->get_yaw_factor());
  }
  const double v = eng.get_rider_by_id(1)->get_speed();
  std::cout << "  [standing start] max yaw factor " << max_yaw << ", speed "
            << v << " m/s\n";
  check(std::isfinite(v) && v > 5.0,
        "standing start in strong crosswind stays finite and rideable");
  check(max_yaw <= 3.0 + 1e-12, "yaw factor respects the cap");
  check(max_yaw > 1.0, "crosswind actually engaged the yaw factor");
}

static void test_determinism() {
  auto run = [](Course& course) {
    const double dt = 0.01;
    PhysicsEngine eng(&course);
    eng.add_rider(cfg(1));
    eng.add_rider(cfg(2));
    eng.set_rider_effort(1, 1.0);
    eng.set_follow_target(2, 1);
    for (int i = 0; i < 5000; ++i)
      eng.update(dt);
    return std::pair<double, double>{eng.get_rider_by_id(1)->get_pos(),
                                     eng.get_rider_by_id(2)->get_pos()};
  };
  Course a = Course::create_flat();
  a.set_wind({1.0, 4.0});
  Course b = Course::create_flat();
  b.set_wind({1.0, 4.0});
  const auto ra = run(a);
  const auto rb = run(b);
  check(ra.first == rb.first && ra.second == rb.second,
        "determinism: identical runs land on identical positions");
}

// The payoff: leader + follower steering to the rotated wake axis + an
// exposed chaser pinned to the road centre (HoldLineBehavior clears the
// wake steering; ambient centering holds it there).  All three hold gaps,
// so speeds are matched — but only the aligned follower is sheltered, and
// the exposed chaser burns W' while the sheltered one coasts below FTP.
static void test_echelon_integration() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  course.set_wind({M_PI / 2.0, 5.0}); // pure crosswind, c = +5 (leeward = +)
  PhysicsEngine eng(&course);
  eng.add_rider(cfg(1));
  eng.add_rider(cfg(2));
  eng.add_rider(cfg(3));
  eng.set_rider_effort(1, 1.15); // above FTP so exposure costs W'
  eng.set_follow_target(2, 1);
  eng.set_follow_target(3, 2);
  eng.set_rider_behavior(3, std::make_shared<HoldLineBehavior>());

  for (int i = 0; i < 12000; ++i) // 120 s
    eng.update(dt);

  const Rider* leader = eng.get_rider_by_id(1);
  const Rider* aligned = eng.get_rider_by_id(2);
  const Rider* exposed = eng.get_rider_by_id(3);

  std::cout << "  [echelon] lat_target " << aligned->get_lat_target().value_or(-99)
            << " m; cda aligned " << aligned->get_cda_factor() << " vs exposed "
            << exposed->get_cda_factor() << "; wbal aligned "
            << aligned->get_energy_fraction() << " vs exposed "
            << exposed->get_energy_fraction() << "\n";

  check(std::fabs(aligned->get_speed() - leader->get_speed()) < 0.3 &&
            std::fabs(exposed->get_speed() - leader->get_speed()) < 0.3,
        "echelon: all three hold matched speed");
  check(aligned->get_lat_target().has_value() &&
            aligned->get_lat_target().value() > 0.4,
        "echelon: follower's lat_target sits leeward on the rotated axis");
  check(exposed->get_cda_factor() > 1.0,
        "echelon: exposed chaser pays yaw drag with no shelter");
  check(aligned->get_cda_factor() < exposed->get_cda_factor() - 0.3,
        "echelon: aligned follower's total cda_factor well below exposed");
  check(exposed->get_energy_fraction() < 0.99,
        "echelon: exposed chaser actually burned W'");
  check(exposed->get_energy_fraction() <
            aligned->get_energy_fraction() - 0.05,
        "echelon: exposed chaser burns W' faster than the sheltered one");
}

int main() {
  test_default_wind_is_zero();
  test_set_wind_roundtrip();
  test_get_heading_per_segment();
  test_projection_ordering();

  test_crosswind_slows_symmetrically();
  test_still_air_yaw_factor_is_one();
  test_standing_start_crosswind_capped();
  test_determinism();
  test_echelon_integration();

  if (checks_failed) {
    std::cout << checks_failed << " check(s) FAILED\n";
    return 1;
  }
  std::cout << "All wind tests passed\n";
  return 0;
}
