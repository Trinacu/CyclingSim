// Tests for workstream B — wind.
//
// B1: wind field on Course, per-segment rider heading, and the longitudinal
// projection into the core's scalar env->headwind (the sign-fix proof lives
// in tests/core/test_wind_core.c).  Mirrors test_drafting / test_follow.

#include "course.h"
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

int main() {
  test_default_wind_is_zero();
  test_set_wind_roundtrip();
  test_get_heading_per_segment();
  test_projection_ordering();

  if (checks_failed) {
    std::cout << checks_failed << " check(s) FAILED\n";
    return 1;
  }
  std::cout << "All wind tests passed\n";
  return 0;
}
