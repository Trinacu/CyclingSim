// Tests for the C1 CourseIntel digest (course_intel.h): climb merging on a
// synthetic profile, queries on create_endulating, edge cases.

#include "course_intel.h"

#include <cmath>
#include <iostream>
#include <string>

static int checks_failed = 0;

static void check(bool ok, const std::string& label) {
  if (ok) {
    std::cout << "  ok: " << label << "\n";
  } else {
    std::cout << "FAIL: " << label << "\n";
    ++checks_failed;
  }
}

static bool near(double a, double b, double eps = 1e-9) {
  return std::fabs(a - b) < eps;
}

// Synthetic profile exercising every merge rule:
//   1 km flat | 500 @6% | 100 flat dip | 500 @8% | 1 km flat |
//   100 @10% (too short) | 1 km flat | 300 @-4% | 400 @3% | rest flat.
static void test_climb_merging() {
  Course course = Course::from_segments({{1000, 0.0, 0, 0, 8},
                                         {500, 0.06, 0, 0, 8},
                                         {100, 0.0, 0, 0, 8},
                                         {500, 0.08, 0, 0, 8},
                                         {1000, 0.0, 0, 0, 8},
                                         {100, 0.10, 0, 0, 8},
                                         {1000, 0.0, 0, 0, 8},
                                         {300, -0.04, 0, 0, 8},
                                         {400, 0.03, 0, 0, 8},
                                         {1000, 0.0, 0, 0, 8}});
  CourseIntel intel(course);

  check(intel.climbs().size() == 2, "merge: two climbs found");
  if (intel.climbs().size() != 2)
    return;

  // Climb 1: 1000 -> 2100 across the 100 m dip; 70 m over 1100 m.
  const Climb& c1 = intel.climbs()[0];
  check(near(c1.start, 1000.0) && near(c1.crest_pos, 2100.0),
        "merge: short dip bridged into one climb");
  check(near(c1.avg_gradient, 70.0 / 1100.0),
        "merge: avg gradient spans the dip");

  // The 100 m @10% bump is dropped (min_climb_len).
  // Climb 2: the 400 m @3% after a descent (descent not merged in).
  const Climb& c2 = intel.climbs()[1];
  check(near(c2.start, 4500.0) && near(c2.length, 400.0),
        "merge: bump dropped, shallow climb kept, descent excluded");
}

// A long descent between uphill runs must split them even though a short
// dip would not; a merged run diluted below min_gradient is dropped.
static void test_split_and_dilution() {
  Course a = Course::from_segments({{500, 0.05, 0, 0, 8},
                                    {300, -0.05, 0, 0, 8},
                                    {500, 0.05, 0, 0, 8}});
  check(CourseIntel(a).climbs().size() == 2,
        "split: 300 m descent separates two climbs");

  // 200 m @2.5% + 200 m dip at -3% + 200 m @2.5%: merged avg ~0.7% -> gone.
  Course b = Course::from_segments({{200, 0.025, 0, 0, 8},
                                    {200, -0.03, 0, 0, 8},
                                    {200, 0.025, 0, 0, 8},
                                    {400, 0.0, 0, 0, 8}});
  check(CourseIntel(b).climbs().empty(),
        "dilution: merged run below min_gradient dropped");
}

static void test_endulating_queries() {
  Course course = Course::create_endulating();
  CourseIntel intel(course);

  // Profile: 1000 flat | 200 @10% | 200 flat | 2000 @10% | 2000 flat |
  //          2000 @10% | 2000 flat | 5000 @5%.  The 200 m flat shoulder
  //          merges -> climbs: [1000, 3400], [5400, 7400], [9400, 14400].
  check(intel.climbs().size() == 3, "endulating: three climbs");
  if (intel.climbs().size() != 3)
    return;
  check(near(intel.climbs()[0].start, 1000.0) &&
            near(intel.climbs()[0].crest_pos, 3400.0),
        "endulating: first climb spans the 200 m shoulder");
  check(near(intel.climbs()[0].avg_gradient, 220.0 / 2400.0),
        "endulating: first climb avg gradient");
  check(near(intel.climbs()[2].crest_pos, 14400.0),
        "endulating: last crest at the finish");

  check(near(intel.total_length(), 14400.0), "endulating: total length");
  check(near(intel.distance_to_finish(400.0), 14000.0),
        "endulating: distance to finish");

  // next_climb: before / inside / exactly at a crest / past the last crest.
  check(near(intel.next_climb(0.0)->start, 1000.0),
        "endulating: next climb from the start");
  check(near(intel.next_climb(2000.0)->crest_pos, 3400.0),
        "endulating: midway up -> the current climb");
  check(near(intel.next_climb(3400.0)->crest_pos, 7400.0),
        "endulating: at a crest -> the following climb");
  check(!intel.next_climb(14400.0).has_value(),
        "endulating: past the last crest -> nullopt");

  check(near(*intel.distance_to_crest(2000.0), 1400.0),
        "endulating: distance to crest inside a climb");
  check(!intel.distance_to_crest(14400.0).has_value(),
        "endulating: distance to crest nullopt at the end");

  // avg_gradient over a mixed window: 2000..4400 = 1400 m @10% + 1000 flat.
  check(near(intel.avg_gradient(2000.0, 4400.0), 140.0 / 2400.0),
        "endulating: avg gradient over a mixed window");
  check(near(intel.avg_gradient(500.0, 500.0), 0.0),
        "endulating: degenerate window -> 0");
}

int main() {
  test_climb_merging();
  test_split_and_dilution();
  test_endulating_queries();

  if (checks_failed) {
    std::cout << checks_failed << " check(s) FAILED\n";
    return 1;
  }
  std::cout << "All course intel tests passed\n";
  return 0;
}
