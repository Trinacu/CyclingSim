// Tests for the D1 drafting aero model (drafting.h) — the pure solver is
// exercised on flat DraftRiderState inputs, mirroring test_lateral_physics /
// test_group_tracker, plus one engine-level integration test at the end.

#include "drafting.h"

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

// State factory with the same defaults the engine would supply for a road
// rider in still air: radius 0.5, bike_len 1.5, no wind components.
static DraftRiderState ds(int id, double lon, double lat = 0.0) {
  DraftRiderState r{};
  r.id = id;
  r.lon_pos = lon;
  r.lat_pos = lat;
  r.speed = 10.0;
  return r;
}

static void test_gap_falloff() {
  const DraftingParams p{};
  check(approx(draft_gap_falloff(-1.0, p), 1.0), "falloff: overlap clamps to 1");
  check(approx(draft_gap_falloff(0.0, p), 1.0), "falloff(0) = 1");
  check(approx(draft_gap_falloff(2.5, p), 0.85), "falloff(2.5) = 0.85");
  check(approx(draft_gap_falloff(5.0, p), 0.7), "falloff(5) = 0.7 (knee)");
  check(approx(draft_gap_falloff(6.5, p), 0.35), "falloff(6.5) = 0.35");
  check(approx(draft_gap_falloff(8.0, p), 0.0), "falloff(8) = 0");
  check(approx(draft_gap_falloff(10.0, p), 0.0), "falloff(10) = 0");
}

static void test_lone_and_pair() {
  const DraftingParams p{};

  check(approx(compute_draft_factors({ds(1, 0)}, p)[0], 1.0),
        "lone rider: factor 1");

  // Nose-to-tail: leader 1.5 m ahead -> wheel gap 0.
  const auto f = compute_draft_factors({ds(1, 0), ds(2, 1.5)}, p);
  check(approx(f[0], 0.61), "nose-to-tail follower = table[1] = 0.61");
  check(approx(f[1], 0.98), "nose-to-tail leader = table[0] = 0.98");
}

static void test_gap_modulation() {
  const DraftingParams p{};

  // Wheel gap 5 m: 70% of the benefit remains.
  auto f = compute_draft_factors({ds(1, 0), ds(2, 6.5)}, p);
  check(approx(f[0], 1.0 - 0.39 * 0.7), "gap 5: follower = 1 - 0.39*0.7");
  check(approx(f[1], 1.0 - 0.02 * 0.7), "gap 5: leader = 1 - 0.02*0.7");

  // Wheel gap 8 m: no draft, no link, no leader push.
  f = compute_draft_factors({ds(1, 0), ds(2, 9.5)}, p);
  check(approx(f[0], 1.0) && approx(f[1], 1.0), "gap 8: both 1.0");
}

static void test_lateral_offset() {
  const DraftingParams p{};

  // Half the cutoff (0.75 m of 3*0.5 m): benefit exactly halved.
  auto f = compute_draft_factors({ds(1, 0, 0.75), ds(2, 1.5)}, p);
  check(approx(f[0], 1.0 - 0.39 * 0.5), "offset 0.75: follower benefit halved");
  check(approx(f[1], 1.0 - 0.02 * 0.5), "offset 0.75: leader push halved");

  // At the cutoff: no link at all.
  f = compute_draft_factors({ds(1, 0, 1.5), ds(2, 1.5)}, p);
  check(approx(f[0], 1.0) && approx(f[1], 1.0), "offset 1.5: no draft");
}

static void test_chain_saturation() {
  const DraftingParams p{};

  // 8 riders spaced one bike length apart (wheel gap 0 everywhere).
  std::vector<DraftRiderState> riders;
  for (int i = 0; i < 8; ++i)
    riders.push_back(ds(i, 1.5 * (7 - i))); // index 0 is the front rider

  const auto f = compute_draft_factors(riders, p);
  check(approx(f[0], 0.98), "chain: front = 0.98");
  check(approx(f[1], 0.61), "chain: P2 = 0.61");
  check(approx(f[2], 0.50), "chain: P3 = 0.50");
  check(approx(f[3], 0.44), "chain: P4 = 0.44");
  check(approx(f[4], 0.42), "chain: P5 = 0.42");
  check(approx(f[5], 0.41), "chain: P6 = 0.41");
  check(approx(f[6], 0.41) && approx(f[7], 0.41),
        "chain: P7+ saturate at 0.41");
}

static void test_side_by_side_chains() {
  const DraftingParams p{};

  // Two 2-rider chains 3 m apart laterally (beyond the 1.5 m cutoff):
  // each follower must link to its own leader, not the neighbouring one.
  const auto f = compute_draft_factors(
      {ds(1, 0, 0.0), ds(2, 1.5, 0.0), ds(3, 0, 3.0), ds(4, 1.5, 3.0)}, p);
  check(approx(f[0], 0.61) && approx(f[2], 0.61),
        "parallel chains: both followers at 0.61");
  check(approx(f[1], 0.98) && approx(f[3], 0.98),
        "parallel chains: both leaders at 0.98");
}

static void test_wake_axis_lat() {
  // Still air: the axis is the leader's own line at any distance.
  DraftRiderState leader = ds(1, 10.0, 0.4);
  check(approx(wake_axis_lat(leader, 7.0), 0.4),
        "wake_axis_lat: still air -> leader's line");

  // 5 m/s crosswind at 10 m/s: the axis tilts half a metre per metre behind.
  leader.crosswind = 5.0;
  check(approx(wake_axis_lat(leader, 7.0), 0.4 + 3.0 * 0.5),
        "wake_axis_lat: crosswind tilts the axis leeward");
  check(approx(wake_axis_lat(leader, 10.0), 0.4),
        "wake_axis_lat: zero offset at the leader itself");
}

static void test_crosswind_axis() {
  const DraftingParams p{};

  // Leader at 10 m/s with 5 m/s crosswind: its wake axis tilts sideways by
  // lon_sep * 5/10.  At 3 m behind, the wake sits 1.5 m to the side —
  // exactly the cutoff for a rider directly behind.
  DraftRiderState leader = ds(3, 3.0);
  leader.crosswind = 5.0;

  DraftRiderState behind = ds(1, 0.0, 0.0);  // in the leader's lon shadow
  DraftRiderState leeward = ds(2, 0.0, 1.5); // sitting in the shifted wake

  const auto f = compute_draft_factors({behind, leeward, leader}, p);
  check(approx(f[0], 1.0), "crosswind: directly-behind rider gets no draft");
  const double s = draft_gap_falloff(1.5, p); // on-axis, wheel gap 1.5
  check(approx(f[1], 1.0 - 0.39 * s), "crosswind: leeward rider drafts");
  check(approx(f[2], 1.0 - 0.02 * s), "crosswind: leader pushed by leeward");
}

static void test_split_continuity() {
  const DraftingParams p{};

  // 3-rider chain whose front link sits just inside vs. just outside
  // max_draft_gap.  Everyone's factor must move only marginally across the
  // split — this is what continuous depth and the (1-s)-weighted leader push
  // buy us (integer depth would step the tail rider 0.50 -> 0.61, and the
  // middle rider would step by the full 2% push on becoming a chain head).
  auto chain = [&](double front_gap) {
    return compute_draft_factors(
        {ds(1, 0), ds(2, 1.5), ds(3, 3.0 + front_gap)}, p);
  };
  const auto near = chain(7.99);
  const auto past = chain(8.01);
  check(std::fabs(near[0] - past[0]) < 0.005,
        "split: tail rider continuous across chain break");
  check(std::fabs(near[1] - past[1]) < 0.005,
        "split: new chain head continuous across chain break");
  check(approx(past[0], 0.61) && approx(past[1], 0.98),
        "split: after break the pair behaves as a fresh 2-chain");

  // Knee of the gap falloff is a slope change, not a step.
  const auto lo = compute_draft_factors({ds(1, 0), ds(2, 6.5 - 1e-4)}, p);
  const auto hi = compute_draft_factors({ds(1, 0), ds(2, 6.5 + 1e-4)}, p);
  check(std::fabs(lo[0] - hi[0]) < 1e-3, "knee: follower continuous at 5 m");
}

// D3.0: the link picks the best wheel, not the nearest one.
static void test_best_wheel_selection() {
  const DraftingParams p{};

  // A merging rider (id 2) sits right on id 1's wheel but 1.2 m off the
  // line; the line wheel (id 3) is a whole position further up.  Nearest
  // linking would take the weak sliver of id 2's wake; best-wheel linking
  // takes id 3's well-aligned wheel through the gap.
  const DraftRiderState me = ds(1, 0.0);
  const DraftRiderState merger = ds(2, 1.6, 1.2);
  const DraftRiderState line = ds(3, 3.2);
  const auto f = compute_draft_factors({me, merger, line}, p);

  // Benefit via the line wheel: depth 1, gap 1.7, on-axis.
  const double expected = 1.0 - 0.39 * draft_gap_falloff(1.7, p);
  check(approx(f[0], expected, 1e-6),
        "best-wheel: drafts the aligned far wheel, not the offset near one");

  // Fully offset merger: same link (this also held under the old rule).
  const DraftRiderState merger_out = ds(2, 1.6, 1.5);
  const auto f2 = compute_draft_factors({me, merger_out, line}, p);
  check(approx(f2[0], expected, 1e-6),
        "best-wheel: fully offset near wheel is skipped entirely");
}

// D3.0: cda_factor stays continuous while a merging rider sweeps laterally
// across the follower's link-switch point.
static void test_link_switch_continuity() {
  const DraftingParams p{};

  double prev = -1.0, max_step = 0.0;
  for (double lat = 0.0; lat <= 2.0 + 1e-9; lat += 0.01) {
    const auto f =
        compute_draft_factors({ds(1, 0.0), ds(2, 1.6, lat), ds(3, 3.2)}, p);
    if (prev >= 0.0)
      max_step = std::max(max_step, std::fabs(f[0] - prev));
    prev = f[0];
  }
  check(max_step < 0.02,
        "link switch: factor continuous under lateral sweep of the merger");
}

// D3.0: candidate cap — only the link_candidates closest in-range wheels are
// considered.  Three offset wheels between me and an aligned one starve the
// link at the default cap; raising the cap restores it.
static void test_link_candidate_cap() {
  DraftingParams p{};

  const std::vector<DraftRiderState> riders = {
      ds(1, 0.0),      ds(2, 1.6, 3.0), ds(3, 1.7, 3.0),
      ds(4, 1.8, 3.0), ds(5, 3.5)};

  const auto f = compute_draft_factors(riders, p);
  check(approx(f[0], 1.0), "cap: 3 offset wheels starve the link (documented)");

  p.link_candidates = 4;
  const auto f4 = compute_draft_factors(riders, p);
  check(f4[0] < 0.75, "cap: raising link_candidates reaches the aligned wheel");
}

static void test_body_role() {
  const DraftingParams p{};

  auto body = [](int id, double lon) {
    DraftRiderState r = ds(id, lon);
    r.group_id = 0;
    r.role = GroupRole::Body;
    return r;
  };
  auto chain = [](int id, double lon) {
    DraftRiderState r = ds(id, lon);
    r.group_id = 0;
    return r;
  };

  // Body rider with two group members ahead inside the window -> curve[2].
  auto f = compute_draft_factors({body(1, 0), chain(2, 3.0), chain(3, 6.0)}, p);
  check(approx(f[0], 0.50), "body: two ahead in window -> curve[2] = 0.50");

  // Exposed body rider (nobody ahead) -> curve[0], not 1.0.
  f = compute_draft_factors({body(1, 3.0), chain(2, 0.0)}, p);
  check(approx(f[0], 0.90), "body: front edge -> curve[0] = 0.90");

  // A Body rider's wheel still shelters a chain rider behind it.
  f = compute_draft_factors({chain(1, 0), body(2, 1.5)}, p);
  check(approx(f[0], 0.61), "body rider anchors a chain link behind it");
}

// Engine-level: two riders on a flat course, the stronger one rides away
// from a standing start while the weaker one sits in its draft as the gap
// opens through the draftable range.  The drafting follower must end up
// further down the road than an identical solo rider at the same effort.
static void test_engine_integration() {
  const double dt = 0.01;
  const int steps = 3000; // 30 s

  Course duo_course = Course::create_flat();
  PhysicsEngine duo(&duo_course);

  auto cfg = [](int id) {
    return RiderConfig{id,  "R" + std::to_string(id),
                       250, 6,
                       2,   0.05,
                       700, 3.5,
                       65,  0.3,
                       24000, Bike::create_road(),
                       Team("T")};
  };
  duo.add_rider(cfg(1));
  duo.add_rider(cfg(2));
  duo.set_rider_effort(1, 1.1); // rides away
  duo.set_rider_effort(2, 0.9); // sits in the draft while the gap opens

  double min_factor_follower = 1.0;
  double min_factor_leader = 1.0;
  for (int i = 0; i < steps; ++i) {
    duo.update(dt);
    min_factor_follower =
        std::min(min_factor_follower, duo.get_rider_by_id(2)->get_cda_factor());
    min_factor_leader =
        std::min(min_factor_leader, duo.get_rider_by_id(1)->get_cda_factor());
  }

  Course solo_course = Course::create_flat();
  PhysicsEngine solo(&solo_course);
  solo.add_rider(cfg(2));
  solo.set_rider_effort(2, 0.9);
  for (int i = 0; i < steps; ++i)
    solo.update(dt);

  const double duo_pos = duo.get_rider_by_id(2)->get_pos();
  const double solo_pos = solo.get_rider_by_id(2)->get_pos();

  std::cout << "  [integration] follower pos " << duo_pos << " m vs solo "
            << solo_pos << " m; min factors follower "
            << min_factor_follower << ", leader " << min_factor_leader
            << "\n";
  check(min_factor_follower < 0.8, "integration: follower drafted (<0.8)");
  check(min_factor_leader > 0.9,
        "integration: leader never deeply sheltered");
  check(duo_pos > solo_pos + 0.5,
        "integration: drafting follower beats identical solo rider");
}

int main() {
  test_gap_falloff();
  test_lone_and_pair();
  test_gap_modulation();
  test_lateral_offset();
  test_chain_saturation();
  test_side_by_side_chains();
  test_wake_axis_lat();
  test_crosswind_axis();
  test_split_continuity();
  test_best_wheel_selection();
  test_link_switch_continuity();
  test_link_candidate_cap();
  test_body_role();
  test_engine_integration();

  if (checks_failed) {
    std::cout << checks_failed << " check(s) FAILED\n";
    return 1;
  }
  std::cout << "All drafting tests passed\n";
  return 0;
}
