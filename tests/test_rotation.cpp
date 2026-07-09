// Tests for the D3 paceline rotation coordinator (rotation.h) — pure
// coordinator units first (synthetic inputs, no engine), then engine-level
// rotation/merge/removal/determinism tests.  Mirrors test_follow.cpp.

#include "rotation.h"

#include "course.h"
#include "rider.h"
#include "sim.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <map>
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

// --- Pure coordinator ---

static RotationInput ri(int id, double pos, double effort = 0.85,
                        double crosswind = 0.0) {
  return RotationInput{.id = id,
                       .lon_pos = pos,
                       .speed = 10.0,
                       .bike_len = 1.5,
                       .crosswind = crosswind,
                       .target_effort = effort};
}

// An engaged line: ids front-first, wheel gaps 0.25 m (spacing 1.75).
static std::vector<RotationInput> engaged_line(const std::vector<int>& ids,
                                               double crosswind = 0.0) {
  std::vector<RotationInput> in;
  for (size_t i = 0; i < ids.size(); ++i)
    in.push_back(ri(ids[i], 100.0 - 1.75 * i, 0.85, crosswind));
  return in;
}

static const RotationDirective* dir_for(const std::vector<RotationDirective>& d,
                                        int id) {
  for (const auto& x : d)
    if (x.id == id)
      return &x;
  return nullptr;
}

static std::vector<RotationMember> rotators(const std::vector<int>& ids) {
  std::vector<RotationMember> r;
  for (int id : ids)
    r.push_back({id, false});
  return r;
}

// Initial directives express the follow graph: puller pulls (no follow),
// InLine members follow the previous rider, SittingIn chains behind the tail.
static void test_roster_directives() {
  auto roster = rotators({1, 2, 3});
  roster.push_back({4, true}); // sits in
  roster.push_back({5, true});
  PacelineRotation rot(roster, RotationParams{});

  const auto d = rot.tick(0.01, engaged_line({1, 2, 3, 4, 5}));
  check(d.size() == 5, "roster: one directive per member");
  check(dir_for(d, 1) && dir_for(d, 1)->pulling, "roster: front pulls");
  check(dir_for(d, 1) && !dir_for(d, 1)->set_effort.has_value(),
        "roster: no effort inheritance without a promotion");
  check(dir_for(d, 2) && dir_for(d, 2)->follow == 1, "roster: 2 follows 1");
  check(dir_for(d, 3) && dir_for(d, 3)->follow == 2, "roster: 3 follows 2");
  check(dir_for(d, 4) && dir_for(d, 4)->follow == 3,
        "roster: first SittingIn follows the tail");
  check(dir_for(d, 5) && dir_for(d, 5)->follow == 4,
        "roster: SittingIn chain behind each other");
  check(rot.puller() == 1 && rot.inline_count() == 3 && rot.member_count() == 5,
        "roster: introspection counts");
}

// After pull_time engaged, the front swings off (side set, follows the tail),
// the second rider is promoted and inherits the old puller's target_effort.
static void test_swing_and_promotion() {
  RotationParams rp;
  rp.pull_time = 1.0;
  PacelineRotation rot(rotators({1, 2, 3, 4}), rp);

  // Capture the directives of the swing tick itself: set_effort and
  // swing_side are one-shot.
  const auto in = engaged_line({1, 2, 3, 4});
  std::vector<RotationDirective> d;
  for (int i = 0; i < 105 && rot.puller() == 1; ++i) // <= 1.05 s at 100 Hz
    d = rot.tick(0.01, in);

  check(rot.puller() == 2, "swing: second rider promoted");
  check(rot.drifting_count() == 1, "swing: old puller drifting");
  const auto* p = dir_for(d, 2);
  check(p && p->pulling, "swing: promoted directive pulls");
  check(p && p->set_effort.has_value() &&
            std::fabs(*p->set_effort - 0.85) < 1e-12,
        "swing: promoted inherits the old puller's target_effort");
  const auto* s = dir_for(d, 1);
  check(s && !s->pulling && s->follow == 4,
        "swing: drifter follows the last InLine rider");
  check(s && s->swing_side == 1.0,
        "swing: default side (+1) with zero crosswind");

  // set_effort and swing_side are one-shot: gone on the next tick.
  d = rot.tick(0.01, in);
  check(dir_for(d, 2) && !dir_for(d, 2)->set_effort.has_value(),
        "swing: effort inheritance is one-shot");
  check(dir_for(d, 1) && dir_for(d, 1)->swing_side == 0.0,
        "swing: swing_side is one-shot");
}

// The drifter always swings windward: opposite the +crosswind (leeward wake)
// side.
static void test_crosswind_side() {
  RotationParams rp;
  rp.pull_time = 0.5;

  // Tick to the swing (swing_side is one-shot) and report the side taken.
  auto swing_side = [&rp](double crosswind, double default_side) {
    RotationParams p = rp;
    p.default_side = default_side;
    PacelineRotation rot(rotators({1, 2, 3}), p);
    std::vector<RotationDirective> d;
    for (int i = 0; i < 60 && rot.puller() == 1; ++i)
      d = rot.tick(0.01, engaged_line({1, 2, 3}, crosswind));
    const auto* s = dir_for(d, 1);
    return s ? s->swing_side : 0.0;
  };

  check(swing_side(2.0, 1.0) == -1.0,
        "crosswind: +crosswind swings to -1 (windward)");
  check(swing_side(-2.0, 1.0) == 1.0, "crosswind: -crosswind swings to +1");
  check(swing_side(0.0, -1.0) == -1.0,
        "crosswind: default_side honoured in still air");
}

// The pull timer only advances while the line is engaged, and a rotation
// never leaves fewer than 2 riders in line.
static void test_trigger_guards() {
  RotationParams rp;
  rp.pull_time = 0.5;

  {
    // Puller 5 m off the front: never engaged, never rotates.
    PacelineRotation rot(rotators({1, 2, 3}), rp);
    std::vector<RotationInput> in = engaged_line({1, 2, 3});
    in[0].lon_pos += 5.0;
    for (int i = 0; i < 500; ++i)
      rot.tick(0.01, in);
    check(rot.puller() == 1 && rot.drifting_count() == 0,
          "guard: unengaged line never rotates");
  }
  {
    // Two rotators: a swing would leave 1 in line — blocked forever.
    PacelineRotation rot(rotators({1, 2}), rp);
    for (int i = 0; i < 500; ++i)
      rot.tick(0.01, engaged_line({1, 2}));
    check(rot.puller() == 1 && rot.drifting_count() == 0,
          "guard: no rotation below 3 in line");
  }
}

// Attach is positional and one-shot: a drifter whose position drops below the
// tail's joins the back of the line; two drifters crossing in the same tick
// append in spatial order.
static void test_attach() {
  RotationParams rp;
  rp.pull_time = 0.2;
  PacelineRotation rot(rotators({1, 2, 3, 4, 5}), rp);

  // Two swings 0.2 s apart -> drifters {1, 2}, inline {3, 4, 5}.
  std::map<int, double> pos = {{1, 100.0}, {2, 98.25}, {3, 96.5},
                               {4, 94.75}, {5, 93.0}};
  auto inputs = [&]() {
    std::vector<RotationInput> in;
    for (const auto& [id, p] : pos)
      in.push_back(ri(id, p));
    return in;
  };
  for (int i = 0; i < 45; ++i)
    rot.tick(0.01, inputs());
  check(rot.drifting_count() == 2 && rot.puller() == 3,
        "attach: two drifters in flight after two quick swings");

  // Both cross below the tail (5, at 93.0) in the same tick, 2 ahead of 1.
  pos[2] = 92.0;
  pos[1] = 91.0;
  const auto d = rot.tick(0.01, inputs());
  check(rot.drifting_count() == 0 && rot.inline_count() == 5,
        "attach: both drifters appended");
  check(dir_for(d, 2) && dir_for(d, 2)->follow == 5,
        "attach: higher drifter appends first (follows old tail)");
  check(dir_for(d, 1) && dir_for(d, 1)->follow == 2,
        "attach: lower drifter appends last (follows the other)");
}

// The front sequence cycles through all rotators repeatedly, in line order.
// Positions are synthetic: drifters are teleported below the tail one tick
// after swinging, so cycles are fast and purely order-driven.
static void test_pull_cycle_order() {
  RotationParams rp;
  rp.pull_time = 0.5;
  PacelineRotation rot(rotators({1, 2, 3, 4}), rp);

  std::map<int, double> pos = {{1, 100.0}, {2, 98.25}, {3, 96.5}, {4, 94.75}};
  double floor_pos = 94.75; // lowest occupied position so far
  auto inputs = [&]() {
    std::vector<RotationInput> in;
    for (const auto& [id, p] : pos)
      in.push_back(ri(id, p));
    return in;
  };

  std::vector<int> seq = {rot.puller()};
  for (int i = 0; i < 2000; ++i) {
    const auto d = rot.tick(0.01, inputs());
    if (rot.puller() != seq.back())
      seq.push_back(rot.puller());
    // Teleport any fresh drifter just below the current floor -> attaches on
    // the next tick.
    for (const auto& x : d) {
      if (x.swing_side != 0.0) {
        floor_pos -= 2.0;
        pos[x.id] = floor_pos;
      }
    }
  }

  check(seq.size() >= 9, "cycle: many rotations completed");
  bool cyclic = true;
  for (size_t i = 1; i < seq.size(); ++i)
    if (seq[i] != seq[i - 1] % 4 + 1)
      cyclic = false;
  check(cyclic, "cycle: promotion follows line order 1-2-3-4-1-...");
}

// A member riding detached from the rider ahead for longer than detach_time
// is removed from the roster (and reported); a re-attach resets the timer.
static void test_detach_removal() {
  auto roster = rotators({1, 2, 3});
  roster.push_back({4, true});
  PacelineRotation rot(roster, RotationParams{}); // detach 8 m / 3 s

  auto in_gap = [&](double tail_pos, double sit_pos) {
    return std::vector<RotationInput>{ri(1, 100.0), ri(2, 98.25),
                                      ri(3, tail_pos), ri(4, sit_pos)};
  };

  // 2.0 s detached, brief re-attach, 2.0 s detached again: timer must have
  // reset — nobody removed.
  for (int i = 0; i < 200; ++i)
    rot.tick(0.01, in_gap(96.5, 80.0));
  rot.tick(0.01, in_gap(96.5, 94.75));
  for (int i = 0; i < 200; ++i)
    rot.tick(0.01, in_gap(96.5, 80.0));
  check(rot.member_count() == 4, "detach: re-attach resets the timer");

  // Held past detach_time: the sitting rider is removed and reported.
  bool reported = false;
  for (int i = 0; i < 150; ++i) {
    rot.tick(0.01, in_gap(96.5, 80.0));
    for (int id : rot.removed_last_tick())
      if (id == 4)
        reported = true;
  }
  check(reported, "detach: removal reported via removed_last_tick");
  check(!rot.is_member(4) && rot.member_count() == 3,
        "detach: sitting rider removed from the roster");

  // Same rule mid-line: tail rotator detached from the rider ahead.
  for (int i = 0; i < 350; ++i)
    rot.tick(0.01, {ri(1, 100.0), ri(2, 98.25), ri(3, 80.0)});
  check(!rot.is_member(3) && rot.member_count() == 2,
        "detach: inline rotator removed too");
}

// Members absent from the input (rider removed from the engine) are silently
// dropped from the roster.
static void test_missing_input_prune() {
  PacelineRotation rot(rotators({1, 2, 3}), RotationParams{});
  const auto d = rot.tick(0.01, engaged_line({1, 2}));
  check(rot.member_count() == 2 && !rot.is_member(3),
        "prune: member without input dropped");
  check(d.size() == 2 && dir_for(d, 3) == nullptr,
        "prune: no directive for the dropped member");
}

// --- Engine-level ---

static RiderConfig cfg(int id, double ftp = 250, double w_prime = 24000) {
  return RiderConfig{id,  "R" + std::to_string(id),
                     ftp, 6,
                     2,   0.05,
                     700, 3.5,
                     65,  0.3,
                     w_prime, Bike::create_road(),
                     Team("T")};
}

static double wheel_gap(const PhysicsEngine& e, int follower, int leader) {
  const Rider* f = e.get_rider_by_id(follower);
  const Rider* l = e.get_rider_by_id(leader);
  return (l->get_pos() - f->get_pos()) - l->get_bike_len();
}

// Build a settled n-rider chain (leader on constant effort) — the standard
// starting point before handing the line to a rotation.
static void settle_chain(PhysicsEngine& eng, int n, double effort,
                         int steps) {
  for (int id = 1; id <= n; ++id)
    eng.add_rider(cfg(id));
  eng.set_rider_effort(1, effort);
  for (int id = 2; id <= n; ++id)
    eng.set_follow_target(id, id - 1);
  for (int i = 0; i < steps; ++i)
    eng.update(0.01);
}

// The full choreography: 5 rotators + 1 SittingIn.  The front cycles through
// all rotators in line order, repeatedly; the sitting rider never pulls; the
// drifter swings off-axis and comes back onto it; nobody gets dropped; the
// line reforms with the sitting rider at the rear.
static void test_engine_rotation_cycles() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  PhysicsEngine eng(&course);
  settle_chain(eng, 6, 0.85, 20000); // 200 s

  for (int id = 2; id <= 6; ++id)
    check(std::fabs(wheel_gap(eng, id, id - 1) - 0.25) < 0.15,
          "engine cycle: chain settled (rider " + std::to_string(id) + ")");

  RotationParams rp;
  rp.pull_time = 25.0; // > drift time: single drifter, clean windows
  auto roster = rotators({1, 2, 3, 4, 5});
  roster.push_back({6, true});
  eng.set_paceline_rotation(roster, rp);
  const auto* rot = eng.get_paceline_rotation();

  std::vector<int> seq = {rot->puller()};
  double max_swing_off = 0.0; // drifter lat_target offset from the line
  double end_off = 1e9;       // ... and after it merged back
  int drifter = -1;
  for (int i = 0; i < 50000; ++i) { // 500 s
    eng.update(dt);
    if (rot->puller() != seq.back()) {
      drifter = seq.back(); // the rider that just swung off
      seq.push_back(rot->puller());
    }
    if (drifter >= 0 && rot->drifting_count() > 0) {
      const auto lt = eng.get_rider_by_id(drifter)->get_lat_target();
      if (lt.has_value()) {
        const double off =
            std::fabs(*lt - eng.get_rider_by_id(rot->puller())->get_lat_pos());
        max_swing_off = std::max(max_swing_off, off);
      }
    } else if (drifter >= 0 && rot->drifting_count() == 0) {
      const auto lt = eng.get_rider_by_id(drifter)->get_lat_target();
      if (lt.has_value())
        end_off =
            std::fabs(*lt - eng.get_rider_by_id(rot->puller())->get_lat_pos());
    }
  }

  std::cout << "  [engine cycle] pull sequence:";
  for (int id : seq)
    std::cout << " " << id;
  std::cout << "\n  [engine cycle] max swing offset " << max_swing_off
            << " m, offset after merge " << end_off << " m\n";

  check(seq.size() >= 11, "engine cycle: >= 10 rotations in 500 s");
  check(std::find(seq.begin(), seq.end(), 6) == seq.end(),
        "engine cycle: SittingIn never pulls");
  bool cyclic = true;
  for (size_t i = 1; i < seq.size(); ++i)
    if (seq[i] != seq[i - 1] % 5 + 1)
      cyclic = false;
  check(cyclic, "engine cycle: promotion follows line order");
  int pulls[6] = {0, 0, 0, 0, 0, 0};
  for (int id : seq)
    ++pulls[id - 1];
  check(*std::min_element(pulls, pulls + 5) >= 2,
        "engine cycle: every rotator pulled repeatedly");
  check(rot->member_count() == 6, "engine cycle: nobody dropped");

  const double radius = eng.get_rider_by_id(1)->get_radius();
  check(max_swing_off > 2.0 * radius,
        "engine cycle: drifter rides off-axis while merging");
  check(end_off < radius, "engine cycle: merged drifter back on the axis");

  // Wait for a clean single-file window, then check the reformed line:
  // puller first, sitting rider last, everyone on a wheel.
  int guard = 0;
  while (rot->drifting_count() > 0 && guard++ < 4000)
    eng.update(dt);
  check(rot->drifting_count() == 0, "engine cycle: clean window reached");

  std::vector<std::pair<double, int>> order; // (pos desc, id)
  for (int id = 1; id <= 6; ++id)
    order.push_back({eng.get_rider_by_id(id)->get_pos(), id});
  std::sort(order.rbegin(), order.rend());
  check(order.front().second == rot->puller(),
        "engine cycle: puller leads the reformed line");
  check(order.back().second == 6,
        "engine cycle: sitting rider holds the rear through every merge");
  bool tight = true;
  std::cout << "  [engine cycle] reformed line gaps:";
  for (size_t i = 1; i < order.size(); ++i) {
    const double g = wheel_gap(eng, order[i].second, order[i - 1].second);
    std::cout << " " << g;
    if (g < -2.0 || g > 3.0)
      tight = false;
  }
  std::cout << "\n";
  // The freshest merger may still overlap its wheel by up to a bike length
  // (attach is positional; the gap opens over the next few seconds), hence
  // the -2 m floor.
  check(tight, "engine cycle: line intact (all wheel gaps within [-2, 3])");
}

// Short pull_time: several drifters in flight at once, and the system still
// coheres — cyclic promotions, no one dropped.
static void test_engine_multi_drifter() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  PhysicsEngine eng(&course);
  settle_chain(eng, 6, 0.85, 20000);

  RotationParams rp;
  rp.pull_time = 5.0;
  eng.set_paceline_rotation(rotators({1, 2, 3, 4, 5, 6}), rp);
  const auto* rot = eng.get_paceline_rotation();

  std::vector<int> seq = {rot->puller()};
  int max_drifting = 0;
  for (int i = 0; i < 20000; ++i) { // 200 s
    eng.update(dt);
    max_drifting = std::max(max_drifting, rot->drifting_count());
    if (rot->puller() != seq.back())
      seq.push_back(rot->puller());
  }

  std::cout << "  [multi-drifter] max in flight " << max_drifting
            << ", promotions " << (seq.size() - 1) << "\n  [multi-drifter] pull sequence:";
  for (int id : seq)
    std::cout << " " << id;
  std::cout << "\n";
  check(max_drifting >= 2, "multi-drifter: >= 2 in flight at once");
  check(rot->member_count() == 6, "multi-drifter: nobody dropped");
  check(seq.size() >= 14, "multi-drifter: rotation kept turning");
  int pulls[6] = {0, 0, 0, 0, 0, 0};
  for (int id : seq)
    ++pulls[id - 1];
  check(*std::min_element(pulls, pulls + 6) >= 2,
        "multi-drifter: everyone keeps pulling");
  // Sustained deep flight (line drained to 2-3) occasionally swaps adjacent
  // drifters' merge order — physically induced and accepted; order is only
  // guaranteed for single-drifter flight (test_engine_rotation_cycles).
  // Coherence here means spatial: the paceline never disintegrates.
  std::vector<double> pos;
  for (int id = 1; id <= 6; ++id)
    pos.push_back(eng.get_rider_by_id(id)->get_pos());
  std::sort(pos.rbegin(), pos.rend());
  bool coherent = true;
  for (size_t i = 1; i < pos.size(); ++i)
    if (pos[i - 1] - pos[i] > 8.0)
      coherent = false;
  check(coherent, "multi-drifter: pack stays together (no gap > 8 m)");
}

// A member too weak for the pace detaches physically; the roster notices and
// removes it (follow target cleared -> it reverts to a plain rider).
static void test_engine_weak_member_removed() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  PhysicsEngine eng(&course);
  for (int id = 1; id <= 3; ++id)
    eng.add_rider(cfg(id));
  eng.add_rider(cfg(4, 80)); // sitting-in, far too weak
  eng.set_rider_effort(1, 0.5); // gentle start so 4 can latch
  for (int id = 2; id <= 4; ++id)
    eng.set_follow_target(id, id - 1);
  for (int i = 0; i < 8000; ++i)
    eng.update(dt);

  RotationParams rp;
  rp.pull_time = 1e9; // no rotations: isolate the removal rule
  auto roster = rotators({1, 2, 3});
  roster.push_back({4, true});
  eng.set_paceline_rotation(roster, rp);
  const auto* rot = eng.get_paceline_rotation();
  check(rot->member_count() == 4, "weak: roster complete at the start");

  eng.set_rider_effort(1, 0.95); // screws turn
  for (int i = 0; i < 50000; ++i) // 500 s
    eng.update(dt);

  std::cout << "  [weak] gap to line " << wheel_gap(eng, 4, 3) << " m\n";
  check(!rot->is_member(4), "weak: removed from the roster");
  check(rot->member_count() == 3, "weak: roster shrank to 3");
  check(wheel_gap(eng, 4, 3) > 8.0, "weak: physically dropped");
  check(!eng.get_rider_by_id(4)->get_lat_target().has_value(),
        "weak: follow target cleared on removal");
  check(rot->is_member(2) && rot->is_member(3), "weak: the rest stay");
}

// Two identical runs, swings and merges included, land on bit-identical
// state.
static void test_engine_determinism() {
  auto run = [](std::vector<double>& pos, std::vector<double>& lat) {
    Course course = Course::create_flat();
    PhysicsEngine eng(&course);
    settle_chain(eng, 5, 0.85, 10000);
    RotationParams rp;
    rp.pull_time = 6.0;
    auto roster = rotators({1, 2, 3, 4});
    roster.push_back({5, true});
    eng.set_paceline_rotation(roster, rp);
    for (int i = 0; i < 10000; ++i) // 100 s of rotating
      eng.update(0.01);
    for (int id = 1; id <= 5; ++id) {
      pos.push_back(eng.get_rider_by_id(id)->get_pos());
      lat.push_back(eng.get_rider_by_id(id)->get_lat_pos());
    }
  };

  std::vector<double> pos_a, lat_a, pos_b, lat_b;
  run(pos_a, lat_a);
  run(pos_b, lat_b);
  check(pos_a == pos_b, "determinism: longitudinal positions identical");
  check(lat_a == lat_b, "determinism: lateral positions identical");
}

// --- Simulation-level: command-queue wrappers ---

static void test_simulation_api() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2), cfg(3), cfg(4)});
  sim.set_rider_effort(1, 0.5);

  auto roster = rotators({1, 2, 3});
  roster.push_back({4, true});
  sim.set_paceline_rotation(roster, RotationParams{});
  sim.step_fixed(dt);
  const auto* rot = sim.get_engine()->get_paceline_rotation();
  check(rot != nullptr && rot->member_count() == 4,
        "sim api: rotation installed via the command queue");
  check(sim.get_effort_source(2) == EffortSource::Follow,
        "sim api: directives assign follow targets");
  check(sim.get_effort_source(1) == EffortSource::Manual,
        "sim api: the puller's own effort source stays live");

  sim.clear_paceline_rotation();
  sim.step_fixed(dt);
  check(sim.get_engine()->get_paceline_rotation() == nullptr,
        "sim api: clear via the command queue");

  sim.set_paceline_rotation(roster, RotationParams{});
  sim.step_fixed(dt);
  sim.reset();
  check(sim.get_engine()->get_paceline_rotation() == nullptr,
        "sim api: reset clears the rotation");
}

int main() {
  test_roster_directives();
  test_swing_and_promotion();
  test_crosswind_side();
  test_trigger_guards();
  test_attach();
  test_pull_cycle_order();
  test_detach_removal();
  test_missing_input_prune();
  test_engine_rotation_cycles();
  test_engine_multi_drifter();
  test_engine_weak_member_removed();
  test_engine_determinism();
  test_simulation_api();

  if (checks_failed) {
    std::cout << checks_failed << " check(s) FAILED\n";
    return 1;
  }
  std::cout << "All rotation tests passed\n";
  return 0;
}
