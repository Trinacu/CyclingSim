// Tests for the C1 decision-layer pieces (decision.h): draft-factor
// helpers, the W′-budget pace estimator (properties + an end-to-end budget
// check against the physics), and DecisionContext construction.

#include "decision.h"

#include "course.h"
#include "rider.h"
#include "sim.h"

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

static bool near(double a, double b, double eps) {
  return std::fabs(a - b) < eps;
}

static RiderConfig cfg(int id, double ftp = 250, double w_prime = 24000) {
  return RiderConfig{id,  "R" + std::to_string(id),
                     ftp, 6,
                     2,   0.05,
                     700, 3.5,
                     65,  0.3,
                     w_prime, Bike::create_road(),
                     kNoTeam};
}

// --- Draft-factor helpers ---

static void test_draft_helpers() {
  const DraftingParams p; // {0.98, 0.61, 0.50, 0.44, 0.42, 0.41}

  check(rotation_avg_draft_factor(1, p) == 1.0, "draft: solo -> 1.0");
  check(near(rotation_avg_draft_factor(2, p), (0.98 + 0.61) / 2.0, 1e-12),
        "draft: pair averages front + slot 1");
  const double eight = (0.98 + 0.61 + 0.50 + 0.44 + 0.42 + 0.41 * 3) / 8.0;
  check(near(rotation_avg_draft_factor(8, p), eight, 1e-12),
        "draft: n > 6 averages in extra last-slot entries");
  check(rotation_avg_draft_factor(8, p) < rotation_avg_draft_factor(3, p),
        "draft: bigger rotation -> better average");

  check(near(line_slot_draft_factor(0, p), 0.98, 1e-12),
        "draft: line front slot");
  check(near(line_slot_draft_factor(9, p), 0.41, 1e-12),
        "draft: deep slot clamps to the last entry");
  check(line_slot_draft_factor(-1, p) == 1.0, "draft: no slot -> 1.0");
}

// --- Estimator ---

// Fixed-point properties on an engine-initialised rider (env populated).
static void test_estimator_properties() {
  Course course = Course::from_segments({{2500, 0.06, 0, 0, 8},
                                         {500, 0.0, 0, 0, 8}});
  PhysicsEngine eng(&course);
  eng.add_rider(cfg(1));
  eng.update(0.01); // populate env
  const Rider& r = *eng.get_rider_by_id(1);
  const double ftp = r.get_ftp();
  const double wbal = r.get_energy();

  const auto est = estimate_wprime_pace(r, 2500.0, 0.06, 0.0, wbal, 1.0);
  std::cout << "  [est] climb pace " << est.power << " W (" << est.power / ftp
            << " x ftp), v " << est.speed << " m/s, T " << est.duration
            << " s\n";
  check(est.power > ftp && est.power < r.get_config().max_effort * ftp,
        "est: climb pace above ftp, below max");
  // Self-consistency: (P - ftp) * T == wbal when unclamped.
  check(near((est.power - ftp) * est.duration, wbal, 1.0),
        "est: budget identity (P - ftp) * T = wbal");

  check(estimate_wprime_pace(r, 1e6, 0.06, 0.0, wbal, 1.0).power <
            ftp * 1.02,
        "est: huge distance -> pace ~ ftp");
  check(near(estimate_wprime_pace(r, 10.0, 0.06, 0.0, wbal, 1.0).power,
             r.get_config().max_effort * ftp, 1e-6),
        "est: tiny distance clamps at max power");
  check(estimate_wprime_pace(r, 2500.0, 0.06, 0.0, wbal, 1.0).power >
            estimate_wprime_pace(r, 2500.0, 0.06, 0.0, wbal / 2, 1.0).power,
        "est: more W' -> harder pace");
  check(estimate_wprime_pace(r, 2500.0, 0.06, 0.0, wbal, 0.6).power >=
            est.power,
        "est: draft -> shorter climb -> at least as hard a pace");
  check(estimate_wprime_pace(r, 2500.0, 0.06, 0.0, wbal, 0.6).duration <
            est.duration,
        "est: draft shortens the climb");
}

// End-to-end budget check: ride the estimated pace up the climb; W′ should
// land near its floor at the crest.  Tolerance is wide on purpose — the
// standing start stretches T beyond the estimate, and the energy model's
// effort limit throttles the last stretch once W′bal drops under 20% (the
// rider physically can't hold the pace to exactly zero).
static void test_estimator_budget_on_climb() {
  const double dt = 0.01;
  Course course = Course::from_segments({{2500, 0.06, 0, 0, 8},
                                         {500, 0.0, 0, 0, 8}});
  PhysicsEngine eng(&course);
  eng.add_rider(cfg(1, 250, 20000));
  eng.update(dt);
  const Rider& r = *eng.get_rider_by_id(1);

  const auto est =
      estimate_wprime_pace(r, 2500.0, 0.06, 0.0, r.get_energy(), 1.0);
  eng.set_rider_effort(1, est.power / r.get_ftp());

  int steps = 0;
  while (r.get_pos() < 2500.0 && steps++ < 100000)
    eng.update(dt);

  const double frac = r.get_energy_fraction();
  std::cout << "  [est] crest: wbal_frac " << frac << ", time " << steps * dt
            << " s (estimated " << est.duration << " s)\n";
  check(steps < 100000, "budget: crest reached");
  check(frac < 0.20, "budget: W' spent to (near) the floor at the crest");
  check(near(steps * dt, est.duration, 0.15 * est.duration),
        "budget: duration within 15% of the estimate");
}

// --- DecisionContext ---

static void test_build_context() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2), cfg(3), cfg(4)});
  sim.set_rider_effort(1, 0.9);
  sim.set_rider_effort(3, 0.5);
  sim.set_rider_effort(4, 0.5);
  // Riders 1+2 as a two-man line (no swings below 3): 2 gets a follow
  // target from the rotation, 1 pulls on its manual effort.
  sim.set_paceline_rotation({{1, false}, {2, false}}, RotationParams{});

  for (int i = 0; i < 12000; ++i) // 120 s: 1+2 ride away from 3+4
    sim.step_fixed(dt);

  const DecisionSystem& ds = sim.get_decision();
  check(sim.get_engine()->get_group_tracker().get_snapshot().size() == 2,
        "ctx: two groups (test premise)");

  const DecisionContext c1 = ds.build_context(sim, 1);
  check(c1.id == 1 && c1.team == kNoTeam, "ctx: identity");
  check(c1.pos > 0.0 && c1.speed > 5.0 && c1.ftp > 0.0 &&
            c1.wbal_frac > 0.0 && c1.wbal_frac <= 1.0,
        "ctx: own physical state populated");
  check(c1.effort_source == EffortSource::Manual,
        "ctx: puller's effort source is Manual");
  check(c1.in_rotation && c1.line_depth == 0 && c1.rotation_size == 2 &&
            !c1.sitting_in,
        "ctx: rotation membership (puller)");
  check(c1.group.group_ordinal == 0 && c1.group.group_size == 2,
        "ctx: front group topology");
  check(c1.nearby.size() == 1 && c1.nearby[0].id == 2 &&
            c1.nearby[0].lon_offset < 0.0,
        "ctx: rider window sees only the follower (behind)");
  check(c1.nearby.empty() ||
            (c1.nearby[0].group_ordinal == 0 && c1.nearby[0].group_size == 2),
        "ctx: perceived rider carries group ordinal/size");
  check(c1.time_gap_to_group_ahead == -1.0,
        "ctx: leading group has no gap ahead");
  check(c1.time_gap_to_group_behind > 5.0,
        "ctx: gap to the chasing group measured");

  const DecisionContext c2 = ds.build_context(sim, 2);
  check(c2.effort_source == EffortSource::Follow &&
            c2.in_rotation && c2.line_depth == 1,
        "ctx: follower's source is Follow, line depth 1");

  const DecisionContext c3 = ds.build_context(sim, 3);
  check(c3.group.group_ordinal == 1, "ctx: chase group ordinal");
  check(c3.time_gap_to_group_ahead > 5.0, "ctx: chase sees the gap ahead");
  check(c3.time_gap_to_group_behind == -1.0, "ctx: last group, none behind");
  check(!c3.in_rotation && c3.line_depth == -1,
        "ctx: non-member rotation fields");
  check(c3.nearby.size() == 1 && c3.nearby[0].id == 4,
        "ctx: 200 m window excludes the front group");
  check(!c3.directive.has_value(), "ctx: directive inbox empty until C4");
  check(c3.intel != nullptr && c3.clock != nullptr &&
            near(c3.now, sim.get_sim_seconds(), 1e-9),
        "ctx: world handles + clock time");
  check(c3.intel->distance_to_finish(c3.pos) > 0.0 &&
            c3.clock->crossing_time(1, c3.pos).has_value(),
        "ctx: handles answer queries");

  const DecisionContext none = ds.build_context(sim, 99);
  check(none.id == -1, "ctx: unknown rider -> empty context");
}

int main() {
  test_draft_helpers();
  test_estimator_properties();
  test_estimator_budget_on_climb();
  test_build_context();

  if (checks_failed) {
    std::cout << checks_failed << " check(s) FAILED\n";
    return 1;
  }
  std::cout << "All decision tests passed\n";
  return 0;
}
