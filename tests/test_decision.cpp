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

// --- C2: policies, cadence, arbitration, reconcile ---

// Records its calls; emits a constant effort plus optional role / follow /
// one-shot promote.
struct ProbePolicy : IRiderPolicy {
  double effort;
  GroupRole role = GroupRole::Unassigned;
  std::optional<RiderId> follow_target;
  bool promote_once = false;
  int calls = 0;
  std::vector<double> call_times;

  explicit ProbePolicy(double e) : effort(e) {}
  PolicyOutput decide(const DecisionContext& ctx) override {
    ++calls;
    call_times.push_back(ctx.now);
    PolicyOutput out;
    out.target_effort = effort;
    out.role_decl = role;
    out.follow = follow_target;
    if (promote_once) {
      out.maneuver = Maneuver{};
      promote_once = false;
    }
    return out;
  }
  const char* name() const override { return "probe"; }
};

static void test_decide_cadence() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1)});
  auto p = std::make_shared<ProbePolicy>(0.7);
  sim.set_rider_policy(1, p);

  for (int i = 0; i < 1000; ++i) // 10 s
    sim.step_fixed(dt);

  check(p->calls == 10, "cadence: 10 decide ticks in 10 s at 1 Hz");
  check(p->call_times.size() >= 2 &&
            near(p->call_times[1] - p->call_times[0], 1.0, 1e-9),
        "cadence: ticks 1 s apart");
  check(near(sim.get_engine()->get_rider_by_id(1)->get_target_effort(), 0.7,
             1e-12),
        "cadence: policy effort held between ticks");
  check(sim.get_effort_source(1) == EffortSource::Policy,
        "cadence: source is Policy");

  // Slider inert for a policy rider.
  sim.set_rider_effort(1, 0.3);
  sim.step_fixed(dt);
  check(near(sim.get_engine()->get_rider_by_id(1)->get_target_effort(), 0.7,
             1e-12),
        "cadence: slider is inert under a policy");

  // Snapshot carries the mode + policy name.
  FrameSnapshot prev, curr;
  sim.consume_latest_frame_pair(prev, curr);
  check(curr.riders.at(1).effort_source == EffortSource::Policy &&
            curr.riders.at(1).policy == "probe",
        "cadence: snapshot stamped with source + policy name");
}

static void test_arbitration() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2)});
  sim.set_rider_effort(2, 0.8);

  // Policy -> Schedule replaces it -> Policy replaces the schedule.
  sim.set_rider_policy(1, std::make_shared<ProbePolicy>(0.7));
  sim.step_fixed(dt);
  check(sim.get_effort_source(1) == EffortSource::Policy, "arb: policy set");
  sim.set_effort_schedule(
      1, std::make_shared<StepEffortSchedule>(
             std::vector<EffortBlock>{{1e9, 0.5}}));
  sim.step_fixed(dt);
  check(sim.get_effort_source(1) == EffortSource::Schedule,
        "arb: schedule replaces the policy");
  sim.set_rider_policy(1, std::make_shared<ProbePolicy>(0.7));
  sim.step_fixed(dt);
  check(sim.get_effort_source(1) == EffortSource::Policy,
        "arb: policy replaces the schedule");

  // A follow target outranks the policy; the policy didn't install it, so
  // emitting follow = nullopt must NOT clear it.
  sim.set_follow_target(1, 2);
  for (int i = 0; i < 200; ++i) // two decide ticks
    sim.step_fixed(dt);
  check(sim.get_effort_source(1) == EffortSource::Follow,
        "arb: follow outranks policy");
  check(sim.get_engine()->has_follow_target(1),
        "arb: manual follow target survives the policy's nullopt");

  sim.clear_follow_target(1);
  sim.clear_rider_policy(1);
  sim.step_fixed(dt);
  check(sim.get_effort_source(1) == EffortSource::Manual,
        "arb: cleared back to Manual");
}

static void test_policy_follow_ownership() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2)});
  sim.set_rider_effort(1, 0.8);

  auto p = std::make_shared<ProbePolicy>(0.7);
  p->follow_target = 1;
  sim.set_rider_policy(2, p);
  for (int i = 0; i < 150; ++i)
    sim.step_fixed(dt);
  check(sim.get_engine()->has_follow_target(2) &&
            sim.get_effort_source(2) == EffortSource::Follow,
        "policy follow: installed, source Follow");

  p->follow_target = std::nullopt;
  for (int i = 0; i < 100; ++i)
    sim.step_fixed(dt);
  check(!sim.get_engine()->has_follow_target(2) &&
            sim.get_effort_source(2) == EffortSource::Policy,
        "policy follow: policy-installed target cleared on nullopt");
}

static void test_policy_promote_maneuver() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2), cfg(3), cfg(4)});
  sim.set_rider_effort(1, 0.5);
  std::vector<RotationMember> roster = {
      {1, false}, {2, false}, {3, false}, {4, true}};
  sim.set_paceline_rotation(roster, RotationParams{});
  sim.step_fixed(dt);
  const auto* rot = sim.get_engine()->get_paceline_rotation();
  check(rot->inline_count() == 3, "maneuver: sitter starts outside the line");

  auto p = std::make_shared<ProbePolicy>(0.7);
  p->promote_once = true;
  sim.set_rider_policy(4, p);
  for (int i = 0; i < 150; ++i) // past one decide tick
    sim.step_fixed(dt);
  check(rot->inline_count() == 4,
        "maneuver: policy promote -> first-sitter fast path into the line");
}

static void test_reconcile() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2), cfg(3), cfg(4)});
  for (int id = 1; id <= 4; ++id)
    sim.set_rider_effort(id, 0.7);
  // Declared intent, appstate-style (no policies involved): reconcile is
  // driven by roles however they got set.
  for (const auto& [id, r] : sim.get_engine()->get_riders())
    r->set_group_role(GroupRole::Paceline);

  for (int i = 0; i < 150; ++i) // past the first decide tick
    sim.step_fixed(dt);

  const PhysicsEngine& eng = *sim.get_engine();
  check(eng.auto_rotation_count() == 1, "reconcile: one rotation formed");
  const PacelineRotation* rot = eng.get_rotation_for(1);
  check(rot != nullptr && rot->member_count() == 4,
        "reconcile: all four declarers in it");
  check(eng.get_paceline_rotation() == nullptr,
        "reconcile: no manual rotation involved");

  // Un-declare one: removed on the next tick, rotation survives.
  sim.get_engine()->get_riders().at(3)->set_group_role(GroupRole::Unassigned);
  for (int i = 0; i < 120; ++i)
    sim.step_fixed(dt);
  check(eng.get_rotation_for(3) == nullptr && rot->member_count() == 3,
        "reconcile: ex-declarer removed");
  check(!eng.has_follow_target(3),
        "reconcile: removed rider's follow target cleared");

  // Un-declare all but one: rotation dissolves.
  sim.get_engine()->get_riders().at(2)->set_group_role(GroupRole::Unassigned);
  sim.get_engine()->get_riders().at(4)->set_group_role(GroupRole::Unassigned);
  for (int i = 0; i < 120; ++i)
    sim.step_fixed(dt);
  check(eng.auto_rotation_count() == 0,
        "reconcile: below two declarers the rotation dissolves");
}

static void test_reconcile_per_group_and_manual_wins() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2), cfg(3), cfg(4), cfg(5), cfg(6)});
  // 1+2 manual rotation, riding away; 3+4 fast pair; 5+6 slow pair.
  sim.set_rider_effort(1, 0.95);
  sim.set_paceline_rotation({{1, false}, {2, false}}, RotationParams{});
  sim.set_rider_effort(3, 0.8);
  sim.set_rider_effort(4, 0.8);
  sim.set_rider_effort(5, 0.5);
  sim.set_rider_effort(6, 0.5);

  // Split FIRST, declare after: declaring while everyone is still one bunch
  // would form a single rotation whose follow controllers glue the pack
  // together (the slow pair can hold any wheel, they're just unwilling).
  for (int i = 0; i < 15000; ++i) // 150 s: three separated groups
    sim.step_fixed(dt);
  for (const auto& [id, r] : sim.get_engine()->get_riders())
    r->set_group_role(GroupRole::Paceline);
  for (int i = 0; i < 200; ++i) // past a decide tick
    sim.step_fixed(dt);

  const PhysicsEngine& eng = *sim.get_engine();
  check(eng.get_group_tracker().get_snapshot().size() == 3,
        "reconcile groups: three groups (premise)");
  check(eng.auto_rotation_count() == 2,
        "reconcile groups: one auto rotation per non-manual group");
  check(eng.get_rotation_for(1) == eng.get_paceline_rotation(),
        "reconcile groups: manual roster wins for its riders");
  check(eng.get_rotation_for(3) != nullptr &&
            eng.get_rotation_for(3) == eng.get_rotation_for(4) &&
            eng.get_rotation_for(3) != eng.get_rotation_for(5),
        "reconcile groups: pairs rotate within their own groups");
}

static void test_decide_determinism() {
  auto run = [](std::vector<double>& pos) {
    const double dt = 0.01;
    Course course = Course::create_flat();
    Simulation sim(&course);
    sim.add_riders({cfg(1), cfg(2), cfg(3)});
    for (int id = 1; id <= 3; ++id) {
      // Dynamic, state-dependent policy: effort tracks W' and declares
      // paceline intent, so decisions couple riders via the reconcile.
      struct DynPolicy : IRiderPolicy {
        PolicyOutput decide(const DecisionContext& ctx) override {
          PolicyOutput out;
          out.target_effort = 0.6 + 0.4 * ctx.wbal_frac;
          out.role_decl = GroupRole::Paceline;
          return out;
        }
        const char* name() const override { return "dyn"; }
      };
      sim.set_rider_policy(id, std::make_shared<DynPolicy>());
    }
    for (int i = 0; i < 10000; ++i) // 100 s
      sim.step_fixed(dt);
    for (int id = 1; id <= 3; ++id)
      pos.push_back(sim.get_engine()->get_rider_by_id(id)->get_pos());
  };

  std::vector<double> a, b;
  run(a);
  run(b);
  check(a == b, "determinism: identical runs -> bit-identical positions");
}

// --- C3: WPrimePacingPolicy ---

// One policy rider on `course`, stepped until pos >= until_pos (or cap).
// Returns false on cap.  The sim must consume the course by reference, so
// the caller owns both.
static bool step_policy_rider_to(Simulation& sim, RiderId id, double until_pos,
                                 int max_steps = 500000) {
  const Rider* r = sim.get_engine()->get_rider_by_id(id);
  int steps = 0;
  while (r->get_pos() < until_pos && steps++ < max_steps)
    sim.step_fixed(0.01);
  return steps < max_steps;
}

// Budget honored end-to-end, policy-driven, on a steep-short and a
// long-shallow climb: both must crest with wbal_frac ≈ the reserve floor.
// Tolerance is one-sided-wide above: the energy model throttles effort once
// wbal drops under 20%, so the rider physically can't ride the last stretch
// down to exactly the floor.
static void test_pacing_budget_on_climbs() {
  struct Case {
    const char* label;
    std::vector<std::array<double, 5>> segs;
    double crest;
  };
  const Case cases[] = {
      {"steep-short",
       {{500, 0, 0, 0, 8}, {1500, 0.10, 0, 0, 8}, {1000, 0, 0, 0, 8}},
       2000.0},
      {"long-shallow",
       {{500, 0, 0, 0, 8}, {4000, 0.03, 0, 0, 8}, {1000, 0, 0, 0, 8}},
       4500.0},
  };
  const WPrimePacingParams params{}; // floor 0.15

  double mid_climb_effort[2] = {0.0, 0.0};
  int i = 0;
  for (const Case& c : cases) {
    Course course = Course::from_segments(c.segs);
    Simulation sim(&course);
    sim.add_riders({cfg(1, 250, 20000)});
    sim.set_rider_policy(1, std::make_shared<WPrimePacingPolicy>(params));
    const Rider* r = sim.get_engine()->get_rider_by_id(1);

    check(step_policy_rider_to(sim, 1, (500.0 + c.crest) / 2.0),
          std::string(c.label) + ": reaches mid-climb");
    mid_climb_effort[i++] = r->get_target_effort();
    check(r->get_target_effort() > 1.0,
          std::string(c.label) + ": climb pace above ftp");

    check(step_policy_rider_to(sim, 1, c.crest),
          std::string(c.label) + ": reaches the crest");
    const double frac = r->get_energy_fraction();
    std::cout << "  [pace] " << c.label << ": crest wbal_frac " << frac
              << " (floor " << params.wbal_floor_frac << ")\n";
    check(frac > params.wbal_floor_frac - 0.05 && frac < 0.30,
          std::string(c.label) + ": crests near the reserve floor");
  }
  check(mid_climb_effort[0] > mid_climb_effort[1],
        "pace: steep-short demands a harder pace than long-shallow");
}

// Horizon handoff: pacing toward the crest, then — once past it — toward
// the finish, where the spent budget drops the pace back to ~ftp.
static void test_pacing_horizon_handoff() {
  Course course = Course::from_segments(
      {{500, 0, 0, 0, 8}, {1500, 0.08, 0, 0, 8}, {3000, 0, 0, 0, 8}});
  Simulation sim(&course);
  sim.add_riders({cfg(1, 250, 20000)});
  sim.set_rider_policy(1, std::make_shared<WPrimePacingPolicy>());
  const Rider* r = sim.get_engine()->get_rider_by_id(1);

  check(step_policy_rider_to(sim, 1, 1900.0), "handoff: reaches 1900 m");
  const double effort_before = r->get_target_effort();
  check(effort_before > 1.05, "handoff: spending toward the crest");

  check(step_policy_rider_to(sim, 1, 2100.0), "handoff: past the crest");
  for (int i = 0; i < 150; ++i) // guarantee a decide tick after crossing
    sim.step_fixed(0.01);
  const double effort_after = r->get_target_effort();
  std::cout << "  [pace] handoff: effort " << effort_before << " -> "
            << effort_after << "\n";
  check(effort_after < effort_before - 0.05,
        "handoff: pace drops once the horizon flips to the finish");
  check(effort_after < 1.05, "handoff: post-crest pace back near ftp");
}

// Descents are recovery: sub-FTP effort, W′ recharging.
static void test_pacing_recovery_on_descent() {
  Course course = Course::from_segments(
      {{500, 0, 0, 0, 8}, {2000, -0.05, 0, 0, 8}, {2000, 0, 0, 0, 8}});
  Simulation sim(&course);
  sim.add_riders({cfg(1, 250, 20000)});
  const WPrimePacingParams params{};
  sim.set_rider_policy(1, std::make_shared<WPrimePacingPolicy>(params));
  const Rider* r = sim.get_engine()->get_rider_by_id(1);

  check(step_policy_rider_to(sim, 1, 700.0), "recovery: onto the descent");
  for (int i = 0; i < 150; ++i) // a decide tick on the descent
    sim.step_fixed(0.01);
  check(near(r->get_target_effort(), params.recovery_effort, 1e-9),
        "recovery: descent effort is the recovery param");
  const double frac_on_descent = r->get_energy_fraction();

  // Sample the recharge on the descent proper: the lookahead window flips
  // back to spending ~100 m before the descent ends.
  check(step_policy_rider_to(sim, 1, 2300.0), "recovery: descent ridden");
  check(r->get_energy_fraction() > frac_on_descent,
        "recovery: W' recharged on the way down");
  check(step_policy_rider_to(sim, 1, 2600.0), "recovery: onto the flat");
  for (int i = 0; i < 150; ++i) // a decide tick on the flat
    sim.step_fixed(0.01);
  check(r->get_target_effort() > params.recovery_effort + 0.1,
        "recovery: spending resumes on the flat run-out");
}

// The C gate (PLAN § C3): create_endulating, one policy rider vs one
// constant-schedule rider at the policy run's average power.  The policy
// rider must crest (the last crest IS the finish) near the reserve floor
// and arrive no later.
static void test_gate_policy_vs_schedule() {
  const double dt = 0.01;
  double policy_time = 0.0, policy_frac = 0.0, avg_effort = 0.0;
  {
    Course course = Course::create_endulating();
    const double total = course.get_total_length();
    Simulation sim(&course);
    sim.add_riders({cfg(1, 250, 20000)});
    // The gate races to the line: minimal reserve.  The 0.15 default is a
    // road-race reserve (matches kept burnable), and the equal-average-power
    // uniform opponent spends every joule — a held-back 15% is ~10 s of
    // climbing time given away, not pacing quality.
    WPrimePacingParams gate_params;
    gate_params.wbal_floor_frac = 0.05;
    sim.set_rider_policy(1, std::make_shared<WPrimePacingPolicy>(gate_params));
    const Rider* r = sim.get_engine()->get_rider_by_id(1);

    double power_sum = 0.0;
    int steps = 0;
    while (r->get_pos() < total && steps < 800000) {
      sim.step_fixed(dt);
      power_sum += r->get_power();
      ++steps;
    }
    check(steps < 800000, "gate: policy rider finishes");
    policy_time = sim.get_decision()
                      .race_clock()
                      .crossing_time(1, total)
                      .value_or(steps * dt);
    policy_frac = r->get_energy_fraction();
    avg_effort = (power_sum / steps) / r->get_ftp();
  }

  double schedule_time = 0.0;
  {
    Course course = Course::create_endulating();
    const double total = course.get_total_length();
    Simulation sim(&course);
    sim.add_riders({cfg(1, 250, 20000)});
    sim.set_effort_schedule(1, std::make_shared<StepEffortSchedule>(
                                   std::vector<EffortBlock>{{1e9, avg_effort}}));
    const Rider* r = sim.get_engine()->get_rider_by_id(1);
    int steps = 0;
    while (r->get_pos() < total && steps < 800000) {
      sim.step_fixed(dt);
      ++steps;
    }
    check(steps < 800000, "gate: schedule rider finishes");
    schedule_time = sim.get_decision()
                        .race_clock()
                        .crossing_time(1, total)
                        .value_or(steps * dt);
  }

  std::cout << "  [gate] policy " << policy_time << " s vs schedule "
            << schedule_time << " s at equal avg effort " << avg_effort
            << "; policy finish wbal_frac " << policy_frac << "\n";
  check(policy_frac > 0.02 && policy_frac < 0.25,
        "gate: policy rider finishes near the reserve floor");
  check(policy_time <= schedule_time + 1.0,
        "gate: policy rider arrives no later than equal-power schedule");
}

// --- C4: director, directives, tactics, MoveUp join ---

static RiderConfig cfgt(int id, TeamId team, double ftp = 250) {
  RiderConfig c = cfg(id, ftp);
  c.team_id = team;
  return c;
}

// Director rules as pure functions: hand-built contexts, no engine.
static void test_director_rules() {
  RacePlan plan;
  plan.leader = 1;
  plan.assignments = {{2, Directive::Type::ProtectLeader},
                      {3, Directive::Type::SitIn}};
  plan.chase_gap_max = 30.0;
  const TeamDirector director(plan);

  auto ctx = [](RiderId id, double gap_ahead) {
    DecisionContext c;
    c.id = id;
    c.time_gap_to_group_ahead = gap_ahead;
    return c;
  };

  std::unordered_map<RiderId, Directive> out;
  director.direct({ctx(1, 20.0), ctx(2, 20.0), ctx(3, 20.0), ctx(4, 20.0)},
                  out);
  check(out.at(2).type == Directive::Type::ProtectLeader &&
            out.at(2).target == 1,
        "director: standing order + leader target filled");
  check(out.at(3).type == Directive::Type::SitIn,
        "director: sit-in assignment holds (chase never repurposes it)");
  check(out.at(4).type == Directive::Type::Chase,
        "director: free rider within chase_gap_max is sent chasing");
  check(out.at(1).type == Directive::Type::Free,
        "director: the leader is never sent chasing");

  out.clear();
  director.direct({ctx(4, 45.0)}, out);
  check(out.at(4).type == Directive::Type::Free,
        "director: gap beyond chase_gap_max -> no chase");
  out.clear();
  director.direct({ctx(4, -1.0)}, out);
  check(out.at(4).type == Directive::Type::Free,
        "director: unknown gap -> no chase");
}

// Tactics as policy-level units: real context from the engine, directive
// injected, policy called directly.
static void test_tactics_orders_and_clamp() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2)});
  sim.set_rider_effort(1, 0.7);
  for (int i = 0; i < 200; ++i)
    sim.step_fixed(dt);

  WPrimePacingPolicy policy;
  DecisionContext ctx = sim.get_decision().build_context(sim, 1);
  const PolicyOutput free_out = policy.decide(ctx);
  check(free_out.target_effort.has_value() &&
            free_out.role_decl == GroupRole::Unassigned && !free_out.follow,
        "tactics: Free baseline (premise)");
  const double baseline = *free_out.target_effort;

  // Chase: bounded delta over the baseline + Paceline declaration.
  ctx.directive = Directive{Directive::Type::Chase, -1};
  const PolicyOutput chase_out = policy.decide(ctx);
  check(chase_out.role_decl == GroupRole::Paceline,
        "tactics: Chase declares Paceline");
  check(chase_out.target_effort.has_value() &&
            near(*chase_out.target_effort,
                 std::min(baseline + 0.15, ctx.effort_limit), 1e-9),
        "tactics: Chase = baseline + chase_delta, feasibility-clamped");

  // The rider-side clamp: a cooked rider ignores the order.
  ctx.wbal_frac = 0.10; // under chase_reserve_frac
  const PolicyOutput cooked = policy.decide(ctx);
  check(cooked.role_decl == GroupRole::Unassigned &&
            near(*cooked.target_effort, baseline, 1e-9),
        "tactics: cooked rider ignores Chase (the clamp is final)");
  ctx.wbal_frac = 1.0;

  // SitIn: stop working — no paceline declaration, baseline pace.
  ctx.directive = Directive{Directive::Type::SitIn, -1};
  const PolicyOutput sit = policy.decide(ctx);
  check(sit.role_decl == GroupRole::Unassigned &&
            near(*sit.target_effort, baseline, 1e-9),
        "tactics: SitIn -> out of the rotation, baseline pace");

  // Pull: declare in; as a sitter, resolve to the promotion maneuver.
  ctx.directive = Directive{Directive::Type::Pull, -1};
  const PolicyOutput pull = policy.decide(ctx);
  check(pull.role_decl == GroupRole::Paceline && !pull.maneuver,
        "tactics: Pull as non-member -> declare Paceline");
  ctx.sitting_in = true;
  const PolicyOutput pull_sit = policy.decide(ctx);
  check(pull_sit.maneuver.has_value() &&
            pull_sit.maneuver->type == Maneuver::Type::PromoteSitter,
        "tactics: Pull as sitter -> promotion maneuver");
  ctx.sitting_in = false;

  // ProtectLeader: protect install (relation Ahead), leave any rotation.
  ctx.directive = Directive{Directive::Type::ProtectLeader, 2};
  const PolicyOutput prot = policy.decide(ctx);
  check(prot.follow && *prot.follow == 2 &&
            prot.follow_relation == FollowRelation::Ahead &&
            prot.role_decl == GroupRole::Unassigned,
        "tactics: ProtectLeader -> protect the ward, out of rotations");
  ctx.directive = Directive{Directive::Type::ProtectLeader, 1};
  check(!policy.decide(ctx).follow,
        "tactics: refusing to protect oneself (rides Free)");
}

// Directive flow end-to-end: a race-plan Pull order to a sitting policy
// rider promotes it into the line and it reaches the front within a few
// (shortened) pull cycles.
static void test_directive_pull_flow() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  const TeamId alpha = sim.get_engine()->add_team("Alpha");
  sim.add_riders({cfgt(1, alpha), cfgt(2, alpha), cfgt(3, alpha),
                  cfgt(4, alpha)});
  sim.set_rider_effort(1, 0.7);
  RotationParams rp;
  rp.pull_time = 3.0; // fast cycles so "ends up puller" fits a short run
  sim.set_paceline_rotation({{1, false}, {2, false}, {3, false}, {4, true}},
                            rp);
  sim.set_rider_policy(4, std::make_shared<WPrimePacingPolicy>());
  RacePlan plan;
  plan.assignments = {{4, Directive::Type::Pull}};
  sim.set_race_plan(alpha, plan);
  sim.step_fixed(dt); // drain the queued setup commands

  const auto* rot = sim.get_engine()->get_paceline_rotation();
  int puller_step = -1;
  int inline_step = -1;
  for (int i = 0; i < 9000; ++i) { // 90 s budget
    sim.step_fixed(dt);
    if (inline_step < 0 && rot->line_depth(4) >= 0)
      inline_step = i;
    if (rot->puller() == 4) {
      puller_step = i;
      break;
    }
  }
  check(sim.get_decision().last_directive(4).has_value() &&
            sim.get_decision().last_directive(4)->type ==
                Directive::Type::Pull,
        "pull flow: directive delivered");
  check(inline_step >= 0 && inline_step < 300,
        "pull flow: sitter promoted into the line within ~2 decide ticks");
  std::cout << "  [pull flow] in line at " << (inline_step * dt)
            << " s, puller at " << (puller_step * dt) << " s\n";
  check(puller_step >= 0, "pull flow: ordered rider reaches the front");

  // Radio = team membership: the manual riders got directives too (Free),
  // surfaced but not obeyed.
  check(sim.get_decision().last_directive(1).has_value() &&
            sim.get_decision().last_directive(1)->type ==
                Directive::Type::Free,
        "pull flow: manual teammate receives (unobeyed) directive");
  check(!sim.get_decision().last_directive(99).has_value(),
        "pull flow: strangers get no directive");
}

// MoveUp join from the pack: a declared rider beyond detach_gap (but inside
// the group) is routed into the capped transit, arrives, and merges without
// a lateral step.
static void test_join_from_pack() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2), cfg(3), cfg(4)});
  // Three in a tight line, the fourth ~9.5 m back: same group (chain gap
  // under break_gap) but beyond detach_gap = 8, so admission must go through
  // the join transit, not the direct add.
  sim.get_engine()->get_riders().at(1)->set_start_pos(21.0);
  sim.get_engine()->get_riders().at(2)->set_start_pos(19.5);
  sim.get_engine()->get_riders().at(3)->set_start_pos(18.0);
  sim.get_engine()->get_riders().at(4)->set_start_pos(8.5);
  for (int id = 1; id <= 4; ++id)
    sim.set_rider_effort(id, 0.7);
  for (const auto& [id, r] : sim.get_engine()->get_riders())
    r->set_group_role(GroupRole::Paceline);

  // First decide tick: rotation forms from 1-3; 4 enters transit.
  for (int i = 0; i < 150; ++i)
    sim.step_fixed(dt);
  const PhysicsEngine& eng = *sim.get_engine();
  check(eng.get_group_tracker().get_snapshot().size() == 1,
        "join: one group (test premise)");
  const PacelineRotation* rot = eng.get_rotation_for(4);
  check(rot != nullptr && rot->line_depth(4) == -1 && !rot->is_sitting(4),
        "join: distant declarer is a member in transit, not in line");

  // Transit: capped effort (never a sprint), then arrival at the tail.
  int arrived_step = -1;
  double max_transit_effort = 0.0;
  double max_lat_step = 0.0;
  double prev_lat = eng.get_rider_by_id(4)->get_lat_pos();
  for (int i = 0; i < 6000; ++i) { // 60 s budget
    sim.step_fixed(dt);
    const double lat = eng.get_rider_by_id(4)->get_lat_pos();
    max_lat_step = std::max(max_lat_step, std::fabs(lat - prev_lat));
    prev_lat = lat;
    if (arrived_step < 0 && rot->line_depth(4) >= 0) {
      arrived_step = i;
      break;
    }
    max_transit_effort = std::max(
        max_transit_effort, eng.get_rider_by_id(4)->get_target_effort());
  }
  std::cout << "  [join] arrived at " << (arrived_step * dt)
            << " s, max transit effort " << max_transit_effort
            << ", max lat step " << max_lat_step << " m\n";
  check(arrived_step >= 0, "join: transit arrives and merges into the line");
  check(max_transit_effort <= 1.35,
        "join: transit effort stays capped (no sprint up the side)");
  check(max_lat_step < 0.05,
        "join: no lateral step (offset fade did its job)");
}

// ProtectLeader end-to-end: the directive installs a protect pairing through
// the policy seam and the protector ends up riding in front of the ward.
static void test_protect_directive_flow() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  const TeamId alpha = sim.get_engine()->add_team("Alpha");
  sim.add_riders({cfgt(1, alpha), cfgt(2, alpha)});
  sim.get_engine()->get_riders().at(2)->set_start_pos(3.0);
  sim.set_rider_effort(1, 0.7); // the ward paces manually
  sim.set_rider_policy(2, std::make_shared<WPrimePacingPolicy>());
  RacePlan plan;
  plan.leader = 1;
  plan.assignments = {{2, Directive::Type::ProtectLeader}};
  sim.set_race_plan(alpha, plan);

  for (int i = 0; i < 12000; ++i) // 120 s
    sim.step_fixed(dt);

  const PhysicsEngine& eng = *sim.get_engine();
  const FollowState* fs = eng.get_follow_state(2);
  check(fs != nullptr && fs->leader == 1 &&
            fs->relation == FollowRelation::Ahead,
        "protect flow: directive installed the protect pairing");
  check(sim.get_effort_source(2) == EffortSource::Follow,
        "protect flow: protect controller owns the effort");
  const double lead = eng.get_rider_by_id(2)->get_pos() -
                      eng.get_rider_by_id(1)->get_pos();
  check(lead > 0.0 && lead < 5.0,
        "protect flow: protector rides just ahead of the ward");
  check(eng.get_rider_by_id(1)->get_cda_factor() < 0.8,
        "protect flow: the ward is sheltered");
}

// The C4 gate scenario: two groups, the director orders a chase, the chase
// rotation forms and the time gap falls.  Run twice: byte-identical
// positions prove the director phase kept decide() deterministic.
static void test_chase_scenario() {
  const double dt = 0.01;
  auto run = [&](std::vector<double>& final_pos) -> bool {
    Course course = Course::create_flat();
    Simulation sim(&course);
    const TeamId alpha = sim.get_engine()->add_team("Alpha");
    sim.add_riders({cfg(1), cfg(2), cfgt(10, alpha), cfgt(11, alpha),
                    cfgt(12, alpha)});
    // Rabbits ride away while the team soft-pedals...
    sim.set_rider_effort(1, 0.9);
    sim.set_follow_target(2, 1);
    for (int id : {10, 11, 12})
      sim.set_rider_effort(id, 0.55);
    for (int i = 0; i < 5000; ++i) // 50 s: two separated groups
      sim.step_fixed(dt);

    // ...then the team goes on the radio.
    for (int id : {10, 11, 12})
      sim.set_rider_policy(id, std::make_shared<WPrimePacingPolicy>());
    RacePlan plan;
    plan.chase_gap_max = 60.0;
    sim.set_race_plan(alpha, plan);
    for (int i = 0; i < 200; ++i) // two decide ticks
      sim.step_fixed(dt);

    const DecisionSystem& ds = sim.get_decision();
    bool ok = true;
    ok &= sim.get_engine()->get_group_tracker().get_snapshot().size() == 2;
    ok &= ds.last_directive(10).has_value() &&
          ds.last_directive(10)->type == Directive::Type::Chase;
    const double gap0 =
        ds.build_context(sim, 10).time_gap_to_group_ahead;

    // Chase up to 120 s, stopping at the catch — an unwatched chase rides
    // clean through the rabbits and "group ahead" stops existing.
    bool caught = false;
    for (int i = 0; i < 12000 && !caught; ++i) {
      sim.step_fixed(dt);
      caught =
          sim.get_engine()->get_group_tracker().get_snapshot().size() == 1;
    }

    ok &= sim.get_engine()->get_rotation_for(10) != nullptr &&
          sim.get_engine()->get_rotation_for(10) ==
              sim.get_engine()->get_rotation_for(12);
    const double gap1 =
        caught ? 0.0 : ds.build_context(sim, 10).time_gap_to_group_ahead;
    std::cout << "  [chase] gap " << gap0 << " s -> " << gap1 << " s"
              << (caught ? " (caught)" : "") << "\n";
    ok &= gap0 > 0.0 && (caught || (gap1 >= 0.0 && gap1 < gap0 * 0.7));

    for (const auto& [id, r] : sim.get_engine()->get_riders())
      final_pos.push_back(r->get_pos());
    std::sort(final_pos.begin(), final_pos.end());
    return ok;
  };

  std::vector<double> pos_a, pos_b;
  check(run(pos_a), "chase: directive -> rotation -> gap falls");
  check(run(pos_b) && pos_a == pos_b,
        "chase: rerun byte-identical (director phase deterministic)");
}

int main() {
  test_draft_helpers();
  test_estimator_properties();
  test_estimator_budget_on_climb();
  test_build_context();
  test_decide_cadence();
  test_arbitration();
  test_policy_follow_ownership();
  test_policy_promote_maneuver();
  test_reconcile();
  test_reconcile_per_group_and_manual_wins();
  test_decide_determinism();
  test_pacing_budget_on_climbs();
  test_pacing_horizon_handoff();
  test_pacing_recovery_on_descent();
  test_gate_policy_vs_schedule();
  test_director_rules();
  test_tactics_orders_and_clamp();
  test_directive_pull_flow();
  test_join_from_pack();
  test_protect_directive_flow();
  test_chase_scenario();

  if (checks_failed) {
    std::cout << checks_failed << " check(s) FAILED\n";
    return 1;
  }
  std::cout << "All decision tests passed\n";
  return 0;
}
