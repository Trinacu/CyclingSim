// Tests for the D2 gap-holding follow controller (follow.h) — pure controller
// units first, then engine-level convergence/recovery/accordion/drop tests,
// then Simulation-level effort-source arbitration.  Mirrors test_drafting.cpp.

#include "follow.h"

#include "course.h"
#include "rider.h"
#include "sim.h"

#include <algorithm>
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

// --- Pure controller ---

static void test_controller_steady_state() {
  const FollowParams p{};

  // Converged: zero error, zero relative speed -> output is exactly the
  // integrator (the integral term holds the entire cruise effort).
  double integ = 0.7;
  const double u = follow_effort(
      {.gap = p.d0, .rel_speed = 0.0, .own_speed = 10.0, .max_effort = 6.0},
      0.01, integ, p);
  check(approx(u, 0.7), "steady state: output = integrator");
  check(approx(integ, 0.7), "steady state: integrator unchanged at e = 0");

  // Headway variant: with h > 0 the zero-error gap grows with speed.
  FollowParams ph{};
  ph.h = 0.05;
  integ = 0.7;
  const double uh = follow_effort({.gap = ph.d0 + 0.05 * 10.0,
                                   .rel_speed = 0.0,
                                   .own_speed = 10.0,
                                   .max_effort = 6.0},
                                  0.01, integ, ph);
  check(approx(uh, 0.7), "headway: e = 0 at gap = d0 + h*v");
}

static void test_controller_clamps() {
  const FollowParams p{};

  // Overlap held for a long time: integrator must floor at 0, never negative,
  // and the commanded effort must floor at 0 (no braking).
  double integ = 0.3;
  double u = 0.0;
  for (int i = 0; i < 10000; ++i)
    u = follow_effort(
        {.gap = -1.0, .rel_speed = 0.0, .own_speed = 10.0, .max_effort = 6.0},
        0.01, integ, p);
  check(approx(integ, 0.0), "overlap: integrator floors at 0");
  check(approx(u, 0.0), "overlap: commanded effort floors at 0");

  // Huge gap held for a long time: integrator and output cap at max_effort.
  integ = 0.0;
  for (int i = 0; i < 10000; ++i)
    u = follow_effort(
        {.gap = 50.0, .rel_speed = 0.0, .own_speed = 10.0, .max_effort = 6.0},
        0.01, integ, p);
  check(approx(integ, 6.0), "runaway leader: integrator caps at max_effort");
  check(approx(u, 6.0), "runaway leader: output caps at max_effort");
}

static void test_controller_dt_independence() {
  const FollowParams p{};
  const FollowInput in{
      .gap = 1.0, .rel_speed = 0.0, .own_speed = 10.0, .max_effort = 6.0};

  // Same 1 s of constant error integrated at 100 Hz vs 1 kHz: the integrator
  // (and therefore the output) must land in the same place.
  double integ_a = 0.0, integ_b = 0.0;
  double ua = 0.0, ub = 0.0;
  for (int i = 0; i < 100; ++i)
    ua = follow_effort(in, 0.01, integ_a, p);
  for (int i = 0; i < 1000; ++i)
    ub = follow_effort(in, 0.001, integ_b, p);
  check(approx(integ_a, integ_b, 1e-9), "dt-independence: integrator");
  check(approx(ua, ub, 1e-9), "dt-independence: output");
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

// Two riders from a standing start: leader on constant effort, follower on
// the controller.  The follower must latch onto the wheel and hold it —
// converged mean at the 0.25 m setpoint with no sawtooth.
static void test_convergence() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  PhysicsEngine eng(&course);
  eng.add_rider(cfg(1));
  eng.add_rider(cfg(2));
  eng.set_rider_effort(1, 0.85);
  eng.set_follow_target(2, 1);

  for (int i = 0; i < 12000; ++i) // 120 s to converge
    eng.update(dt);

  double min_gap = 1e9, max_gap = -1e9, sum = 0.0;
  const int window = 3000; // 30 s observation
  for (int i = 0; i < window; ++i) {
    eng.update(dt);
    const double g = wheel_gap(eng, 2, 1);
    min_gap = std::min(min_gap, g);
    max_gap = std::max(max_gap, g);
    sum += g;
  }
  const double mean = sum / window;
  std::cout << "  [convergence] gap mean " << mean << " m, range [" << min_gap
            << ", " << max_gap << "]\n";
  check(std::fabs(mean - 0.25) < 0.05, "convergence: mean gap at setpoint");
  check(max_gap - min_gap < 0.1, "convergence: no sawtooth (p2p < 0.1 m)");
}

// Leader soft-pedals (follower can't brake -> overlap), then resumes.  The
// integrator's >= 0 clamp means recovery is immediate: the follower re-locks
// quickly and the gap never blows up on the way back.
static void test_overlap_recovery() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  PhysicsEngine eng(&course);
  eng.add_rider(cfg(1));
  eng.add_rider(cfg(2));
  eng.set_rider_effort(1, 0.85);
  eng.set_follow_target(2, 1);

  for (int i = 0; i < 12000; ++i)
    eng.update(dt);

  eng.set_rider_effort(1, 0.1); // soft-pedal 10 s
  double min_gap = 1e9;
  for (int i = 0; i < 1000; ++i) {
    eng.update(dt);
    min_gap = std::min(min_gap, wheel_gap(eng, 2, 1));
  }

  eng.set_rider_effort(1, 0.85); // resume
  double max_gap = -1e9;
  int relock_step = -1;
  const int recovery_steps = 3000; // 30 s budget
  for (int i = 0; i < recovery_steps; ++i) {
    eng.update(dt);
    const double g = wheel_gap(eng, 2, 1);
    max_gap = std::max(max_gap, g);
    if (relock_step < 0 && std::fabs(g - 0.25) < 0.05)
      relock_step = i;
  }
  // Re-locked means it also stayed: check the last second.
  double end_dev = 0.0;
  for (int i = 0; i < 100; ++i) {
    eng.update(dt);
    end_dev = std::max(end_dev, std::fabs(wheel_gap(eng, 2, 1) - 0.25));
  }
  std::cout << "  [recovery] min gap " << min_gap << " m, max gap after resume "
            << max_gap << " m, re-lock at " << (relock_step * dt) << " s\n";
  check(min_gap < 0.25, "recovery: soft-pedal actually closed the gap");
  check(relock_step >= 0, "recovery: re-locked within 30 s");
  check(max_gap < 3.0, "recovery: gap never blew up after resume");
  check(end_dev < 0.1, "recovery: stayed locked");
}

// The tuning gate: a 5-rider chain, leader steps effort up and back down.
// The disturbance must not amplify down the chain (string stability) and the
// line must re-converge.
static void test_accordion() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  PhysicsEngine eng(&course);
  for (int id = 1; id <= 5; ++id)
    eng.add_rider(cfg(id));
  eng.set_rider_effort(1, 0.85);
  for (int id = 2; id <= 5; ++id)
    eng.set_follow_target(id, id - 1);

  for (int i = 0; i < 24000; ++i) // 240 s: let the chain unspool and settle
    eng.update(dt);

  // Leader surges 20 s, then settles back.
  eng.set_rider_effort(1, 1.05);
  double dev[6] = {0, 0, 0, 0, 0, 0};
  for (int i = 0; i < 2000; ++i) {
    eng.update(dt);
    for (int id = 2; id <= 5; ++id)
      dev[id] = std::max(dev[id],
                         std::fabs(wheel_gap(eng, id, id - 1) - 0.25));
  }
  eng.set_rider_effort(1, 0.85);
  for (int i = 0; i < 4000; ++i) {
    eng.update(dt);
    for (int id = 2; id <= 5; ++id)
      dev[id] = std::max(dev[id],
                         std::fabs(wheel_gap(eng, id, id - 1) - 0.25));
  }

  std::cout << "  [accordion] max |gap err| down the chain:";
  for (int id = 2; id <= 5; ++id)
    std::cout << " " << dev[id];
  std::cout << "\n";

  // String stability: the tail's disturbance must not exceed the first
  // follower's (small absolute slack for the lateral-model coupling noise).
  check(dev[5] <= dev[2] * 1.2 + 0.05,
        "accordion: disturbance does not amplify down the chain");

  // Everyone re-locks.
  double end_dev = 0.0;
  for (int i = 0; i < 1000; ++i) {
    eng.update(dt);
    for (int id = 2; id <= 5; ++id)
      end_dev = std::max(end_dev,
                         std::fabs(wheel_gap(eng, id, id - 1) - 0.25));
  }
  std::cout << "  [accordion] end deviation " << end_dev << " m\n";
  check(end_dev < 0.15, "accordion: chain re-converges after the surge");
}

// A follower too weak to hold the wheel must get dropped emergently — the
// energy model caps its realized effort, the gap grows past the draft cutoff,
// and it never comes back.  The controller knows nothing about effort_limit.
static void test_weak_follower_dropped() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  PhysicsEngine eng(&course);
  eng.add_rider(cfg(1));           // leader, 250 W FTP
  eng.add_rider(cfg(2, 100));      // follower, 100 W FTP
  eng.set_follow_target(2, 1);

  // Gentle start so the weak rider can latch on before the screws turn.
  eng.set_rider_effort(1, 0.5);
  for (int i = 0; i < 6000; ++i) // 60 s
    eng.update(dt);
  const double latched_gap = wheel_gap(eng, 2, 1);

  // Leader lifts the pace above what the follower can hold even in the
  // draft; W' drains, the energy model caps effort, the elastic snaps.
  eng.set_rider_effort(1, 0.95);
  for (int i = 0; i < 40000; ++i) // 400 s
    eng.update(dt);

  const double final_gap = wheel_gap(eng, 2, 1);
  std::cout << "  [drop] latched gap " << latched_gap << " m, final gap "
            << final_gap << " m, follower cda_factor "
            << eng.get_rider_by_id(2)->get_cda_factor() << "\n";
  check(std::fabs(latched_gap - 0.25) < 0.1, "drop: follower latched at 0.5x");
  check(final_gap > 8.0, "drop: weak follower dropped past the draft cutoff");
  check(approx(eng.get_rider_by_id(2)->get_cda_factor(), 1.0),
        "drop: no draft once dropped");
}

// A follower steers to its leader's wake axis (in still air: the leader's
// own line), and loses that target when the follow target is cleared.
static void test_wake_axis_steering() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  PhysicsEngine eng(&course);
  eng.add_rider(cfg(1));
  eng.add_rider(cfg(2));
  eng.set_rider_effort(1, 0.85);

  check(!eng.get_rider_by_id(2)->get_lat_target().has_value(),
        "steering: no lat target without a follow target");

  eng.set_follow_target(2, 1);
  for (int i = 0; i < 100; ++i)
    eng.update(dt);
  const auto lt = eng.get_rider_by_id(2)->get_lat_target();
  const double leader_lat = eng.get_rider_by_id(1)->get_lat_pos();
  check(lt.has_value(), "steering: follower has a lat target");
  check(lt.has_value() && std::fabs(*lt - leader_lat) < 0.1,
        "steering: target tracks the leader's line (still air)");

  eng.clear_follow_target(2);
  check(!eng.get_rider_by_id(2)->get_lat_target().has_value(),
        "steering: clear_follow_target drops the lat target");
}

// --- Simulation-level: effort-source arbitration ---

static void test_effort_source_arbitration() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2)});
  const auto effort = [&](int id) {
    return sim.get_engine()->get_rider_by_id(id)->get_target_effort();
  };

  // Manual is the default source and the slider is live.
  check(sim.get_effort_source(1) == EffortSource::Manual, "source: default Manual");
  sim.set_rider_effort(1, 0.7);
  sim.step_fixed(dt);
  check(approx(effort(1), 0.7), "manual: slider applies");

  // Follow mode: the slider is inert.
  sim.set_follow_target(1, 2);
  sim.step_fixed(dt);
  check(sim.get_effort_source(1) == EffortSource::Follow, "source: Follow");
  sim.set_rider_effort(1, 0.9);
  sim.step_fixed(dt);
  check(!approx(effort(1), 0.9), "follow: slider ignored");

  // Back to Manual: the slider path is restored.
  sim.clear_follow_target(1);
  sim.step_fixed(dt);
  check(sim.get_effort_source(1) == EffortSource::Manual,
        "source: Manual after clear");
  sim.set_rider_effort(1, 0.9);
  sim.step_fixed(dt);
  check(approx(effort(1), 0.9), "manual restored: slider applies");

  // Schedule owns effort over the slider; Follow outranks Schedule.
  sim.set_effort_schedule(
      1, std::make_shared<StepEffortSchedule>(
             std::vector<EffortBlock>{{1e9, 0.6}}));
  sim.step_fixed(dt);
  check(sim.get_effort_source(1) == EffortSource::Schedule, "source: Schedule");
  check(approx(effort(1), 0.6), "schedule: drives effort");
  sim.set_rider_effort(1, 0.3);
  sim.step_fixed(dt);
  check(approx(effort(1), 0.6), "schedule: slider ignored");

  sim.set_follow_target(1, 2);
  sim.step_fixed(dt);
  check(sim.get_effort_source(1) == EffortSource::Follow,
        "source: Follow outranks Schedule");
  sim.clear_follow_target(1);
  sim.step_fixed(dt);
  check(sim.get_effort_source(1) == EffortSource::Schedule,
        "source: Schedule restored after Follow clears");

  // reset() clears follow targets along with schedules.
  sim.set_follow_target(2, 1);
  sim.step_fixed(dt);
  check(sim.get_effort_source(2) == EffortSource::Follow, "reset precondition");
  sim.reset();
  check(sim.get_effort_source(2) == EffortSource::Manual,
        "reset: follow targets cleared");
  check(sim.get_effort_source(1) == EffortSource::Manual,
        "reset: schedules cleared");
}

int main() {
  test_controller_steady_state();
  test_controller_clamps();
  test_controller_dt_independence();
  test_convergence();
  test_overlap_recovery();
  test_accordion();
  test_weak_follower_dropped();
  test_wake_axis_steering();
  test_effort_source_arbitration();

  if (checks_failed) {
    std::cout << checks_failed << " check(s) FAILED\n";
    return 1;
  }
  std::cout << "All follow tests passed\n";
  return 0;
}
