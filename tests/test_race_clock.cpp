// Tests for the C0 RaceClock (race_clock.h) — pure clock units first
// (synthetic samples, no engine), then Simulation-level integration:
// DecisionSystem feed, group time gaps in the snapshot, reset.

#include "race_clock.h"

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

// --- Pure clock ---

// Two riders at the same constant speed, 30 s apart: the trace-based gap is
// exactly 30 s at any position and any time, including inside the first
// cell (anchor interpolation) and right behind the leader's live sample.
static void test_constant_speed_gap() {
  RaceClock clock(10000.0, {}, 100.0);
  const double v = 10.0;
  for (double t = 0.0; t <= 200.0; t += 0.5) {
    clock.record(1, v * t, t);
    if (t >= 30.0)
      clock.record(2, v * (t - 30.0), t);
  }

  check(near(*clock.crossing_time(1, 1234.5), 123.45, 1e-9),
        "constant: crossing time exact at arbitrary position");
  check(near(*clock.crossing_time(1, 25.0), 2.5, 1e-9),
        "constant: exact inside the first cell (anchor interpolation)");

  const double now = 200.0;
  const double pos2 = v * (now - 30.0);
  check(near(*clock.time_gap(1, pos2, now), 30.0, 1e-9),
        "constant: 30 s gap measured as exactly 30 s");
  // Just behind the leader's live sample (between last gridline and latest).
  check(near(*clock.time_gap(1, v * now - 5.0, now), 0.5, 1e-9),
        "constant: gap right behind the leader uses the live sample");
}

// A speed step inside a cell: the estimate is off (constant-speed cell
// assumption) but bounded, and exact again on the gridlines themselves.
static void test_speed_step_bounded_error() {
  RaceClock clock(10000.0, {}, 100.0);
  // 15 m/s to pos 150 (t = 10), 5 m/s after; sampled at 100 Hz.
  double pos = 0.0, t = 0.0;
  const double dt = 0.01;
  while (pos < 400.0) {
    pos += (pos < 150.0 ? 15.0 : 5.0) * dt;
    t += dt;
    clock.record(7, pos, t);
  }

  const double true_175 = 10.0 + 25.0 / 5.0; // 15 s
  const double est_175 = *clock.crossing_time(7, 175.0);
  std::cout << "  [step] crossing 175 m: true " << true_175 << " s, est "
            << est_175 << " s (err " << est_175 - true_175 << ")\n";
  check(std::fabs(est_175 - true_175) < 5.0,
        "step: mid-cell error bounded (few seconds)");
  check(near(*clock.crossing_time(7, 200.0), 20.0, 0.02),
        "step: exact again on the gridline");
  check(near(*clock.crossing_time(7, 100.0), 100.0 / 15.0, 0.02),
        "step: gridline before the step exact too");
}

// Checkpoints are captured exactly (within one sample step) at arbitrary
// positions, once, and stay nullopt until crossed.
static void test_checkpoints() {
  RaceClock clock(1000.0, {{333.3, "TC1"}, {1000.0, "Finish"}}, 100.0);
  check(clock.checkpoints().size() == 2 &&
            clock.checkpoints()[0].label == "TC1",
        "checkpoints: list carried");

  const double v = 12.5;
  double t = 0.0;
  clock.record(3, 0.0, 0.0);
  check(!clock.checkpoint_time(3, 0).has_value(),
        "checkpoints: nullopt before crossing");
  while (t < 90.0) {
    t += 0.01;
    clock.record(3, v * t, t);
  }
  check(near(*clock.checkpoint_time(3, 0), 333.3 / v, 1e-9),
        "checkpoints: TC1 captured exactly");
  check(near(*clock.checkpoint_time(3, 1), 1000.0 / v, 1e-9),
        "checkpoints: finish captured exactly");
  check(!clock.checkpoint_time(3, 2).has_value(),
        "checkpoints: out-of-range index is nullopt");

  // A rider spawned past a checkpoint never gets a time for it.
  clock.record(4, 500.0, 0.0);
  for (t = 0.0; t < 50.0; t += 0.01)
    clock.record(4, 500.0 + v * t, t);
  check(!clock.checkpoint_time(4, 0).has_value(),
        "checkpoints: spawned-past checkpoint stays nullopt");
  check(clock.checkpoint_time(4, 1).has_value(),
        "checkpoints: later checkpoint still captured");
}

// Query edges: unknown rider, not-yet-reached positions, positions before
// the spawn, and stalls (no forward progress keeps first-crossing times).
static void test_query_edges() {
  RaceClock clock(1000.0, {}, 100.0);
  check(!clock.crossing_time(9, 10.0).has_value(), "edges: unknown rider");

  clock.record(1, 200.0, 0.0);
  for (double t = 0.0; t <= 10.0; t += 0.5)
    clock.record(1, 200.0 + 10.0 * t, t);

  check(!clock.crossing_time(1, 350.0).has_value(),
        "edges: ahead of the rider -> nullopt");
  check(!clock.crossing_time(1, 150.0).has_value(),
        "edges: before the spawn -> nullopt");
  check(!clock.time_gap(1, 350.0, 10.0).has_value(),
        "edges: gap propagates nullopt");

  // Stall: 20 s stationary at 300, then move again — 300's crossing time
  // stays the *first* arrival.
  const double t_at_300 = *clock.crossing_time(1, 300.0);
  for (double t = 10.5; t <= 30.0; t += 0.5)
    clock.record(1, 300.0, t);
  for (double t = 30.5; t <= 40.0; t += 0.5)
    clock.record(1, 300.0 + 10.0 * (t - 30.0), t);
  check(near(*clock.crossing_time(1, 300.0), t_at_300, 1e-9),
        "edges: stall keeps the first-crossing time");
  check(clock.crossing_time(1, 320.0).has_value(),
        "edges: trace continues after the stall");
}

// --- Simulation integration ---

static RiderConfig cfg(int id, double ftp = 250) {
  return RiderConfig{id,  "R" + std::to_string(id),
                     ftp, 6,
                     2,   0.05,
                     700, 3.5,
                     65,  0.3,
                     24000, Bike::create_road(),
                     kNoTeam};
}

// Two riders at very different efforts split into two groups; the snapshot
// carries a growing time gap for the chase group; reset clears the traces.
static void test_simulation_gaps() {
  const double dt = 0.01;
  Course course = Course::create_flat();
  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2)});
  sim.set_rider_effort(1, 0.9);
  sim.set_rider_effort(2, 0.5);

  for (int i = 0; i < 12000; ++i) // 120 s
    sim.step_fixed(dt);

  const RaceClock& clock = sim.get_decision().race_clock();
  const double pos2 = sim.get_engine()->get_rider_by_id(2)->get_pos();
  const auto gap = clock.time_gap(1, pos2, sim.get_sim_seconds());
  check(gap.has_value() && *gap > 5.0,
        "sim: trace gap present and substantial after the split");

  FrameSnapshot prev, curr;
  check(sim.consume_latest_frame_pair(prev, curr), "sim: frame published");
  check(curr.groups.size() == 2, "sim: two groups formed");
  if (curr.groups.size() == 2) {
    check(near(curr.groups[0].time_gap_ahead, -1.0, 1e-12),
          "sim: leading group has no gap (-1)");
    check(near(curr.groups[1].time_gap_ahead, *gap, 1.0),
          "sim: snapshot gap matches the trace gap");
  }

  // Gap keeps growing while the efforts stay uneven.
  const double g0 = *gap;
  for (int i = 0; i < 6000; ++i)
    sim.step_fixed(dt);
  const auto g1 = clock.time_gap(
      1, sim.get_engine()->get_rider_by_id(2)->get_pos(),
      sim.get_sim_seconds());
  std::cout << "  [sim] gap 120 s: " << g0 << " s -> 180 s: " << *g1 << " s\n";
  check(g1.has_value() && *g1 > g0, "sim: gap grows");

  sim.reset();
  check(!sim.get_decision().race_clock().crossing_time(1, 100.0).has_value(),
        "sim: reset clears the traces");
}

// The implicit finish checkpoint (course data) is captured through the
// whole Simulation pipeline and resolves the finish order.  (Near-equal
// riders started together draft each other to a wheel-to-wheel finish —
// order would be draft dynamics, not strength — so the pair is separated
// by effort here.)
static void test_simulation_finish_capture() {
  const double dt = 0.01;
  Course course = Course::create_flat_short(); // 1 km, implicit finish
  check(course.get_checkpoints().size() == 1 &&
            course.get_checkpoints()[0].label == "Finish" &&
            near(course.get_checkpoints()[0].pos, 1000.0, 1e-12),
        "finish: implicit checkpoint on every course");

  Simulation sim(&course);
  sim.add_riders({cfg(1), cfg(2)});
  sim.set_rider_effort(1, 0.9);
  sim.set_rider_effort(2, 0.8);

  const RaceClock& clock = sim.get_decision().race_clock();
  int guard = 0;
  while (guard++ < 30000 &&
         !(clock.checkpoint_time(1, 0) && clock.checkpoint_time(2, 0)))
    sim.step_fixed(dt);

  const auto t1 = clock.checkpoint_time(1, 0);
  const auto t2 = clock.checkpoint_time(2, 0);
  check(t1.has_value() && t2.has_value(), "finish: both captures present");
  if (t1 && t2) {
    std::cout << "  [finish] t1 " << *t1 << " s, t2 " << *t2 << " s (margin "
              << *t2 - *t1 << " s)\n";
    check(*t1 < *t2, "finish: stronger rider wins");
  }
}

int main() {
  test_constant_speed_gap();
  test_speed_step_bounded_error();
  test_checkpoints();
  test_query_edges();
  test_simulation_gaps();
  test_simulation_finish_capture();

  if (checks_failed) {
    std::cout << checks_failed << " check(s) FAILED\n";
    return 1;
  }
  std::cout << "All race clock tests passed\n";
  return 0;
}
