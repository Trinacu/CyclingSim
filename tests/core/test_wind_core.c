/*
 * test_wind_core.c
 *
 * B1 core checks on the scalar env->headwind contract:
 *   1. Terminal speed ordering: headwind < still air < tailwind.
 *   2. Sign-fix proof: a tailwind stronger than rider speed pushes — a
 *      zero-effort rider accelerates from rest instead of being pinned by
 *      the (previously squared) drag term.
 *
 * (Named test_wind_core to avoid a CMake target collision with the C++
 * tests/test_wind.cpp.)
 */

#include "sim_core.h"
#include <math.h>
#include <stdio.h>

static int tests_failed = 0;

#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    if (!(cond)) {                                                             \
      ++tests_failed;                                                          \
      printf("FAIL  %s\n", msg);                                               \
    } else {                                                                   \
      printf("pass  %s\n", msg);                                               \
    }                                                                          \
  } while (0)

static EnvState flat_env(double headwind) {
  EnvState env = {.rho = 1.2234,
                  .g = 9.80665,
                  .crr = 0.0,
                  .slope = 0.0,
                  .headwind = headwind,
                  .altitude = 0.0,
                  .bearing_c0 = 0.091,
                  .bearing_c1 = 0.0087};
  return env;
}

static RiderInitParams default_params(void) {
  RiderInitParams p = {0};
  p.ftp_base = 300.0;
  p.w_prime = 20000.0;
  p.max_effort = 6.0;
  p.ftp_degrade_threshold = 2.0;
  p.ftp_degrade_rate = 0.05;
  p.max_drive_force = 700.0;
  p.oxy_p50 = 3.5;
  p.mass_rider = 80.0;
  p.cda = 0.3;
  p.mass_bike = 7.0;
  p.wheel_i = 0.14;
  p.wheel_r = 0.311;
  p.wheel_drag_factor = 0.02;
  p.crr = 0.006;
  p.drivetrain_loss = 0.02;
  return p;
}

/* Run at target_effort `effort` under constant headwind, return final speed. */
static double run_to_speed(double effort, double headwind, double seconds) {
  RiderInitParams p = default_params();
  RiderState r;
  rider_state_init(&r, &p);
  r.solver = SIM_SOLVER_ACCEL_FORCE;
  r.target_effort = effort;

  EnvState env = flat_env(headwind);
  const double dt = 0.1;
  const int steps = (int)(seconds / dt);
  for (int i = 0; i < steps; ++i)
    sim_step_rider(&r, &env, dt, NULL);

  return r.speed;
}

static void test_terminal_speed_ordering(void) {
  const double v_head = run_to_speed(1.0, 3.0, 300.0);
  const double v_still = run_to_speed(1.0, 0.0, 300.0);
  const double v_tail = run_to_speed(1.0, -3.0, 300.0);

  printf("      headwind %.2f, still %.2f, tailwind %.2f m/s\n", v_head,
         v_still, v_tail);
  CHECK(v_head < v_still - 0.5, "headwind terminal speed < still air");
  CHECK(v_still < v_tail - 0.5, "still air terminal speed < tailwind");
}

static void test_strong_tailwind_pushes_from_rest(void) {
  /* 15 m/s tailwind, zero effort: v_air = -15 at rest, so with the sign fix
   * drag is a ~44 N push against ~5 N of rolling resistance.  With the old
   * squared term the rider would sit at 0 m/s forever. */
  const double v = run_to_speed(0.0, -15.0, 120.0);
  printf("      zero-effort speed after 120 s of 15 m/s tailwind: %.2f m/s\n",
         v);
  CHECK(v > 5.0, "strong tailwind accelerates a zero-effort rider from rest");
  CHECK(v < 15.0, "coasting speed stays below the tailwind speed");
}

int main(void) {
  printf("=== wind sign / ordering ===\n");
  test_terminal_speed_ordering();
  test_strong_tailwind_pushes_from_rest();

  if (tests_failed > 0) {
    printf("=== %d check(s) FAILED ===\n", tests_failed);
    return 1;
  }
  printf("=== all checks passed ===\n");
  return 0;
}
