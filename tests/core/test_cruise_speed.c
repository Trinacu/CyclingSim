/*
 * test_cruise_speed.c
 *
 * sim_cruise_speed() is the Newton inverse of sim_cruise_power(): checked
 * as an exact round-trip, and against the ACCEL_FORCE solver's terminal
 * velocity (same force model, so the two must agree) on the flat, uphill,
 * and in head/tail wind.
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

static EnvState flat_env(void) {
  EnvState env = {.rho = 1.2234,
                  .g = 9.80665,
                  .crr = 0.0,
                  .slope = 0.0,
                  .headwind = 0.0,
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

/* cruise_speed must agree with where the ACCEL_FORCE solver actually
 * settles when riding at that power. */
static void check_against_solver(EnvState env, const char* label) {
  RiderInitParams p = default_params();
  RiderState r;
  rider_state_init(&r, &p);
  r.solver = SIM_SOLVER_ACCEL_FORCE;
  r.target_effort = 0.9;

  const double dt = 0.1;
  for (int i = 0; i < 3000; ++i)
    sim_step_rider(&r, &env, dt, NULL);

  const double v_pred = sim_cruise_speed(&r, &env, r.power);
  printf("      %s: solver %.4f m/s, cruise_speed %.4f m/s\n", label, r.speed,
         v_pred);
  CHECK(fabs(v_pred - r.speed) < 0.01,
        "cruise_speed matches the solver's terminal velocity");
}

static void test_against_solver(void) {
  check_against_solver(flat_env(), "flat");

  EnvState up = flat_env();
  up.slope = 0.06;
  check_against_solver(up, "6% climb");

  EnvState head = flat_env();
  head.headwind = 4.0;
  check_against_solver(head, "4 m/s headwind");

  EnvState tail = flat_env();
  tail.headwind = -8.0;
  check_against_solver(tail, "8 m/s tailwind");
}

static void test_round_trip(void) {
  RiderInitParams p = default_params();
  RiderState r;
  rider_state_init(&r, &p);

  const double powers[] = {80.0, 250.0, 420.0, 900.0};
  const double slopes[] = {-0.04, 0.0, 0.03, 0.10};
  for (int i = 0; i < 4; ++i) {
    for (int j = 0; j < 4; ++j) {
      EnvState env = flat_env();
      env.slope = slopes[j];
      const double v = sim_cruise_speed(&r, &env, powers[i]);
      const double back = sim_cruise_power(&r, &env, v);
      if (fabs(back - powers[i]) > 1e-4) {
        printf("      P %.0f W slope %.2f: v %.3f, back %.6f\n", powers[i],
               slopes[j], v, back);
        CHECK(0, "round-trip cruise_power(cruise_speed(P)) == P");
        return;
      }
    }
  }
  CHECK(1, "round-trip cruise_power(cruise_speed(P)) == P (16 cases)");
}

static void test_edges(void) {
  RiderInitParams p = default_params();
  RiderState r;
  rider_state_init(&r, &p);
  EnvState env = flat_env();

  CHECK(sim_cruise_speed(&r, &env, 0.0) == 0.0, "P = 0 -> v = 0");
  CHECK(sim_cruise_speed(&r, &env, -50.0) == 0.0, "P < 0 -> v = 0");
  CHECK(sim_cruise_speed(&r, &env, 300.0) >
            sim_cruise_speed(&r, &env, 200.0),
        "more power -> faster");

  const double solo = sim_cruise_speed(&r, &env, 250.0);
  r.cda_factor = 0.5;
  const double sheltered = sim_cruise_speed(&r, &env, 250.0);
  printf("      250 W: solo %.2f m/s, sheltered %.2f m/s\n", solo, sheltered);
  CHECK(sheltered > solo, "draft raises cruise speed at equal power");
}

int main(void) {
  printf("=== cruise speed (Newton inverse) ===\n");
  test_against_solver();
  test_round_trip();
  test_edges();

  if (tests_failed > 0) {
    printf("=== %d check(s) FAILED ===\n", tests_failed);
    return 1;
  }
  printf("=== all checks passed ===\n");
  return 0;
}
