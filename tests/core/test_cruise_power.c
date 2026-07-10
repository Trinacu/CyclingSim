/*
 * test_cruise_power.c
 *
 * sim_cruise_power() must agree with the ACCEL_FORCE solver's force model:
 * at terminal velocity the drive power equals the cruise power at that
 * speed, by definition.  Checked on the flat, uphill, and into head/tail
 * wind, plus basic shape properties (monotonic in v, draft-sensitive).
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
  p.ftp_degrade_threshold = 2.0; /* hours — not reached in short runs */
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

/* Ride ACCEL_FORCE to terminal velocity in `env`, then check that
 * sim_cruise_power at that speed reproduces the drive power. */
static void check_round_trip(EnvState env, const char* label) {
  RiderInitParams p = default_params();
  RiderState r;
  rider_state_init(&r, &p);
  r.solver = SIM_SOLVER_ACCEL_FORCE;
  r.target_effort = 0.9; /* below FTP: no W' depletion, steady power */

  const double dt = 0.1;
  for (int i = 0; i < 3000; ++i) /* 300 s — many time constants */
    sim_step_rider(&r, &env, dt, NULL);

  const double cruise = sim_cruise_power(&r, &env, r.speed);
  printf("      %s: v_term %.2f m/s, drive %.1f W, cruise %.1f W\n", label,
         r.speed, r.power, cruise);
  CHECK(fabs(cruise - r.power) < 0.005 * r.power,
        "cruise power matches drive power at terminal velocity");
}

static void test_round_trips(void) {
  check_round_trip(flat_env(), "flat");

  EnvState up = flat_env();
  up.slope = 0.03;
  check_round_trip(up, "3% climb");

  EnvState head = flat_env();
  head.headwind = 3.0;
  check_round_trip(head, "3 m/s headwind");

  EnvState tail = flat_env();
  tail.headwind = -8.0;
  check_round_trip(tail, "8 m/s tailwind");
}

static void test_shape(void) {
  RiderInitParams p = default_params();
  RiderState r;
  rider_state_init(&r, &p);
  EnvState env = flat_env();

  CHECK(sim_cruise_power(&r, &env, 0.0) == 0.0, "v = 0 costs nothing");
  CHECK(sim_cruise_power(&r, &env, -1.0) == 0.0, "v < 0 returns 0");
  CHECK(sim_cruise_power(&r, &env, 12.0) > sim_cruise_power(&r, &env, 10.0),
        "cruise power is monotonic in v");

  const double solo = sim_cruise_power(&r, &env, 11.0);
  r.cda_factor = 0.5; /* sheltered */
  const double sheltered = sim_cruise_power(&r, &env, 11.0);
  printf("      11 m/s: solo %.1f W, sheltered %.1f W\n", solo, sheltered);
  CHECK(sheltered < solo, "draft (cda_factor) lowers cruise power");
}

int main(void) {
  printf("=== cruise power round-trip ===\n");
  test_round_trips();
  test_shape();

  if (tests_failed > 0) {
    printf("=== %d check(s) FAILED ===\n", tests_failed);
    return 1;
  }
  printf("=== all checks passed ===\n");
  return 0;
}
