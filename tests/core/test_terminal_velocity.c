/*
 * test_terminal_velocity.c
 *
 * End-to-end sanity checks on sim_step_rider():
 *   1. Terminal velocity on a flat course is physically plausible for all
 *      three solvers (catches aero drag being disabled, e.g. cda_factor = 0).
 *   2. Higher CdA yields a lower terminal velocity (drag is actually live).
 *   3. FTP fatigue degradation engages once w_expended passes the threshold
 *      (catches ftp_degrade_rate being dropped between config and core).
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

/* Run a rider at target_effort 1.0 for `seconds` and return final speed. */
static double run_to_speed(const RiderInitParams* p, SimSolverType solver,
                           double seconds) {
  RiderState r;
  rider_state_init(&r, p);
  r.solver = solver;
  r.target_effort = 1.0;

  EnvState env = flat_env();
  const double dt = 0.1;
  const int steps = (int)(seconds / dt);
  for (int i = 0; i < steps; ++i)
    sim_step_rider(&r, &env, dt, NULL);

  return r.speed;
}

static void test_terminal_velocity_plausible(void) {
  const char* names[3] = {"POWER_BALANCE", "ACCEL_FORCE", "ACCEL_ENERGY"};
  SimSolverType solvers[3] = {SIM_SOLVER_POWER_BALANCE, SIM_SOLVER_ACCEL_FORCE,
                              SIM_SOLVER_ACCEL_ENERGY};
  RiderInitParams p = default_params();

  for (int i = 0; i < 3; ++i) {
    double v = run_to_speed(&p, solvers[i], 300.0);
    double kmh = v * 3.6;
    printf("      %s terminal speed: %.1f km/h\n", names[i], kmh);
    CHECK(kmh > 30.0 && kmh < 55.0,
          "terminal speed at FTP on flat course is 30-55 km/h");
  }
}

static void test_higher_cda_is_slower(void) {
  RiderInitParams lo = default_params(); /* cda 0.3 */
  RiderInitParams hi = default_params();
  hi.cda = 0.6;

  double v_lo = run_to_speed(&lo, SIM_SOLVER_ACCEL_FORCE, 300.0);
  double v_hi = run_to_speed(&hi, SIM_SOLVER_ACCEL_FORCE, 300.0);

  printf("      cda 0.3: %.2f m/s, cda 0.6: %.2f m/s\n", v_lo, v_hi);
  CHECK(v_hi < v_lo - 0.5, "doubling CdA lowers terminal speed noticeably");
}

static void test_fatigue_degradation_engages(void) {
  RiderInitParams p = default_params();
  p.ftp_degrade_threshold = 0.01; /* ~36 s at FTP before degradation starts */
  p.ftp_degrade_rate = 0.05;

  RiderState r;
  rider_state_init(&r, &p);
  r.solver = SIM_SOLVER_ACCEL_FORCE;
  r.target_effort = 1.0;

  EnvState env = flat_env();
  const double dt = 0.1;
  for (int i = 0; i < 6000; ++i) /* 600 s at ~FTP */
    sim_step_rider(&r, &env, dt, NULL);

  printf("      ftp after 600 s: %.2f W (base %.2f W)\n", r.ftp, p.ftp_base);
  CHECK(r.energy.w_expended >
            p.ftp_degrade_threshold * p.ftp_base * 3600.0,
        "test premise: w_expended passed the degradation threshold");
  CHECK(r.ftp < 0.999 * p.ftp_base, "ftp degrades once past the threshold");
  CHECK(r.ftp >= 0.5 * p.ftp_base, "degraded ftp respects the floor");
}

int main(void) {
  printf("=== terminal velocity / degradation sanity ===\n");
  test_terminal_velocity_plausible();
  test_higher_cda_is_slower();
  test_fatigue_degradation_engages();

  if (tests_failed > 0) {
    printf("=== %d check(s) FAILED ===\n", tests_failed);
    return 1;
  }
  printf("=== all checks passed ===\n");
  return 0;
}
