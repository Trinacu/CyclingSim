#define _POSIX_C_SOURCE 199309L

#include "sim_core.h"

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <time.h>

/* ---------- timing helpers ---------- */

static double now_seconds(void) {
  struct timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return ts.tv_sec + ts.tv_nsec * 1e-9;
}

static double course_slope(double pos_m) {
  double cycle = fmod(pos_m, 400.0);

  if (cycle < 100.0)
    return 0.0;
  if (cycle < 200.0)
    return 0.05;
  if (cycle < 300.0)
    return 0.10;
  return 0.15;
}

/* ---------- benchmark ---------- */

static void run_benchmark(const char* name, RiderState* r, EnvState* env,
                          double dt, int steps) {
  double t0 = now_seconds();

  for (int i = 0; i < steps; ++i) {
    env->slope = course_slope(r->pos);
    sim_step_rider(r, env, dt, NULL);
  }

  double t1 = now_seconds();
  double elapsed = t1 - t0;

  printf("%-22s : %8.3f ms  |  %.2f ns/step  | v=%.2f m/s\n", name,
         elapsed * 1000.0, elapsed * 1e9 / steps, r->speed);
}

int main(void) {
  const int steps = 200000; /* tune if needed */
  const double dt = 0.01;

  /* ---------- environment ---------- */
  EnvState env = {.rho = 1.2234,
                  .g = 9.80665,
                  .slope = 0.0,
                  .headwind = 0.0,
                  .bearing_c0 = 0.091,
                  .bearing_c1 = 0.0087};

  /* ---------- baseline rider ---------- */
  RiderState base;
  memset(&base, 0, sizeof(base));

  base.mass_rider = 70.0;
  base.mass_bike = 8.0;
  base.wheel_i = 0.14;
  base.wheel_r = 0.311;

  base.crr = 0.005;
  base.drivetrain_loss = 0.02;

  base.cda_rider = 0.30;
  base.cda_wheel_drag = 0.02;

  base.ftp = 300.0;
  base.target_effort = 0.8;

  energy_init(&base.energy, 300.0, 20000.0, 1.5);

  /* ---------- run solvers ---------- */

  RiderState r1 = base;
  r1.solver = SIM_SOLVER_POWER_BALANCE;
  run_benchmark("POWER_BALANCE (Newton)", &r1, &env, dt, steps);

  RiderState r2 = base;
  r2.solver = SIM_SOLVER_ACCEL_FORCE;
  run_benchmark("ACCEL_FORCE", &r2, &env, dt, steps);

  RiderState r3 = base;
  r3.solver = SIM_SOLVER_ACCEL_ENERGY;
  run_benchmark("ACCEL_ENERGY", &r3, &env, dt, steps);

  return 0;
}
