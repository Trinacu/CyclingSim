#include "sim_core.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

static double POS_MARGIN = 0.01;
static double SPEED_MARGIN = 0.1;

static double kinetic(double m_eq, double v) { return 0.5 * m_eq * v * v; }

int main(void) {
  const double dt = 0.001;
  const int steps = 50000;
  const double power = 300.0;

  EnvState env = {.rho = 1.2234,
                  .g = 9.80665,
                  .slope = 0.0,
                  .headwind = 0.0,
                  .bearing_c0 = 0.091,
                  .bearing_c1 = 0.0087};

  RiderState riders[3];
  SimSolverType solvers[3] = {SIM_SOLVER_POWER_BALANCE, SIM_SOLVER_ACCEL_FORCE,
                              SIM_SOLVER_ACCEL_ENERGY};

  for (int i = 0; i < 3; ++i) {
    rider_state_init(&riders[i], 300.0, 20000.0, 1.5, 75.0, 0.30);
    riders[i].mass_bike = 8.0;
    riders[i].wheel_i = 0.14;
    riders[i].wheel_r = 0.311;
    riders[i].crr = 0.006;
    riders[i].drivetrain_loss = 0.02;
    riders[i].cda_rider = 0.30;
    riders[i].cda_wheel_drag = 0.02;
    riders[i].solver = solvers[i];
    riders[i].target_effort = power / riders[i].ftp;
  }

  printf("t,v_newton,v_force,v_energy\n");

  for (int k = 0; k < steps; ++k) {
    double t = k * dt;

    for (int i = 0; i < 3; ++i)
      sim_step_rider(&riders[i], &env, dt, NULL);

    if ((fabs(riders[0].pos - riders[1].pos) > POS_MARGIN) ||
        (fabs(riders[1].pos - riders[2].pos) > POS_MARGIN) ||
        (fabs(riders[0].speed - riders[1].speed) > SPEED_MARGIN) ||
        (fabs(riders[1].speed - riders[2].speed) > SPEED_MARGIN)) {
      return 1;
    }

    printf("%.2f s\n", t);
    printf("%.6f,%.6f,%.6f m/s\n", riders[0].speed, riders[1].speed,
           riders[2].speed);
    printf("%.6f,%.6f,%.6f m\n", riders[0].pos, riders[1].pos, riders[2].pos);
  }

  return 0;
}
