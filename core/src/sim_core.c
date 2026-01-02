#include "sim_core.h"

#include <math.h>
#include <stddef.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------
 * Internal helpers
 * ------------------------------ */

static double clamp(double x, double lo, double hi) {
  if (x < lo)
    return lo;
  if (x > hi)
    return hi;
  return x;
}

/* Skiba-style tau formulation */
static double compute_tau(const EnergyState* e, double power) {
  double dcp = e->ftp - power;
  // return 546 * exp(-0.01 * dcp) + 316;
  return e->tau_base * exp(-e->tau_slope * dcp) + e->tau_offset;
}

double piecewise(double x, double threshold) {
  if (threshold <= 0.0) {
    printf("Threshold passed into piecewise should be 0-1\n");
    return -1.0;
  }

  if (x <= 0.0) {
    return 0.0;
  }
  else if (x >= threshold) {
    return 1.0;
  }
  else {
    return x / threshold;
  }
}

#define ETOL   1e-1   /* W */
#define ERTOL  1e-3
#define PTOL   2e-3   /* m/s */
#define PRTOL  1e-4


static int is_close(double value, double target,
                    double tol, double rtol)
{
    return fabs(value - target) <= tol + rtol * fabs(target);
}

/* RIDER helpers */
static double equivalent_mass(const RiderState* r)
{
    double total_mass = r->mass_rider + r->mass_bike;
    return total_mass + r->wheel_i / (r->wheel_r * r->wheel_r);
}


/* ------------------------------
 * Public API
 * ------------------------------ */

void energy_init(EnergyState* e, double ftp, double w_prime,
                 double max_effort_base) {
  if (!e)
    return;

  e->ftp_base = ftp;
  e->ftp = ftp;
  e->w_prime = w_prime;
  e->max_effort_base = max_effort_base;

  /* Tuned to match your C++ defaults */
  e->tau_base = 546.0;
  e->tau_slope = 0.01;
  e->tau_offset = 316.0;

  e->fatigue_I = 0.0;
  e->effort_limit = max_effort_base;
}

void energy_reset(EnergyState* e) {
  if (!e)
    return;

  e->fatigue_I = 0.0;
  e->w_expended = 0.0;
  e->effort_limit = e->max_effort_base;
  e->ftp = e->ftp_base;
}

void energy_update(EnergyState* e, double power, double dt) {
  if (!e || dt <= 0.0)
    return;

  // TODO - degrade ftp
  e->w_expended += power * dt;

  if (power > e->ftp) {
    /* Linear depletion above FTP */
    e->fatigue_I += (power - e->ftp) * dt;
  } else {
    /* Exponential recovery */
    double tau = compute_tau(e, power);
    double alpha = exp(-dt / tau);
    e->fatigue_I *= alpha;
  }

  e->fatigue_I = clamp(e->fatigue_I, 0.0, e->w_prime);

  double wbal_frac = energy_wbal_fraction(e);

  // double s = sigmoid(wbal_frac, SIGMOID_K, SIGMOID_X0);
  double s = piecewise(wbal_frac, 0.2);

  // offset by 1 so it scales from max_effort_base to 1
  // ftp is effort_limit when empty
  // actually 0.8 so it reaches 0 when it's still somewhat steep
  e->effort_limit = 0.8 + (e->max_effort_base - 0.8) * s;

  // if (wbal_frac < 0.01)
  //   printf("wbal fraction: %.6f\neffort limit: %.2f", wbal_frac, e->effort_limit);

}

double energy_wbal(const EnergyState* e) {
  if (!e)
    return 0.0;
  return e->w_prime - e->fatigue_I;
}

double energy_wbal_fraction(const EnergyState* e) {
  if (!e || e->w_prime <= 0.0)
    return 0.0;
  return energy_wbal(e) / e->w_prime;
}

double energy_effort_limit(const EnergyState* e) {
  if (!e)
    return 0.0;
  return e->effort_limit;
}

/* ================================
 * RiderState helpers
 * ================================ */

void rider_state_init(RiderState* r, double ftp, double w_prime,
                      double max_effort, double mass, double cda) {
  if (!r)
    return;

  memset(r, 0, sizeof(*r));

  r->pos = 0.0;
  r->speed = 0.0;
  r->slope = 0.0;

  r->target_effort = 0.5;
  r->power = 0.0;

  r->mass_rider = mass;
  r->cda_rider = cda;
  r->ftp = ftp;

  r->heading = 0.0;

  energy_init(&r->energy, ftp, w_prime, max_effort);

  r->solver = SIM_SOLVER_POWER_BALANCE;

  printf("%.1f %.2f %.1f %.2f %.1f %.1f %.2f %.1f %.1f\n", r->pos, r->speed, r->slope, r->target_effort, r->power, r->mass_rider, r->cda_rider, r->ftp, r->heading);
  printf("INIT RiderState ptr=%p target=%.3f\n", (void*)r, r->target_effort);
}

void rider_reset(RiderState* r) {
  r->pos = 0.0;
  r->speed = 0.0;

  energy_reset(&r->energy);
}

static double pow_speed(
    double v_new,
    double v_old,
    double dt,
    const RiderState* r,
    const EnvState* env
) {
    double v_air = v_new + env->headwind;

    double cda_total = r->cda_rider + r->cda_wheel_drag;
    double drag_coeff = 0.5 * env->rho * cda_total;

    double total_mass = r->mass_rider + r->mass_bike;
    double mass_ir = equivalent_mass(r);

    double P_aero = drag_coeff * v_air * v_air * v_new;
    double P_roll = r->crr * total_mass * env->g * v_new;
    double P_bear = (env->bearing_c0 + env->bearing_c1 * v_new) * v_new;
    double P_grav = total_mass * env->g * sin(atan(env->slope)) * v_new;

    double P_inertia =
        0.5 * mass_ir * (v_new * v_new - v_old * v_old) / dt;

    double P_raw =
        P_aero + P_roll + P_bear + P_grav + P_inertia;

    return P_raw / (1.0 - r->drivetrain_loss);
}

static double pow_speed_prime(
    double v,
    double dt,
    const RiderState* r,
    const EnvState* env
) {
    double v_air = v + env->headwind;

    double cda_total = r->cda_rider + r->cda_wheel_drag;
    double drag_coeff = 0.5 * env->rho * cda_total;

    double total_mass = r->mass_rider + r->mass_bike;
    double mass_ir = equivalent_mass(r);

    double dP =
        drag_coeff * (2.0 * v_air * v + v_air * v_air) +
        r->crr * total_mass * env->g +
        env->bearing_c0 + 2.0 * env->bearing_c1 * v +
        total_mass * env->g * sin(atan(env->slope)) +
        mass_ir * v / dt;

    return dP / (1.0 - r->drivetrain_loss);
}

static int solve_speed_newton(
    double power,
    double* speed_io,
    double dt,
    const RiderState* r,
    const EnvState* env,
    StepDiagnostics* diag
) {
    double x = (*speed_io > 0.1) ? *speed_io : 0.1;

    for (int i = 0; i < 25; ++i) {
        double f = pow_speed(x, r->speed, dt, r, env) - power;
        double fp = pow_speed_prime(x, dt, r, env);

        if (fabs(fp) < 1e-12)
            break;

        double x_next = x - f / fp;
        if (x_next < 0.0)
            x_next = 0.0;

        if (is_close(f, 0.0, ETOL, ERTOL) ||
            is_close(x_next, x, PTOL, PRTOL)) {
            *speed_io = x_next;
            if (diag) {
                diag->converged = 1;
                diag->iterations = i + 1;
                diag->residual_power = f;
            }
            return 1;
        }

        x = x_next;
    }

    if (diag) {
        diag->converged = 0;
        diag->iterations = 25;
        diag->residual_power =
            pow_speed(x, r->speed, dt, r, env) - power;
    }

    *speed_io = x;
    return 0;
}

static double resistive_force(
    double v,
    const RiderState* r,
    const EnvState* env
) {
    double v_air = v + env->headwind;

    double cda = r->cda_rider + r->cda_wheel_drag;
    double drag = 0.5 * env->rho * cda * v_air * v_air;

    double total_mass = r->mass_rider + r->mass_bike;
    double roll = r->crr * total_mass * env->g;

    double grav = total_mass * env->g * sin(atan(env->slope));

    double bear = env->bearing_c0 + env->bearing_c1 * v;

    return drag + roll + grav + bear;
}

void sim_step_rider(
    RiderState* r,
    const EnvState* env,
    double dt,
    StepDiagnostics* diag
) {
    if (!r || !env || dt <= 0.0)
        return;

    /* 1. Effort limiting */
    double effort_cap = energy_effort_limit(&r->energy);
    double effort = r->target_effort;
    if (effort > effort_cap) effort = effort_cap;
    if (effort < 0.0) effort = 0.0;

    r->power = effort * r->ftp;

  if (r->solver == SIM_SOLVER_ACCEL_FORCE) {
    step_acceleration(r, env, dt);
  }
  else if (r->solver == SIM_SOLVER_ACCEL_ENERGY) {
    step_energy_accel(r, env, dt);
  } else {
    solve_speed_newton(
        r->power,
        &r->speed,
        dt,
        r,
        env,
        diag
    );
    /* 3. Integrate position */
    r->pos += r->speed * dt;
  }

    /* 4. Energy update */
    energy_update(&r->energy, r->power, dt);
}

static void step_acceleration(
  RiderState* r,
  const EnvState* env,
  double dt) {
    double v = r->speed;
    if (v < 0.1) v = 0.1;

    double mass_eq =
        r->mass_rider + r->mass_bike +
        r->wheel_i / (r->wheel_r * r->wheel_r);

    /* propulsion force */
    double F_prop = r->power * (1.0 - r->drivetrain_loss) / v;

    /* resistive forces */
    double F_res = resistive_force(v, r, env);

    double a = (F_prop - F_res) / mass_eq;

    r->speed += a * dt;
    if (r->speed < 0.0)
        r->speed = 0.0;

    r->pos += r->speed * dt;
}

static void step_energy_accel(
    RiderState* r,
    const EnvState* env,
    double dt
) {
    double v = r->speed;
    double m_eq = equivalent_mass(r);

    double P_res =
        resistive_force(v, r, env) * v;

    // double dE = (r->power - P_res) * dt;
    double dE = (r->power * (1.0 - r->drivetrain_loss) - P_res) * dt;

    double v2 = v*v + 2.0 * dE / m_eq;
    if (v2 < 0.0) v2 = 0.0;

    r->speed = sqrt(v2);
    r->pos += r->speed * dt;
}
