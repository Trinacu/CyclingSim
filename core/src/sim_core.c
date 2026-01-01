#include "sim_core.h"

#include <math.h>
#include <stddef.h>

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

/* Sigmoid effort limiter: (0..1) */
static double sigmoid(double x, double k, double x0) {
  return 1.0 / (1.0 + exp(-k * (x - x0)));
}

static const double SIGMOID_K = 30.0;
static const double SIGMOID_X0 = 0.15;

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

  /* ------------------------------
   * Fatigue update
   * ------------------------------ */

  // TODO - add to w_expended and degrade ftp
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

  /* ------------------------------
   * Effort limiting via sigmoid
   * ------------------------------ */

  double wbal_frac = energy_wbal_fraction(e);

  /*
   * As W'bal → 0:
   *   sigmoid → 0
   *   effort_limit → 0
   */
  double s = sigmoid(wbal_frac, SIGMOID_K, SIGMOID_X0);

  e->effort_limit = e->max_effort_base * s;
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
