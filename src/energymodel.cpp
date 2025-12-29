#include "energymodel.h"
#include <algorithm>
#include <cmath>

EnergyModel::EnergyModel(double w_prime_total, double ftp_base_, double tau_,
                         double max_effort_base_)
    : w_prime(w_prime_total), ftp_base(ftp_base_), ftp(ftp_base_), tau(tau_),
      max_effort(max_effort_base_) {}

double EnergyModel::compute_tau(double dcp) const {
  // Skiba / Waterworth formulation
  return 546.0 * std::exp(-0.01 * dcp) + 316.0;
}

void EnergyModel::update(double power, double dt) {
  if (power > ftp) {
    I += (power - ftp) * dt; // linear depletion
  } else {
    double alpha = std::exp(-dt / calc_tau(power));
    I *= alpha;
  }
  I = std::clamp(I, 0.0, w_prime);

  update_effort_limit();
}

void EnergyModel::update2(double power, double dt) {
  // W'exp = work rate above C1P
  double w_exp = std::max(power - ftp, 0.0);

  // decay factor
  double alpha = std::exp(-dt / tau);

  // recursive update (this IS the rearranged sum)
  I = alpha * I + w_exp * dt;

  // clamp for safety
  I = std::clamp(I, 0.0, w_prime);
}

void EnergyModel::reset() { w_prime = w_prime_base; }

double EnergyModel::get_wbal() const { return w_prime - I; }

double EnergyModel::get_ftp() const { return ftp; }

double EnergyModel::get_wbal_fraction() const {
  return (w_prime > 0.0) ? (get_wbal() / w_prime) : 0.0;
}

void EnergyModel::update_effort_limit() {
  double x = std::clamp((w_prime - I) / w_prime, 0.0, 1.0);

  constexpr double x0 = 0.15; // start limiting below 15%
  constexpr double k = 30.0;  // steepness

  effort_limit = max_effort / (1.0 + std::exp(-k * (x - x0)));
}

double EnergyModel::calc_tau(double power) {
  return 546 * std::exp(-0.01 * (ftp - power)) + 316;
}

double EnergyModel::get_effort_limit() const { return effort_limit; }
