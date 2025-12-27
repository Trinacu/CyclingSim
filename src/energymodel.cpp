#include "energymodel.h"
#include <algorithm>
#include <cmath>

EnergyModel::EnergyModel(double w_prime_total, double ftp_base_, double tau_)
    : w_prime(w_prime_total), ftp_base(ftp_base_), tau(tau_) {}

double EnergyModel::compute_tau(double dcp) const {
  // Skiba / Waterworth formulation
  return 546.0 * std::exp(-0.01 * dcp) + 316.0;
}

void EnergyModel::update(double power, double dt) {
  // W'exp = work rate above CP
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
