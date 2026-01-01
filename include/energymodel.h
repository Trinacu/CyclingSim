#pragma once

#include "sim_core.h"

/*
 * Thin C++ adapter over the C99 EnergyState.
 * Owns state, delegates all logic to sim_core.
 */

class EnergyModel {
public:
  EnergyModel(double w_prime_base, double ftp_base,
              double /* tau (deprecated) */, double max_effort_base) {
    energy_init(&state, ftp_base, w_prime_base, max_effort_base);
  }

  void reset() { energy_reset(&state); }

  void update(double power, double dt) { energy_update(&state, power, dt); }

  double get_wbal() const { return energy_wbal(&state); }

  double get_wbal_fraction() const { return energy_wbal_fraction(&state); }

  double get_effort_limit() const { return energy_effort_limit(&state); }

  double get_ftp() const { return state.ftp; }

private:
  EnergyState state;
};
