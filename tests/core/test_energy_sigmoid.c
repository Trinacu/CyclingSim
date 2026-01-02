#include "sim_core.h"
#include <stdio.h>

int main(void) {
  EnergyState e;
  energy_init(&e, 300.0, 20000.0, 6);

  const double dt = 1;

  printf("step,wbal_frac,effort_limit\n");

  for (int i = 0; i < 100; ++i) {
    energy_update(&e, 600.0, dt);

    double frac = energy_wbal_fraction(&e);
    double limit = energy_effort_limit(&e);

    printf("iter: %d, energy: %.2f %%, effort_limit: %.2f\n", i, 100.0 * frac,
           limit);

    if (frac < 0.01 && limit > 1.01) {
      printf("FAIL: effort_limit too high near exhaustion\n");
      return 1;
    }
  }

  return 0;
}
