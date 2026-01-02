#include "sim_core.h"
#include <math.h>
#include <stdio.h>

static int nearly(double a, double b, double eps) { return fabs(a - b) < eps; }

int main(void) {
  EnergyState e;
  energy_init(&e, 300.0, 20000.0, 1.5);

  const double dt = 0.1;

  /* Constant supra-FTP effort */
  for (int i = 0; i < 1000; ++i) {
    energy_update(&e, 450.0, dt);

    double wbal = energy_wbal(&e);
    if (wbal < 0.0) {
      printf("FAIL: W'bal < 0 at step %d\n", i);
      return 1;
    }
  }

  printf("PASS: W'bal never negative\n");
  return 0;
}
