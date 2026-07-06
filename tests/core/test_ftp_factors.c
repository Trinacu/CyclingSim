/*
 * test_ftp_factors.c
 *
 * Tests for altitude_ftp_factor() and fatigue_ftp_factor(), linked against
 * the real core_lib (no mirrored copies of the code under test).
 *
 * Units: the core works in kPa — sea-level pressure 101.325, alveolar
 * water-vapour correction 6.271, P50 ~2.5 (elite) to ~4.5 (untrained).
 */

#include "sim_core.h"

#include <math.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 * Lightweight test harness
 * ============================================================ */

static int tests_run = 0;
static int tests_failed = 0;

#define CHECK(cond, msg)                                                       \
  do {                                                                         \
    ++tests_run;                                                               \
    if (!(cond)) {                                                             \
      ++tests_failed;                                                          \
      printf("  FAIL  %s:%d  %s\n", __FILE__, __LINE__, msg);                  \
    } else {                                                                   \
      printf("  pass  %s\n", msg);                                             \
    }                                                                          \
  } while (0)

#define CHECK_NEAR(a, b, tol, msg)                                             \
  do {                                                                         \
    ++tests_run;                                                               \
    double _diff = fabs((double)(a) - (double)(b));                            \
    if (_diff > (tol)) {                                                       \
      ++tests_failed;                                                          \
      printf("  FAIL  %s:%d  %s  (got %.8f, expected %.8f, diff %.2e)\n",      \
             __FILE__, __LINE__, msg, (double)(a), (double)(b), _diff);        \
    } else {                                                                   \
      printf("  pass  %s\n", msg);                                             \
    }                                                                          \
  } while (0)

/* ============================================================
 * Helpers
 * ============================================================ */

/* Zeroed RiderState with the sealevel_sat lazy-init sentinel set to 1,
 * as rider_state_init() produces. */
static RiderState make_rider(double p50) {
  RiderState r;
  memset(&r, 0, sizeof(r));
  r.oxy_p50 = p50;
  r.sealevel_sat = 1.0; /* sentinel for lazy init */
  return r;
}

static EnergyState make_energy(double ftp_base, double w_expended,
                               double threshold, double rate) {
  EnergyState e;
  memset(&e, 0, sizeof(e));
  e.ftp_base = ftp_base;
  e.w_expended = w_expended;
  e.ftp_degrade_threshold = threshold;
  e.ftp_degrade_rate = rate;
  return e;
}

/* ============================================================
 * altitude_ftp_factor tests
 * ============================================================ */

static void test_altitude_sea_level_is_one(void) {
  printf("\n--- altitude_ftp_factor ---\n");

  /* At sea level the factor must be exactly 1.0 by construction
   * (saturation / sealevel_sat where both are computed at alt=0). */
  double p50 = 3.5; /* kPa — typical midpoint */
  RiderState r = make_rider(p50);
  double f = altitude_ftp_factor(0.0, p50, &r);

  CHECK_NEAR(f, 1.0, 1e-12, "factor == 1.0 at sea level");
}

static void test_altitude_lazy_init_fires_once(void) {
  /* sealevel_sat starts at sentinel (1), should be overwritten on first call
   * and then left unchanged on subsequent calls. */
  double p50 = 3.5;
  RiderState r = make_rider(p50);

  CHECK(r.sealevel_sat == 1.0, "sealevel_sat starts at sentinel value 1");

  altitude_ftp_factor(1000.0, p50, &r);
  double sat_after_first = r.sealevel_sat;

  CHECK(sat_after_first != 1.0, "sealevel_sat is overwritten after first call");
  CHECK_NEAR(sat_after_first, saturation(0.0, p50), 1e-15,
             "cached sealevel_sat equals saturation(0, p50)");

  altitude_ftp_factor(2000.0, p50, &r);
  CHECK(r.sealevel_sat == sat_after_first,
        "sealevel_sat is not rewritten on subsequent calls");
}

static void test_altitude_init_sets_sentinel(void) {
  /* rider_state_init must leave sealevel_sat at the sentinel so the lazy
   * init in altitude_ftp_factor fires. */
  RiderState r;
  RiderInitParams p;
  memset(&r, 0, sizeof(r));
  memset(&p, 0, sizeof(p));
  p.ftp_base = 300.0;
  p.oxy_p50 = 3.5;

  rider_state_init(&r, &p);
  CHECK(r.sealevel_sat == 1.0, "rider_state_init sets sealevel_sat sentinel");
}

static void test_altitude_factor_decreases_with_altitude(void) {
  /* Higher altitude → lower O2 → lower saturation → lower FTP factor. */
  double p50 = 3.5;
  RiderState r = make_rider(p50);

  double f0 = altitude_ftp_factor(0, p50, &r);
  double f1000 = altitude_ftp_factor(1000, p50, &r);
  double f2000 = altitude_ftp_factor(2000, p50, &r);
  double f4000 = altitude_ftp_factor(4000, p50, &r);

  CHECK(f0 > f1000, "factor decreases from 0 -> 1000 m");
  CHECK(f1000 > f2000, "factor decreases from 1000 -> 2000 m");
  CHECK(f2000 > f4000, "factor decreases from 2000 -> 4000 m");
  CHECK(f4000 > 0.0, "factor stays positive even at 4000 m");
}

static void test_altitude_factor_bounded(void) {
  /* Factor must stay in (0, 1] for any non-negative altitude. */
  double p50 = 3.5;
  RiderState r = make_rider(p50);

  double altitudes[] = {0, 500, 1000, 2000, 3000, 4000, 5000, 8848};
  int n = sizeof(altitudes) / sizeof(altitudes[0]);

  for (int i = 0; i < n; ++i) {
    double f = altitude_ftp_factor(altitudes[i], p50, &r);
    CHECK(f > 0.0 && f <= 1.0, "factor in (0, 1] at each altitude");
  }
}

static void test_altitude_higher_p50_means_lower_factor(void) {
  /* Higher P50 = right-shifted dissociation curve = loads less O2 from the
   * lungs at altitude.  At altitude, a rider with higher P50 should have a
   * lower FTP factor relative to their sea-level baseline. */
  double p50_lo = 2.5; /* kPa — left-shifted, better altitude adaptation */
  double p50_hi = 4.5; /* kPa — right-shifted, worse at altitude          */
  double alt = 3000.0;

  RiderState r_lo = make_rider(p50_lo);
  RiderState r_hi = make_rider(p50_hi);

  double f_lo = altitude_ftp_factor(alt, p50_lo, &r_lo);
  double f_hi = altitude_ftp_factor(alt, p50_hi, &r_hi);

  CHECK(f_lo > f_hi,
        "lower P50 (better alt adaptation) yields higher factor at altitude");
}

static void test_altitude_known_value(void) {
  /* Spot-check against a value computed from first principles, using the
   * same ISA constants as sim_core.c (kPa).  Catches gross implementation
   * or unit errors; if the core constants are ever tuned, update these. */
  const double H = 8500.0;      /* atmospheric scale height (m)      */
  const double PRESS0 = 101.325; /* sea-level pressure (kPa)          */
  const double O_PART = 0.2095;  /* O2 fraction of dry air            */
  const double C = 6.271;        /* alveolar water vapour (kPa)       */

  double p50 = 3.5;
  RiderState r = make_rider(p50);

  double rp = exp(-3000.0 / H);
  double alv3 = O_PART * rp * PRESS0 - C;
  double alv0 = O_PART * PRESS0 - C;
  double n = 2.7;
  double s3 = pow(alv3, n) / (pow(alv3, n) + pow(p50, n));
  double s0 = pow(alv0, n) / (pow(alv0, n) + pow(p50, n));
  double expected = s3 / s0;

  double got = altitude_ftp_factor(3000.0, p50, &r);
  CHECK_NEAR(got, expected, 1e-9,
             "factor at 3000 m matches independent calculation");
}

/* ============================================================
 * fatigue_ftp_factor tests
 * ============================================================ */

static void test_fatigue_below_threshold_is_one(void) {
  printf("\n--- fatigue_ftp_factor ---\n");

  /* No degradation until the threshold is crossed. */
  double ftp = 300.0; /* W */
  double rate = 0.05; /* 5% per (ftp*3600) J above threshold */
  double thr = 0.5;   /* threshold at 50% of (ftp * 3600) J */

  double factor_j = ftp * 3600.0;   /* J in one FTP-hour */
  double thresh_j = thr * factor_j; /* threshold in J    */

  /* Well below threshold */
  EnergyState e = make_energy(ftp, 0.0, thr, rate);
  CHECK_NEAR(fatigue_ftp_factor(&e), 1.0, 1e-12,
             "factor == 1.0 when w_expended == 0");

  /* Exactly at threshold */
  e.w_expended = thresh_j;
  CHECK_NEAR(fatigue_ftp_factor(&e), 1.0, 1e-12,
             "factor == 1.0 when w_expended == threshold exactly");

  /* Just below threshold */
  e.w_expended = thresh_j - 1.0;
  CHECK_NEAR(fatigue_ftp_factor(&e), 1.0, 1e-12,
             "factor == 1.0 when w_expended is 1 J below threshold");
}

static void test_fatigue_above_threshold_formula(void) {
  double ftp = 300.0;
  double rate = 0.05;
  double thr = 0.5;

  double factor_j = ftp * 3600.0;
  double thresh_j = thr * factor_j;

  /* One full (ftp*3600) J above the threshold: factor = 1 - 1*rate = 0.95 */
  EnergyState e = make_energy(ftp, thresh_j + factor_j, thr, rate);
  double expected = 1.0 - rate;
  CHECK_NEAR(fatigue_ftp_factor(&e), expected, 1e-12,
             "at threshold + 1 FTP-hour: factor == 1 - rate == 0.95");

  /* Half an FTP-hour above threshold: factor = 1 - 0.5*rate */
  e.w_expended = thresh_j + 0.5 * factor_j;
  expected = 1.0 - 0.5 * rate;
  CHECK_NEAR(fatigue_ftp_factor(&e), expected, 1e-12,
             "at threshold + 0.5 FTP-hour: factor == 1 - 0.5*rate");
}

static void test_fatigue_above_threshold_monotone(void) {
  /* Factor decreases as more work is expended past the threshold. */
  double ftp = 300.0;
  double rate = 0.05;
  double thr = 0.5;

  double factor_j = ftp * 3600.0;
  double thresh_j = thr * factor_j;

  EnergyState e = make_energy(ftp, thresh_j + 0.1 * factor_j, thr, rate);
  double f1 = fatigue_ftp_factor(&e);

  e.w_expended = thresh_j + 0.5 * factor_j;
  double f2 = fatigue_ftp_factor(&e);

  e.w_expended = thresh_j + 1.0 * factor_j;
  double f3 = fatigue_ftp_factor(&e);

  CHECK(f1 > f2, "factor decreases as w_expended grows past threshold");
  CHECK(f2 > f3, "factor decreases as w_expended grows past threshold");
}

static void test_fatigue_rate_zero_means_no_degradation(void) {
  /* rate == 0 means no degradation: factor stays 1.0 everywhere. */
  double ftp = 300.0;
  double thr = 0.5;

  double factor_j = ftp * 3600.0;
  double thresh_j = thr * factor_j;

  EnergyState e = make_energy(ftp, thresh_j + factor_j, thr, 0.0);
  CHECK_NEAR(fatigue_ftp_factor(&e), 1.0, 1e-12,
             "rate == 0 -> factor is 1.0 above threshold (no degradation)");
}

static void test_fatigue_threshold_zero(void) {
  /* Threshold at 0: degradation starts from the very first joule. */
  double ftp = 300.0;
  double rate = 0.05;

  double factor_j = ftp * 3600.0;

  EnergyState e = make_energy(ftp, factor_j, 0.0, rate);
  double expected = 1.0 - rate; /* = 0.95 */
  CHECK_NEAR(fatigue_ftp_factor(&e), expected, 1e-12,
             "threshold == 0: 1 FTP-hour of work gives factor == 1 - rate");
}

static void test_fatigue_floor_clamp(void) {
  /* Degradation must never take the factor below SIM_FTP_FATIGUE_FLOOR,
   * no matter how much work has been expended. */
  double ftp = 300.0;
  double rate = 0.05;
  double thr = 0.5;

  double factor_j = ftp * 3600.0;
  double thresh_j = thr * factor_j;

  /* Unclamped formula would give 1 - 20*0.05 = 0.0 */
  EnergyState e = make_energy(ftp, thresh_j + 20.0 * factor_j, thr, rate);
  CHECK_NEAR(fatigue_ftp_factor(&e), SIM_FTP_FATIGUE_FLOOR, 1e-12,
             "deep fatigue clamps to SIM_FTP_FATIGUE_FLOOR");

  /* Absurdly large expenditure: still the floor, never negative. */
  e.w_expended = thresh_j + 1e6 * factor_j;
  CHECK_NEAR(fatigue_ftp_factor(&e), SIM_FTP_FATIGUE_FLOOR, 1e-12,
             "extreme fatigue still clamps to SIM_FTP_FATIGUE_FLOOR");

  /* Just above the floor crossing: formula value, not the clamp.
   * Floor is reached at (1 - floor)/rate FTP-hours past threshold. */
  double hours_to_floor = (1.0 - SIM_FTP_FATIGUE_FLOOR) / rate;
  e.w_expended = thresh_j + (hours_to_floor - 0.1) * factor_j;
  double expected = SIM_FTP_FATIGUE_FLOOR + 0.1 * rate;
  CHECK_NEAR(fatigue_ftp_factor(&e), expected, 1e-12,
             "just before the floor crossing the formula value applies");
}

/* ============================================================
 * Entry point
 * ============================================================ */

int main(void) {
  printf("=== FTP factor tests (linked against core_lib) ===\n");

  /* altitude_ftp_factor */
  test_altitude_sea_level_is_one();
  test_altitude_lazy_init_fires_once();
  test_altitude_init_sets_sentinel();
  test_altitude_factor_decreases_with_altitude();
  test_altitude_factor_bounded();
  test_altitude_higher_p50_means_lower_factor();
  test_altitude_known_value();

  /* fatigue_ftp_factor */
  test_fatigue_below_threshold_is_one();
  test_fatigue_above_threshold_formula();
  test_fatigue_above_threshold_monotone();
  test_fatigue_rate_zero_means_no_degradation();
  test_fatigue_threshold_zero();
  test_fatigue_floor_clamp();

  printf("\n=== %d / %d tests passed ===\n", tests_run - tests_failed,
         tests_run);

  return tests_failed > 0 ? 1 : 0;
}
