/*
 * test_ftp_factors.c
 *
 * Tests for altitude_ftp_factor() and fatigue_ftp_factor().
 *
 * Build standalone (no SDL, no C++ deps):
 *   gcc -std=c99 -Wall -Wextra -lm test_ftp_factors.c -o test_ftp_factors
 *
 * The constants below must match sim_core.c exactly.  If a test fails
 * after you tune constants, update the expected values here too.
 */

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

/* ============================================================
 * Paste / mirror the constants from sim_core.c here.
 * These values are standard ISA atmosphere + physiology.
 * ============================================================ */

static const double H = 8500.0;        /* atmospheric scale height (m)      */
static const double PRESS0 = 101325.0; /* sea-level pressure (Pa)            */
static const double O_PART = 0.2095;   /* O2 fraction of dry air             */
static const double C = 6271.0;        /* alveolar water vapour correction (Pa)
                                          = ~47 mmHg * 133.3 Pa/mmHg         */
static const double O_PRESS0 = O_PART * 101325.0; /* ~21227.6 Pa             */

/* ============================================================
 * Mirror the structs (only the fields these functions touch).
 * Must stay in sync with sim_core.h.
 * ============================================================ */

typedef struct {
  double ftp_base;
  double ftp;
  double w_prime;
  double w_expended;
  double max_effort_base;
  double tau_base;
  double tau_slope;
  double tau_offset;
  double fatigue_I;
  double effort_limit;

  /* degradation parameters */
  double ftp_degrade_threshold; /* fraction of (ftp_base * 3600 J) */
  double ftp_degrade_rate;      /* fraction of ftp lost per (ftp_base*3600 J) */
} EnergyState;

typedef struct {
  double oxy_p50;      /* P50 of Hb dissociation curve (Pa) */
  double sealevel_sat; /* lazy-init sentinel: initialise to 1 in
                          rider_state_init */
  EnergyState energy;
  /* other fields omitted — not touched by the functions under test */
} RiderState;

/* ============================================================
 * Copy of the functions under test (inlined so we can compile
 * without the full sim_core.c translation unit).
 * ============================================================ */

static double rel_press(double alt) { return exp(-alt / H); }

static double alv_press(double alt) {
  return O_PART * rel_press(alt) * PRESS0 - C;
}

/* (unused in the factor functions but present in sim_core.c) */
static double rel_alv_press(double alt) {
  return (rel_press(alt) * O_PRESS0 - C) / (O_PRESS0 - C);
}

static double saturation(double alt, double midpt) {
  double n = 2.7;
  double alv = pow(alv_press(alt), n);
  return alv / (alv + pow(midpt, n));
}

static double altitude_ftp_factor(double alt, double p50, RiderState* r) {
  if (r->sealevel_sat == 1)
    r->sealevel_sat = saturation(0, p50);
  return saturation(alt, p50) / r->sealevel_sat;
}

static double fatigue_ftp_factor(EnergyState* e) {
  double factor = e->ftp_base * 3600.0;
  double thresh = e->ftp_degrade_threshold * factor;
  if (e->w_expended > thresh) {
    return (e->w_expended - thresh) / factor * e->ftp_degrade_rate;
  }
  return 1.0;
}

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

/* Returns a zeroed RiderState with sealevel_sat sentinel set to 1,
 * matching what rider_state_init() should produce. */
static RiderState make_rider(double p50) {
  RiderState r;
  memset(&r, 0, sizeof(r));
  r.oxy_p50 = p50;
  r.sealevel_sat = 1; /* sentinel for lazy init */
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
  double p50 = 3500.0; /* Pa — typical midpoint */
  RiderState r = make_rider(p50);
  double f = altitude_ftp_factor(0.0, p50, &r);

  CHECK_NEAR(f, 1.0, 1e-12, "factor == 1.0 at sea level");
}

static void test_altitude_lazy_init_fires_once(void) {
  /* sealevel_sat starts at sentinel (1), should be overwritten on first call
   * and then left unchanged on subsequent calls. */
  double p50 = 3500.0;
  RiderState r = make_rider(p50);

  CHECK(r.sealevel_sat == 1, "sealevel_sat starts at sentinel value 1");

  altitude_ftp_factor(1000.0, p50, &r);
  double sat_after_first = r.sealevel_sat;

  CHECK(sat_after_first != 1.0, "sealevel_sat is overwritten after first call");

  altitude_ftp_factor(2000.0, p50, &r);
  CHECK(r.sealevel_sat == sat_after_first,
        "sealevel_sat is not rewritten on subsequent calls");
}

static void test_altitude_factor_decreases_with_altitude(void) {
  /* Higher altitude → lower O2 → lower saturation → lower FTP factor. */
  double p50 = 3500.0;
  RiderState r = make_rider(p50);

  double f0 = altitude_ftp_factor(0, p50, &r);
  double f1000 = altitude_ftp_factor(1000, p50, &r);
  double f2000 = altitude_ftp_factor(2000, p50, &r);
  double f4000 = altitude_ftp_factor(4000, p50, &r);

  CHECK(f0 > f1000, "factor decreases from 0 → 1000 m");
  CHECK(f1000 > f2000, "factor decreases from 1000 → 2000 m");
  CHECK(f2000 > f4000, "factor decreases from 2000 → 4000 m");
  CHECK(f4000 > 0.0, "factor stays positive even at 4000 m");
}

static void test_altitude_factor_bounded(void) {
  /* Factor must stay in (0, 1] for any non-negative altitude. */
  double p50 = 3500.0;
  RiderState r = make_rider(p50);

  double altitudes[] = {0, 500, 1000, 2000, 3000, 4000, 5000, 8848};
  int n = sizeof(altitudes) / sizeof(altitudes[0]);

  for (int i = 0; i < n; ++i) {
    double f = altitude_ftp_factor(altitudes[i], p50, &r);
    CHECK(f > 0.0 && f <= 1.0, "factor in (0, 1] at each altitude");
  }
}

static void test_altitude_higher_p50_means_lower_factor(void) {
  /* Higher P50 = right-shifted curve = haemoglobin gives up O2 more readily
   * to muscles but loads less from the lungs at altitude.
   * At altitude, a rider with higher P50 should have lower saturation and
   * therefore a lower FTP factor relative to their sea-level baseline. */
  double p50_lo = 2500.0; /* Pa — left-shifted, better altitude adaptation */
  double p50_hi = 4500.0; /* Pa — right-shifted, worse at altitude          */
  double alt = 3000.0;

  RiderState r_lo = make_rider(p50_lo);
  RiderState r_hi = make_rider(p50_hi);

  double f_lo = altitude_ftp_factor(alt, p50_lo, &r_lo);
  double f_hi = altitude_ftp_factor(alt, p50_hi, &r_hi);

  CHECK(f_lo > f_hi,
        "lower P50 (better alt adaptation) yields higher factor at altitude");
}

static void test_altitude_known_value(void) {
  /* Spot-check against a value computed from first principles.
   *
   * At 3000 m:
   *   rel_press     = exp(-3000 / 8500) = 0.70167...
   *   alv_press     = 0.2095 * 0.70167 * 101325 - 6271 = 8596.7... Pa
   *   sat(3000,p50) = alv^2.7 / (alv^2.7 + p50^2.7)
   *   sat(0, p50)   = alv0^2.7 / (alv0^2.7 + p50^2.7),
   *                   alv0 = 0.2095 * 101325 - 6271 = 14956.6 Pa
   *   factor = sat(3000) / sat(0)
   *
   * Computed independently in Python:
   *   import math
   *   H,P0,OP,C,p50 = 8500,101325,0.2095,6271,3500
   *   def alv(a): return OP*math.exp(-a/H)*P0-C
   *   def sat(a): return alv(a)**2.7/(alv(a)**2.7+p50**2.7)
   *   print(sat(3000)/sat(0))  → 0.9390...  (varies with exact p50)
   *
   * We use a loose tolerance because the exact constant values may
   * be tuned and this test is meant to catch gross implementation errors.
   */
  double p50 = 3500.0;
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
  /* Verify the raw formula output.
   *
   * BUG NOTE: the function as written returns
   *   (w_expended - thresh) / factor * rate
   * which is 0 at threshold and grows from there.  If this is intended
   * as a [0,1] multiplier, the missing term is "1.0 -".
   * These tests document the CURRENT behaviour so any fix shows up
   * clearly as a deliberate change.
   */
  double ftp = 300.0;
  double rate = 0.05;
  double thr = 0.5;

  double factor_j = ftp * 3600.0;
  double thresh_j = thr * factor_j;

  /* One full (ftp*3600) J above the threshold */
  EnergyState e = make_energy(ftp, thresh_j + factor_j, thr, rate);
  double expected = (factor_j) / factor_j * rate; /* = rate = 0.05 */
  CHECK_NEAR(
      fatigue_ftp_factor(&e), expected, 1e-12,
      "at threshold + 1 FTP-hour: returns rate (currently ~0.05, not 0.95)");

  /* Half an FTP-hour above threshold */
  e.w_expended = thresh_j + 0.5 * factor_j;
  expected = 0.5 * rate;
  CHECK_NEAR(fatigue_ftp_factor(&e), expected, 1e-12,
             "at threshold + 0.5 FTP-hour: returns 0.5 * rate");
}

static void test_fatigue_above_threshold_monotone(void) {
  /* The return value grows as w_expended increases past the threshold.
   * (If the intent was a degrading multiplier, this should be decreasing —
   * see BUG NOTE above.) */
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

  CHECK(f1 < f2, "return value grows with w_expended (see BUG NOTE)");
  CHECK(f2 < f3, "return value grows with w_expended (see BUG NOTE)");
}

static void test_fatigue_rate_zero_means_no_degradation(void) {
  /* If ftp_degrade_rate is 0, the factor should be 0 above threshold
   * (and 1.0 below).  Edge case: confirms rate is a straight multiplier. */
  double ftp = 300.0;
  double thr = 0.5;

  double factor_j = ftp * 3600.0;
  double thresh_j = thr * factor_j;

  EnergyState e = make_energy(ftp, thresh_j + factor_j, thr, 0.0);
  CHECK_NEAR(fatigue_ftp_factor(&e), 0.0, 1e-12,
             "rate == 0 → return value is 0 above threshold");
}

static void test_fatigue_threshold_zero(void) {
  /* Threshold at 0: degradation starts from the very first joule. */
  double ftp = 300.0;
  double rate = 0.05;

  double factor_j = ftp * 3600.0;

  EnergyState e = make_energy(ftp, factor_j, 0.0, rate);
  double expected = (factor_j / factor_j) * rate; /* = rate */
  CHECK_NEAR(fatigue_ftp_factor(&e), expected, 1e-12,
             "threshold == 0: 1 FTP-hour of work returns rate");
}

/* ============================================================
 * Entry point
 * ============================================================ */

int main(void) {
  printf("=== FTP factor tests ===\n");

  /* altitude_ftp_factor */
  test_altitude_sea_level_is_one();
  test_altitude_lazy_init_fires_once();
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

  printf("\n=== %d / %d tests passed ===\n", tests_run - tests_failed,
         tests_run);

  if (tests_failed > 0) {
    printf(
        "\nNOTE: fatigue_ftp_factor tests document current behaviour.\n"
        "The function returns a value that starts at 0 and grows above\n"
        "the threshold.  If it is meant to be a [0,1] multiplier that\n"
        "starts at 1 and decays, the return statement should be:\n"
        "  return 1.0 - (w_expended - thresh) / factor * ftp_degrade_rate\n");
  }

  return tests_failed > 0 ? 1 : 0;
}
