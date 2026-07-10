#pragma once

/*
 * sim_core.h
 *
 * Pure C99 simulation core.
 * C / C++ ABI stable.
 */

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  SIM_SOLVER_POWER_BALANCE = 0, /* Newton */
  SIM_SOLVER_ACCEL_FORCE = 1,   /* Explicit ODE */
  SIM_SOLVER_ACCEL_ENERGY = 2   /* explicit energy based */
} SimSolverType;

/* ================================
 * Energy model (W' balance)
 * ================================ */

typedef struct {
  /* --- base parameters --- */
  double ftp_base;   /* Functional threshold power (W) */
  double ftp;        /* actual ftp that degrades */
  double w_prime;    /* Total anaerobic capacity (J) */
  double w_expended; /* total W' expended - used for ftp degradation */
  double ftp_degrade_threshold; /* when ftp starts degrading */
  double ftp_degrade_rate; /* every hour past threshold at FTP degrade rate */
  double max_effort_base;  /* Max relative effort when fresh */

  /* --- recovery parameters --- */
  double tau_base;   /* Baseline tau (s) */
  double tau_slope;  /* Exponential sensitivity */
  double tau_offset; /* constant part */

  /* --- state --- */
  double fatigue_I;    /* accumulated fatigue integral (J) */
  double effort_limit; /* current effort cap */
} EnergyState;

/* Initialization / reset */
void energy_init(EnergyState* e, double ftp, double w_prime,
                 double ftp_degrade_threshold, double max_effort_base);

void energy_reset(EnergyState* e);

/* Update once per physics step */
void energy_update(EnergyState* e, double power, double dt);

/* Queries */
double energy_wbal(const EnergyState* e);
double energy_wbal_fraction(const EnergyState* e);
double energy_effort_limit(const EnergyState* e);

double piecewise(double x, double threshold);

/* ================================
 * Environment (inputs)
 * ================================ */

typedef struct {
  double rho; /* air density (kg/m^3) */
  double g;   /* gravity (m/s^2) */

  double crr; /* 0 for good surface, up to 0.02 gravel/cobbles */

  double slope;    /* road grade (rise/run) */
  double headwind; /* signed headwind component (m/s) */

  double altitude;

  /* bearing loss model */
  double bearing_c0; /* 0.091 */
  double bearing_c1; /* 0.0087 */
} EnvState;

/* ================================
 * Rider physics state
 * ================================ */

typedef struct {
  /* kinematics */
  double pos;
  double speed;

  double altitude;
  double slope;

  /* masses */
  double mass_rider;
  double mass_bike;
  double wheel_i;
  double wheel_r;

  /* resistance / losses */
  double crr;
  double drivetrain_loss; /* fraction */

  /* aerodynamics */
  double cda_rider;  /* rider CdA (m^2), excludes cda_factor */
  double cda_factor; /* position/drafting multiplier; 1.0 = baseline */
  double cda_wheel_drag; /* wheel / frame */

  /* control */
  double ftp;
  double target_effort;
  double max_effort;
  double effort;
  double power;

  double oxy_p50;
  double sealevel_sat;

  double max_drive_force;

  double heading;

  /* energy system */
  EnergyState energy;

  SimSolverType solver;
} RiderState;

typedef struct {
  double ftp_base;
  double w_prime;
  double max_effort;
  double ftp_degrade_threshold; /* hours of FTP expenditure */
  double ftp_degrade_rate; /* FTP fraction lost per FTP-hour past threshold */
  double max_drive_force;  // 500 - 1000 N
  double oxy_p50;               // 2.5 elite, 4.0 avg Joe
  double mass_rider;
  double cda;
  double mass_bike;
  double wheel_i;
  double wheel_r;
  double wheel_drag_factor;
  double crr;
  double drivetrain_loss;
} RiderInitParams;

void rider_state_init(RiderState* r, const RiderInitParams* p);

/* ================================
 * FTP degradation factors
 * ================================ */

/* Lowest fraction of base FTP that fatigue degradation can reach. */
#define SIM_FTP_FATIGUE_FLOOR 0.5

/* Hill-type O2 saturation of haemoglobin at altitude.
 * midpt is the P50 of the dissociation curve in kPa (elite ~2.5, avg ~4.0). */
double saturation(double alt, double midpt);

/* Ratio of saturation at `alt` to the rider's sea-level saturation, in (0,1].
 * Lazily caches r->sealevel_sat on first call (sentinel value 1 set by
 * rider_state_init). p50 in kPa. */
double altitude_ftp_factor(double alt, double p50, RiderState* r);

/* 1.0 until w_expended crosses ftp_degrade_threshold FTP-hours, then decays
 * by ftp_degrade_rate per FTP-hour, clamped at SIM_FTP_FATIGUE_FLOOR. */
double fatigue_ftp_factor(EnergyState* e);

/* diagnostics (optional) */
typedef struct {
  int converged;
  int iterations;
  double residual_power;
} StepDiagnostics;

void rider_reset(RiderState* r);

// SOLVERS

/* core step */
void sim_step_rider(RiderState* r, const EnvState* env, double dt,
                    StepDiagnostics* diag /* may be NULL */
);

/* Steady-state (cruise) crank power required to hold speed v: resistive
 * forces at v — aero (incl. cda_factor), rolling, gravity, bearings —
 * times v, inflated by drivetrain loss.  Shares resistive_force() with the
 * ACCEL_FORCE solver, so the two can never drift apart.  Perception /
 * decision helper (C-pre-b move-up cap, C1 pace estimator); the solvers
 * themselves do not call it.  Returns 0 for v <= 0. */
double sim_cruise_power(const RiderState* r, const EnvState* env, double v);

/* Inverse: the steady speed at which sim_cruise_power(v) == power.
 * Safeguarded Newton inside an expanding bisection bracket (~5-10
 * evaluations).  Returns 0 for power <= 0; agrees with the ACCEL_FORCE
 * solver's terminal velocity by construction (shared force model). */
double sim_cruise_speed(const RiderState* r, const EnvState* env,
                        double power);

#ifdef __cplusplus
}
#endif
