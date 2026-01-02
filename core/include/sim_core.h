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
    SIM_SOLVER_POWER_BALANCE = 0,  /* Newton */
    SIM_SOLVER_ACCEL_FORCE  = 1,   /* Explicit ODE */
    SIM_SOLVER_ACCEL_ENERGY = 2   /* explicit energy based */
} SimSolverType;


/* ================================
 * Energy model (W' balance)
 * ================================ */

typedef struct {
  /* --- base parameters --- */
  double ftp_base;        /* Functional threshold power (W) */
  double ftp;             /* actual ftp that degrades */
  double w_prime;         /* Total anaerobic capacity (J) */
  double w_expended;      /* total W' expended - used for ftp degradation */
  double max_effort_base; /* Max relative effort when fresh */

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
                 double max_effort_base);

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
    double rho;        /* air density (kg/m^3) */
    double g;          /* gravity (m/s^2) */

    double slope;      /* road grade (rise/run) */
    double headwind;   /* signed headwind component (m/s) */

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
    double cda_rider;       /* already includes cda_factor */
    double cda_wheel_drag;  /* wheel / frame */

    /* control */
    double ftp;
    double target_effort;
    double effort;
    double power;

    double heading;

    /* energy system */
    EnergyState energy;

    SimSolverType solver;
} RiderState;


/* diagnostics (optional) */
typedef struct {
    int converged;
    int iterations;
    double residual_power;
} StepDiagnostics;

void rider_state_init(RiderState* r, double ft, double w_prime, double max_effort, double mass, double cda);

void rider_reset(RiderState* r);

// SOLVERS

/* core step */
void sim_step_rider(
    RiderState* r,
    const EnvState* env,
    double dt,
    StepDiagnostics* diag /* may be NULL */
);

static void step_acceleration(RiderState* r, const EnvState* env, double dt);


static void step_energy_accel(RiderState* r, const EnvState* env, double dt);


#ifdef __cplusplus
}
#endif
