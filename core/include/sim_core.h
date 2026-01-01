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

#ifdef __cplusplus
}
#endif
