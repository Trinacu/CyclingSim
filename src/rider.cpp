#include "rider.h"
#include "SDL3/SDL_log.h"
#include "course.h"
#include <cmath>

#include "sim_core.h"

#include "snapshot.h"

// B2 yaw-drag constants — empirical knobs, tune by recompile (same precedent
// as the lateral kPenaltyScale locals; settle at the interactive feel-check).
// kYawDragGain ~ 1 gives +10-15% CdA at ~20 deg yaw.  The floor and cap only
// matter near a standing start, where u -> 0 while the true force -> 0.
static constexpr double kYawDragGain = 1.0;   // k_yaw in CdA_ratio
static constexpr double kMinApparentLon = 1.0; // m/s floor on |u|
static constexpr double kYawFactorCap = 3.0;

Bike::Bike(double mass_, double wheel_i_, double wheel_r_, double wheelbase_,
           double wheel_drag_factor_, double crr_, double dt_loss_,
           BikeType type_)
    : mass(mass_), wheel_i(wheel_i_), wheel_r(wheel_r_),
      wheel_drag_factor(wheel_drag_factor_), crr(crr_), dt_loss(dt_loss_),
      type(type_), wheelbase(wheelbase_) {}

Bike Bike::create_road() {
  return Bike(7.0, 0.14, ROAD_WHEEL_RADIUS, ROAD_WHEELBASE, 0, 0.006, 0.02,
              BikeType::Road);
}
Bike Bike::create_tt() {
  return Bike(9.0, 0.14, TT_WHEEL_RADIUS, TT_WHEELBASE, 0, 0.006, 0.02,
              BikeType::TT);
}

Rider::Rider(RiderConfig config_)
    : config(config_), id(config_.rider_id), bike(config_.bike),
      name(config_.name) {
  RiderInitParams p{};
  p.ftp_base = config_.ftp_base;
  p.w_prime = config_.w_prime_base;
  p.max_effort = config_.max_effort;
  p.ftp_degrade_threshold = config_.ftp_degrade_threshold;
  p.ftp_degrade_rate = config_.ftp_degrade_rate;
  p.max_drive_force = config_.max_drive_force;
  p.mass_rider = config_.mass;
  p.cda = config_.cda;
  p.mass_bike = config_.bike.mass;
  p.wheel_i = config_.bike.wheel_i;
  p.wheel_r = config_.bike.wheel_r;
  p.wheel_drag_factor = config_.bike.wheel_drag_factor;
  p.crr = config_.bike.crr;
  p.drivetrain_loss = config_.bike.dt_loss;
  p.oxy_p50 = config_.oxy_p50;

  // lat_pos = (std::rand() % 100 - 50.0) / 200.0;
  // SDL_Log("%.2f", lat_pos);

  rider_state_init(&state, &p);
  // All riders use the SIM_SOLVER_ACCEL_FORCE default set by
  // rider_state_init: it is the only solver that respects max_drive_force
  // at launch.  The energy and power-balance solvers are kept in the core
  // as regression references (see tests/core/test_solver_compare.c).
}

std::unique_ptr<Rider> Rider::create_generic(TeamId team_id) {
  RiderConfig cfg = {0,    "Joe Moe", 250, 6,   2,     0.05,
                     700,  3.5,       65,  0.3, 24000, Bike::create_road(),
                     team_id};
  return std::make_unique<Rider>(cfg);
}

RiderConfig Rider::default_config(TeamId team_id) {
  return {0,     "Joe Moe",           250,  6, 2, 0.05, 100, 3.5, 65, 0.3,
          24000, Bike::create_road(), team_id};
}

void Rider::set_course(const ICourseView* cv) { course = cv; }

void Rider::reset() {
  rider_reset(&state);
  draft_factor_ = 1.0;
  yaw_factor_ = 1.0;
  state.cda_factor = 1.0; // not covered by rider_reset
  lat_pos = 0.0;
  lat_vel = 0.0;
  lat_target = std::nullopt;
}

void Rider::update(double dt) {
  if (!course)
    return;

  env.rho = 1.2234;
  env.g = 9.80665;

  env.slope = course->get_slope(state.pos);
  env.crr = course->get_crr(state.pos);
  state.slope = env.slope;

  heading = course->get_heading(state.pos);
  auto [wind_dir, wind_speed] = course->get_wind(state.pos);
  env.headwind = wind_speed * std::cos(wind_dir - heading);

  // B2: crosswind costs energy through yaw-dependent longitudinal drag.  The
  // core's drag term is 1/2 rho CdA cda_factor v_air |v_air|, so scaling
  // cda_factor by yaw_factor_ = CdA_ratio(yaw) V_a / |u| reproduces the
  // target force 1/2 rho CdA CdA_ratio V_a u exactly, signs included.
  const double c = wind_speed * std::sin(wind_dir - heading);
  if (c == 0.0) {
    // Exact by definition: pure longitudinal wind is fully carried by
    // env.headwind (and V_a = |u| would only misbehave under the |u| floor).
    yaw_factor_ = 1.0;
  } else {
    const double u = state.speed + env.headwind; // longitudinal apparent wind
    const double va2 = u * u + c * c;            // V_a^2, > 0 since c != 0
    const double cda_ratio = 1.0 + kYawDragGain * (c * c) / va2;
    yaw_factor_ =
        std::min(kYawFactorCap, cda_ratio * std::sqrt(va2) /
                                    std::max(std::fabs(u), kMinApparentLon));
  }
  state.cda_factor = draft_factor_ * yaw_factor_;

  double altitude = course->get_altitude(state.pos);
  env.altitude = altitude;

  env.bearing_c0 = 0.091;
  env.bearing_c1 = 0.0087;

  /* --- step physics in C --- */
  StepDiagnostics diag{};
  sim_step_rider(&state, &env, dt, &diag);

  if ((state.solver == SIM_SOLVER_POWER_BALANCE) && !diag.converged) {
    SDL_Log("Rider %d: Newton did not converge (residual %.3f W)", id,
            diag.residual_power);
  }

  /* --- sync UI-facing state --- */
  _pos2d = Vector2d{state.pos, altitude};
}

void Rider::apply_lateral_update(double new_lat_pos, double new_lat_vel,
                                 double speed_penalty) {
  lat_pos = new_lat_pos;
  lat_vel = new_lat_vel;
  state.speed *= speed_penalty;
}

void Rider::set_effort(double new_effort) { state.target_effort = new_effort; }

// these 2 are a bit odd, no?
double Rider::get_pos() const { return state.pos; }

double Rider::get_speed() const { return state.speed; }

double Rider::get_energy() const { return energy_wbal(&state.energy); }
double Rider::get_energy_fraction() const {
  return energy_wbal_fraction(&state.energy);
}

RiderSnapshot Rider::snapshot() const {
  return RiderSnapshot{
      .id = this->id,
      .group_id = this->group_id,
      .group_role = this->group_role,
      // Stamped by Simulation at snapshot time; the engine doesn't know.
      .effort_source = EffortSource::Manual,
      .policy = {},
      .name = this->name,
      .max_effort = this->state.max_effort,
      .pos = this->state.pos,
      .slope = this->state.slope,
      .heading = this->heading,
      .speed = this->state.speed,
      .effort = this->state.effort,
      .power = this->state.power,
      .wbal_fraction = this->get_energy_fraction(),
      .cda_factor = this->get_cda_factor(),
      .yaw_factor = this->yaw_factor_,
      .lat_pos = this->lat_pos,
      .pos2d = this->_pos2d,
      .team_id = this->config.team_id,
      .visual_type = this->bike.type,
  };
}

double Rider::cruise_power(double v) const {
  return sim_cruise_power(&state, &env, v);
}

double Rider::cruise_speed_at(double power, double slope, double headwind,
                              double cda_factor) const {
  RiderState s = state;
  EnvState e = env;
  s.cda_factor = cda_factor;
  e.slope = slope;
  e.headwind = headwind;
  return sim_cruise_speed(&s, &e, power);
}
