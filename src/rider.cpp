#include "rider.h"
#include "SDL3/SDL_log.h"
#include "course.h"
#include <cmath>

#include "sim_core.h"

#include "snapshot.h"

bool is_close(double value, double target, double tol, double rtol) {
  return std::fabs(value - target) <= tol + rtol * std::fabs(target);
}

Bike::Bike(double mass_, double wheel_i_, double wheel_r_, double wheelbase,
           double wheel_drag_factor_, double crr_, double dt_loss_,
           BikeType type_)
    : mass(mass_), wheel_i(wheel_i_), wheel_r(wheel_r_),
      wheel_drag_factor(wheel_drag_factor_), crr(crr_), dt_loss(dt_loss_),
      type(type_) {}

Bike Bike::create_road() {
  return Bike(7.0, 0.14, ROAD_WHEEL_RADIUS, ROAD_WHEELBASE, 0, 0.006, 0.02,
              BikeType::Road);
}
Bike Bike::create_tt() {
  return Bike(9.0, 0.14, TT_WHEEL_RADIUS, TT_WHEELBASE, 0, 0.006, 0.02,
              BikeType::TT);
}

Team::Team(const char* name_) : name(name_) { id = 0; }

Rider::Rider(RiderConfig config_)
    : config(config_), id(config_.rider_id), bike(config_.bike),
      team(config_.team), name(config_.name) {
  RiderInitParams p{};
  p.ftp_base = config_.ftp_base;
  p.w_prime = config_.w_prime_base;
  p.max_effort = config_.max_effort;
  p.max_drive_force = config_.max_drive_force;
  p.mass_rider = config_.mass;
  p.cda = config_.cda;
  p.mass_bike = config_.bike.mass;
  p.wheel_i = config_.bike.wheel_i;
  p.wheel_r = config_.bike.wheel_r;
  p.wheel_drag_factor = config_.bike.wheel_drag_factor;
  p.crr = config_.bike.crr;
  p.drivetrain_loss = config_.bike.dt_loss;

  // lat_pos = (std::rand() % 100 - 50.0) / 200.0;
  // SDL_Log("%.2f", lat_pos);

  rider_state_init(&state, &p);

  if (config_.name == "Power")
    state.solver = SIM_SOLVER_POWER_BALANCE;
  if (config_.name == "AccelEnergy")
    state.solver = SIM_SOLVER_ACCEL_ENERGY;
  if (config_.name == "AccelForce")
    state.solver = SIM_SOLVER_ACCEL_FORCE;
}

std::unique_ptr<Rider> Rider::create_generic(Team team_) {
  RiderConfig cfg = {0,     "Joe Moe",           250,  6, 100, 65, 0.3,
                     24000, Bike::create_road(), team_};
  return std::make_unique<Rider>(cfg);
}

RiderConfig Rider::default_config(Team team_) {
  return {0,     "Joe Moe",           250,  6, 100, 65, 0.3,
          24000, Bike::create_road(), team_};
}

void Rider::set_course(const ICourseView* cv) { course = cv; }

void Rider::reset() {
  rider_reset(&state);
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

  auto [wind_dir, wind_speed] = course->get_wind(state.pos);
  env.headwind = wind_speed * std::cos(wind_dir - heading);

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
  double altitude = course->get_altitude(state.pos);
  _pos2d = Vector2d{state.pos, altitude};
}

void Rider::apply_lateral_update(double new_lat_pos, double new_lat_vel,
                                 double speed_penalty) {
  lat_pos = new_lat_pos;
  lat_vel = new_lat_vel;
  // state.speed *= speed_penalty;
}

void Rider::set_effort(double new_effort) { state.target_effort = new_effort; }

// these 2 are a bit odd, no?
double Rider::get_pos() const { return state.pos; }

double Rider::get_speed() const { return state.speed; }

double Rider::get_energy() const { return energy_wbal(&state.energy); }
double Rider::get_energy_fraction() const {
  return energy_wbal_fraction(&state.energy);
}

void Rider::update_power_breakdown(double old_speed) {
  // auto [wind_dir, wind_speed] = course->get_wind(pos);
  // double v_rel_wind = wind_speed * std::cos(wind_dir - heading);
  //
  // double v_air = speed + v_rel_wind;
  //
  // power_breakdown[(int)PowerTerm::Aerodynamic] =
  //     drag_coeff * pow(v_air, 2) * speed;
  // power_breakdown[(int)PowerTerm::Rolling] = roll_coeff * speed;
  // power_breakdown[(int)PowerTerm::Bearings] = (0.091 + 0.0087 * speed) *
  // speed; power_breakdown[(int)PowerTerm::Gravity] = f_grav * sin(atan(slope))
  // * speed; power_breakdown[(int)PowerTerm::Inertia] =
  //     inertia_coeff * (pow(speed, 2) - pow(old_speed, 2)) / timestep;
  // // sum without Drivetrain loss
  // double sum_raw = std::accumulate(
  //     power_breakdown.begin(),
  //     power_breakdown.begin() + (int)PowerTerm::Drivetrain, 0.0);
  //
  // power_breakdown[(int)PowerTerm::Drivetrain] = sum_raw * bike.dt_loss;
  //
  // double total =
  //     std::accumulate(power_breakdown.begin(), power_breakdown.end(), 0.0);
  // if (std::abs(total - power) > 0.2) {
  //   SDL_Log("%.2f", total - power);
  //   SDL_Log("%.2f", sum_raw);
  //   SDL_Log("%.2f", total);
  // }
}

RiderSnapshot Rider::snapshot() const {
  return RiderSnapshot{
      .id = this->id,
      .group_id = this->group_id,
      .group_role = this->group_role,
      .name = this->name,
      .max_effort = this->state.max_effort,
      .pos = this->state.pos,
      .slope = this->state.slope,
      .heading = this->heading,
      .speed = this->state.speed,
      .effort = this->state.effort,
      .power = this->state.power,
      .wbal_fraction = this->get_energy_fraction(),
      .lat_pos = this->lat_pos,
      .pos2d = this->_pos2d,
      .team_id = this->team.id,
      .visual_type = this->bike.type,
  };
}
