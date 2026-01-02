#include "rider.h"
#include "SDL3/SDL_log.h"
#include "course.h"
#include <cmath>
#include <exception>
#include <iostream>
// for std::setprecision
#include <iomanip>
#include <numeric>

#include "sim_core.h"

#include "helpers.h"
#include "snapshot.h"

int Rider::global_id_counter = 0;

const double ETOL = 1e-1; // W
const double ERTOL = 1e-3;
const double PTOL = 2e-3; // m/s
const double PRTOL = 1e-4;

const double RHO = 1.2234;
const double G = 9.80665;

bool is_close(double value, double target, double tol, double rtol) {
  return std::fabs(value - target) <= tol + rtol * std::fabs(target);
}

Bike::Bike(double mass_, double wheel_i_, double wheel_r_,
           double wheel_drag_factor_, double crr_, double dt_loss_,
           BikeType type_)
    : mass(mass_), wheel_i(wheel_i_), wheel_r(wheel_r_),
      wheel_drag_factor(wheel_drag_factor_), crr(crr_), dt_loss(dt_loss_),
      type(type_) {}

Bike Bike::create_generic() {
  return Bike(7.0, 0.14, 0.311, 0, 0.006, 0.02, BikeType::Road);
}

Team::Team(const char* name_) : name(name_) { id = 0; }

Rider::Rider(RiderConfig config_)
    : config(config_), id(config_.rider_id), uid(global_id_counter++),
      name(config_.name), ftp_base(config_.ftp_base), mass(config_.mass),
      cda(config_.cda), bike(config_.bike), team(config_.team),
      effort_limit(config_.max_effort) {
  rider_state_init(&state, config_.ftp_base, config_.w_prime_base,
                   config_.max_effort, config_.mass, config_.cda);

  state.mass_bike = config_.bike.mass;
  state.wheel_i = config.bike.wheel_i;
  state.wheel_r = config_.bike.wheel_r;
  state.crr = config_.bike.crr;
  state.drivetrain_loss = config_.bike.dt_loss;

  if (config_.name == "Mario2")
    state.solver = SIM_SOLVER_ACCEL_ENERGY;
  if (config_.name == "Mario3")
    state.solver = SIM_SOLVER_ACCEL_FORCE;

  state.cda_wheel_drag = config_.bike.wheel_drag_factor;
}

Rider* Rider::create_generic(Team team_) {
  Bike bike = Bike::create_generic();
  RiderConfig cfg = {0, "Joe Moe", 250, 6, 65, 0.3, 24000,  bike, team_};
  return new Rider(cfg);
}

void Rider::set_course(const ICourseView* cv) { course = cv; }

Vector2d Rider::get_pos2d() const { return _pos2d; }

void Rider::set_pos2d(Vector2d pos) { _pos2d = pos; }


void Rider::reset() {
  rider_reset(&state);
  // pos = 0;
  // speed = 0;
  // energymodel.reset();
}

void Rider::update(double dt) {
  if (!course)
    return;

  env.rho = 1.2234;
  env.g = 9.80665;

  env.slope = course->get_slope(state.pos);

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

void Rider::set_effort(double new_effort) { 
  state.target_effort = new_effort;
}

// these 2 are a bit odd, no?
double Rider::km() const { return state.pos / 1000.0; }

double Rider::get_km_h() const { return state.speed * 3.6; }

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
  // power_breakdown[(int)PowerTerm::Bearings] = (0.091 + 0.0087 * speed) * speed;
  // power_breakdown[(int)PowerTerm::Gravity] = f_grav * sin(atan(slope)) * speed;
  // power_breakdown[(int)PowerTerm::Inertia] =
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

// double Rider::pow_speed(double new_speed) {
//   auto [wind_dir, wind_speed] = course->get_wind(pos);
//   double v_rel_wind = wind_speed * std::cos(wind_dir - heading);
//
//   double v_air = new_speed + v_rel_wind;
//
//   return (drag_coeff * pow(v_air, 2) * new_speed + roll_coeff * new_speed +
//           (0.091 + 0.0087 * new_speed) * new_speed +
//           f_grav * sin(atan(slope)) * new_speed +
//           inertia_coeff * (pow(new_speed, 2) - pow(speed, 2)) / timestep) /
//          (1 - bike.dt_loss);
// }
//
// double Rider::pow_speed_prime(double new_speed) const {
//   auto [wind_dir, wind_speed] = course->get_wind(pos);
//   double v_rel_wind = wind_speed * std::cos(wind_dir - heading);
//
//   double v_air = new_speed + v_rel_wind;
//   return (drag_coeff * (2 * v_air * new_speed + pow(v_air, 2)) + roll_coeff +
//           0.091 + 0.0174 * new_speed + f_grav * sin(atan(slope)) +
//           inertia_coeff * 2 * new_speed / timestep) /
//          (1 - bike.dt_loss);
// }
//
// double Rider::pow_speed_double_prime(double new_speed) const {
//   auto [wind_dir, wind_speed] = course->get_wind(pos);
//   double v_rel_wind = wind_speed * std::cos(wind_dir - heading);
//
//   double v_air = new_speed + v_rel_wind;
//   return (drag_coeff * (2 * new_speed + 4 * v_air) + 0.0174 +
//           inertia_coeff * 2 / timestep) /
//          (1 - bike.dt_loss);
// }
//
// double Rider::newton(double power, double speed_guess, int max_iterations) {
//   double x = speed_guess;
//   double x_next;
//   double f, f_prime;
//
//   for (int i = 0; i < max_iterations; ++i) {
//     f = pow_speed(x) - power;
//     f_prime = pow_speed_prime(x);
//
//     if (std::abs(f_prime) < 1e-12) {
//       throw std::runtime_error("Derivative too small.");
//     }
//
//     x_next = x - f / f_prime;
//
//     // std::cout << "error: " << f << "\tprecision: " << x_next - x <<
//     // std::endl;
//
//     if (is_close(f, 0, ETOL, ERTOL) || is_close(x_next, x, PTOL, PRTOL)) {
//       return x_next;
//     }
//
//     x = x_next;
//   }
//
//   throw std::runtime_error("Did not converge. Reached max iterations: " +
//                            std::to_string(max_iterations));
// }
//
// double Rider::householder(double power, double speed_guess,
//                           int max_iterations) {
//   double x = speed_guess;
//   double x_next;
//   double f, f_prime, f_double_prime;
//
//   for (int i = 0; i < max_iterations; ++i) {
//     f = pow_speed(x) - power;
//     f_prime = pow_speed_prime(x);
//     f_double_prime = pow_speed_double_prime(x); // must define this
//
//     if (std::abs(f_prime) < 1e-12) {
//       throw std::runtime_error("Derivative too small.");
//     }
//
//     double correction = (f / f_prime);
//     correction *= (1.0 + (f * f_double_prime) / (2.0 * f_prime * f_prime));
//
//     x_next = x - correction;
//
//     std::cout << "error: " << f << "\tprecision: " << x_next - x << std::endl;
//
//     if (is_close(f, 0, ETOL, ERTOL) || is_close(x_next, x, PTOL, PRTOL)) {
//       return x_next;
//     }
//
//     x = x_next;
//   }
//
//   throw std::runtime_error("Did not converge. Reached max iterations: " +
//                            std::to_string(max_iterations));
// }

RiderSnapshot Rider::snapshot() const {
  return RiderSnapshot{
      .uid = this->uid,
      .id = this->id,
      .name = this->name,
      .pos = this->state.pos,
      .slope = this->state.slope,
      .pos2d = this->_pos2d,
      .power = this->state.power,
      .effort = this->state.effort,
      .max_effort = this->max_effort,
      .speed = this->state.speed,
      .km_h = this->get_km_h(),
      .heading = this->heading,
      .team_id = this->team.id,
      .visual_type = this->bike.type,
      .power_breakdown = this->power_breakdown,
  };
}
