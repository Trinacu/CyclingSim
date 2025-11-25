#include "rider.h"
#include "course.h"
#include <cmath>
#include <iostream>
// for std::setprecision
#include <iomanip>

#include "helpers.h"

size_t Rider::global_id_counter = 0;

const double ETOL = 1e-1; // W
const double ERTOL = 1e-3;
const double PTOL = 2e-3; // m/s
const double PRTOL = 1e-4;

const double RHO = 1.2234;
const double G = 9.80665;

const double PI = 3.14159265358979323846;

bool is_close(double value, double target, double tol, double rtol) {
  return std::fabs(value - target) <= tol + rtol * std::fabs(target);
}

Bike::Bike(double mass_, double wheel_i_, double wheel_r_,
           double wheel_drag_factor_, double crr_, double dt_loss_)
    : mass(mass_), wheel_i(wheel_i_), wheel_r(wheel_r_),
      wheel_drag_factor(wheel_drag_factor_), crr(crr_), dt_loss(dt_loss_) {}

Bike Bike::create_generic() { return Bike(7.0, 0.14, 0.311, 0, 0.006, 0.02); }

Team::Team(const char* name_) : name(name_) {}

Rider::Rider(std::string name_, double ftp_base_, double mass_, double cda_,
             Bike bike_, Team team_)
    : uid(global_id_counter++), name(name_), ftp_base(ftp_base_), mass(mass_),
      cda(cda_), bike(bike_), team(team_) {
  cda_factor = 1.0;
  target_effort = 0.5;
  pos = 0;
  speed = 0;
  slope = 0;

  heading = PI / 3;

  set_cda_factor(1);
  change_bike(bike);

  timestep = 1;
}

Rider* Rider::create_generic(Team team_) {
  Bike bike = Bike::create_generic();
  return new Rider("Joe Moe", 250, 65, 0.3, bike, team_);
}

void Rider::set_course(const ICourseView* cv) { course = cv; }

std::ostream& operator<<(std::ostream& os, const Rider& r) {
  os << std::fixed << std::setprecision(1) << r.name << ":\t" << r.ftp_base
     << " W\t" << r.mass << " kg\npos: " << r.pos
     << "m\tspeed: " << r.speed * 3.6 << " km/h" << std::endl;
  return os;
}

Vector2d Rider::get_pos2d() const { return _pos2d; }

void Rider::set_pos2d(Vector2d pos) { _pos2d = pos; }

void Rider::set_cda_factor(double cda_factor_) {
  cda_factor = cda_factor_;
  effective_cda = cda_factor * cda;
  compute_drag();
}

void Rider::set_mass(double rider_mass) {
  mass = rider_mass;
  total_mass = rider_mass + bike.mass;
  compute_roll();
  compute_drag();
  compute_inertia();
}

void Rider::compute_headwind() {
  Wind wind = course->get_wind(pos);
  v_hw = wind.speed * cos(wind.heading - this->heading);
}

void Rider::compute_drag() {
  drag_coeff = 0.5 * RHO * (effective_cda + bike.wheel_drag_factor);
}

void Rider::compute_roll() { roll_coeff = bike.crr * total_mass * G; }

void Rider::compute_inertia() {
  mass_ir = total_mass + bike.wheel_i / pow(bike.wheel_r, 2);
  inertia_coeff = 0.5 * mass_ir;
}

void Rider::compute_coeff() {
  compute_drag();
  compute_roll();
  compute_inertia();
}

void Rider::change_bike(Bike new_bike) {
  bike = new_bike;
  total_mass = mass + bike.mass;
  f_grav = total_mass * G;
  compute_coeff();
}

void Rider::reset() {
  pos = 0;
  speed = 0;
  // TODO - reset energymodel!
}

void Rider::update(double dt) {
  timestep = dt;
  slope = course->get_slope(pos);
  compute_headwind();
  double ftp = ftp_base;
  // this comes from energymodel
  double effort_limit = 1;
  power = std::min(target_effort, effort_limit) * ftp;
  // energy_model.update(power, timestep);

  speed = newton(power, speed);
  pos += timestep * speed;
  altitude = course->get_altitude(pos);
  set_pos2d(Vector2d{pos, altitude});
}

double Rider::km() const { return pos / 1000.0; }

double Rider::km_h() const { return speed * 3.6; }

double Rider::pow_speed(double new_speed) const {
  double v_air = new_speed + v_hw;
  return (drag_coeff * pow(v_air, 2) * new_speed + roll_coeff * new_speed +
          (0.091 + 0.0087 * new_speed) * new_speed +
          f_grav * sin(atan(slope)) * new_speed +
          inertia_coeff * (pow(new_speed, 2) - pow(speed, 2)) / timestep) /
         (1 - bike.dt_loss);
}

double Rider::pow_speed_prime(double new_speed) {
  double v_air = new_speed + v_hw;
  return (drag_coeff * (2 * v_air * new_speed + pow(v_air, 2)) + roll_coeff +
          0.091 + 0.0174 * new_speed + f_grav * sin(atan(slope)) +
          inertia_coeff * 2 * new_speed / timestep) /
         (1 - bike.dt_loss);
}

double Rider::pow_speed_double_prime(double new_speed) {
  double v_air = new_speed + v_hw;
  return (drag_coeff * (2 * new_speed + 4 * v_air) + 0.0174 +
          inertia_coeff * 2 / timestep) /
         (1 - bike.dt_loss);
}

RiderSnapshot Rider::snapshot() const {
  return RiderSnapshot{
      .uid = this->uid,
      .name = this->name,
      .pos = this->pos,
      .slope = this->slope,
      .pos2d = this->_pos2d,
      .power = this->power,
      .speed = this->speed,
      .km_h = this->km_h(),
      .heading = this->heading,
      .team = this->team,
  };
}

double Rider::newton(double power, double speed_guess, int max_iterations) {
  double x = speed_guess;
  double x_next;
  double f, f_prime;

  for (int i = 0; i < max_iterations; ++i) {
    f = pow_speed(x) - power;
    f_prime = pow_speed_prime(x);

    if (std::abs(f_prime) < 1e-12) {
      throw std::runtime_error("Derivative too small.");
    }

    x_next = x - f / f_prime;

    // std::cout << "error: " << f << "\tprecision: " << x_next - x <<
    // std::endl;

    if (is_close(f, 0, ETOL, ERTOL) || is_close(x_next, x, PTOL, PRTOL)) {
      return x_next;
    }

    x = x_next;
  }

  throw std::runtime_error("Did not converge. Reached max iterations: " +
                           std::to_string(max_iterations));
}

double Rider::householder(double power, double speed_guess,
                          int max_iterations) {
  double x = speed_guess;
  double x_next;
  double f, f_prime, f_double_prime;

  for (int i = 0; i < max_iterations; ++i) {
    f = pow_speed(x) - power;
    f_prime = pow_speed_prime(x);
    f_double_prime = pow_speed_double_prime(x); // must define this

    if (std::abs(f_prime) < 1e-12) {
      throw std::runtime_error("Derivative too small.");
    }

    double correction = (f / f_prime);
    correction *= (1.0 + (f * f_double_prime) / (2.0 * f_prime * f_prime));

    x_next = x - correction;

    std::cout << "error: " << f << "\tprecision: " << x_next - x << std::endl;

    if (is_close(f, 0, ETOL, ERTOL) || is_close(x_next, x, PTOL, PRTOL)) {
      return x_next;
    }

    x = x_next;
  }

  throw std::runtime_error("Did not converge. Reached max iterations: " +
                           std::to_string(max_iterations));
}
