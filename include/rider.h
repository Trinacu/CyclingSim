// rider.h
#ifndef RIDER_H
#define RIDER_H

#include "course.h"
#include "energymodel.h"
#include "pch.hpp"
#include "snapshot.h"
#include "texturemanager.h"
#include "visualmodel.h"
#include <SDL3/SDL.h>
#include <iostream>

struct SDL_Texture;

class Team {
private:
public:
  std::string name;
  Team(const char* name_);

  int id;
};

class Bike {
public:
  double mass;
  double wheel_i;
  double wheel_r;
  double wheel_drag_factor;
  double crr;
  double dt_loss;
  BikeType type;

  Bike(double mass_, double wheel_i_, double wheel_r_,
       double wheel_drag_factor_, double crr_, double dt_loss_, BikeType type_);
  static Bike create_generic();
};

struct RiderConfig {
  std::string name;

  double ftp_base;
  double max_effort;
  double mass;
  double cda;
  double w_prime_base;
  double tau;

  Bike bike;
  Team team;
};

class Rider {
private:
  RiderConfig config;
  static int global_id_counter; // class var
  const int uid;                // instance unique ID
  double ftp_base;
  double effort;
  double max_effort;
  double cda;
  double cda_factor;
  double effective_cda;
  double mass;
  double total_mass;

  double heading;
  Vector2d _pos2d;
  double v_hw;
  double power;

  EnergyModel energymodel;

  double drag_coeff;
  double roll_coeff;
  double inertia_coeff;
  double f_grav;
  double slope;
  double mass_ir;

  double timestep;

  Bike bike;
  Team team;
  TextureManager* tex_manager;
  const ICourseView* course;

  void set_cda_factor(double cda_factor_);
  void set_mass(double total_mass_);

  void update_power_breakdown(double old_speed);

  std::array<double, (int)PowerTerm::COUNT> power_breakdown;

public:
  std::string name;
  double target_effort;
  double pos = 0.0;
  double altitude = 0.0;
  double speed;
  const SDL_Texture* image;

  Rider(RiderConfig config_);
  static Rider* create_generic(Team team_);

  void set_course(const ICourseView* cv);

  RiderSnapshot snapshot() const;

  void change_bike(Bike bike_);

  void reset();
  void update(double dt);

  bool finished() { return pos >= course->get_total_length(); }

  int get_uid() const { return uid; }

  void set_effort(double new_effort);

  double km() const;
  double km_h() const;
  double get_power() const { return power; }
  double get_energy() const;

  Vector2d get_pos2d() const;
  void set_pos2d(Vector2d pos);

  RiderConfig get_config() { return config; }

  double pow_speed(double new_speed);
  double pow_speed_prime(double new_speed) const;
  double pow_speed_double_prime(double new_speed) const;

  void compute_drag();
  void compute_roll();
  void compute_inertia();
  void compute_coeff();
  void compute_headwind();

  double newton(double power, double speed_guess, int max_iterations = 1000);
  double householder(double power, double speed_guess, int max_iterations = 20);

  // friend allows aceesing private/protected members
  friend std::ostream& operator<<(std::ostream& os, const Rider& r);
};

#endif
