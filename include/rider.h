// rider.h
#ifndef RIDER_H
#define RIDER_H

#include "course.h"
#include "texturemanager.h"
#include <SDL3/SDL.h>
#include <iostream>

struct SDL_Texture;

class Team {
private:
public:
  const char* name;
  const SDL_Texture* rider_img;
  Team(const char* name_);
};

struct RiderSnapshot {
  const size_t uid;
  std::string name;
  double pos;
  double slope;
  Vector2d pos2d;
  double power;
  double effort;
  double speed;
  double km_h;
  double heading;
  Team team;
};

class Bike {
public:
  double mass;
  double wheel_i;
  double wheel_r;
  double wheel_drag_factor;
  double crr;
  double dt_loss;

  Bike(double mass_, double wheel_i_, double wheel_r_,
       double wheel_drag_factor_, double crr_, double dt_loss_);
  static Bike create_generic();
};

class Rider {
private:
  static size_t global_id_counter; // declaration
  const size_t uid;                // unique ID
  double ftp_base;
  double effort;
  double cda;
  double cda_factor;
  double effective_cda;
  double mass;
  double total_mass;
  double heading = 0;
  double v_hw;
  Vector2d _pos2d = Vector2d{0, 0};

  double power;

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

public:
  std::string name;
  double target_effort;
  double pos = 0.0;
  double altitude = 0.0;
  double speed;
  const SDL_Texture* image;

  Rider(std::string name_, double ftp_base_, double mass_, double cda_,
        Bike bike_, Team team_);
  static Rider* create_generic(Team team_);

  void set_course(const ICourseView* cv);

  RiderSnapshot snapshot() const;

  void change_bike(Bike bike_);

  void reset();
  void update(double dt);

  size_t get_id() const { return uid; }

  double km() const;
  double km_h() const;

  Vector2d get_pos2d() const;
  void set_pos2d(Vector2d pos);

  double pow_speed(double new_speed) const;
  double pow_speed_prime(double new_speed);
  double pow_speed_double_prime(double new_speed);

  void compute_drag();
  void compute_roll();
  void compute_inertia();
  void compute_coeff();
  void compute_headwind();

  double newton(double power, double speed_guess, int max_iterations = 20);
  double householder(double power, double speed_guess, int max_iterations = 20);

  // friend allows aceesing private/protected members
  friend std::ostream& operator<<(std::ostream& os, const Rider& r);
};

#endif
