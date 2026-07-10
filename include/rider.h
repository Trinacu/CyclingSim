// rider.h
#ifndef RIDER_H
#define RIDER_H

#include "course.h"
#include "group.h"
#include "mytypes.h"
#include "pch.hpp"
#include "sim_core.h"
#include "snapshot.h"
#include "texturemanager.h"
#include "visualmodel.h"
#include <SDL3/SDL.h>
#include <iostream>
#include <optional>

struct SDL_Texture;

class Bike {
public:
  double mass;
  double wheel_i;
  double wheel_r;
  double wheel_drag_factor;
  double crr;
  double dt_loss;
  BikeType type;

  double wheelbase; // used for lateral physics

  Bike(double mass_, double wheel_i_, double wheel_r_, double wheelbase,
       double wheel_drag_factor_, double crr_, double dt_loss_, BikeType type_);
  static Bike create_road();
  static Bike create_tt();
};

struct RiderConfig {
  RiderId rider_id;
  std::string name;

  double ftp_base;
  double max_effort;
  double ftp_degrade_threshold;
  double ftp_degrade_rate;
  double max_drive_force;
  double oxy_p50;
  double mass;
  double cda;
  double w_prime_base;

  Bike bike;
  TeamId team_id = kNoTeam; // registry lives on the engine (team.h)
};

class Rider {
private:
  RiderConfig config;
  const RiderId id; // stable config ID - UI must not use uid
  GroupId group_id;
  GroupRole group_role = GroupRole::Unassigned;

  double heading = 0;

  // state.cda_factor is the product of these two named factors, recomputed
  // in update(): shelter (written by the drafting phase) and crosswind yaw
  // drag (computed from the wind projection) stay separable.
  double draft_factor_ = 1.0;
  double yaw_factor_ = 1.0;

  Vector2d _pos2d;
  double lat_pos = 0.0;
  double lat_vel = 0.0;
  std::optional<double> lat_target = std::nullopt;

  Bike bike;
  const ICourseView* course = nullptr;

  // C core.  env is value-initialised so pre-first-update queries
  // (cruise_power from the rotation phase) read zeros, not garbage.
  RiderState state;
  EnvState env{};

public:
  std::string name;
  const SDL_Texture* image;

  explicit Rider(RiderConfig config_);
  static std::unique_ptr<Rider> create_generic(TeamId team_id);
  static RiderConfig default_config(TeamId team_id);

  void set_course(const ICourseView* cv);

  RiderSnapshot snapshot() const;

  void change_bike(Bike bike_);

  void reset();
  void update(double dt);

  bool finished() { return course && state.pos >= course->get_total_length(); }

  RiderId get_id() const { return id; }

  void set_effort(double new_effort);

  GroupRole get_group_role() const { return group_role; }
  void set_group_role(GroupRole role) { group_role = role; }
  void clear_desired_group_role() { group_role = GroupRole::Unassigned; }

  double get_pos() const;
  double get_speed() const;
  double get_power() const { return state.power; }
  double get_energy() const;
  double get_energy_fraction() const;
  double get_effort_limit() const { return energy_effort_limit(&state.energy); }
  double get_target_effort() const { return state.target_effort; }
  double get_ftp() const { return state.ftp; } // current (degradable) FTP
  TeamId get_team_id() const { return config.team_id; }

  // Crank power needed to hold speed v in the rider's current environment
  // and draft (one-tick-stale env, same as every perception input).  Backed
  // by the core's solver force model — see sim_cruise_power (sim_core.h).
  double cruise_power(double v) const;

  // What-if cruise speed at `power` with slope, headwind and CdA factor
  // substituted into the rider's current env — the estimator's
  // window-averaged view of the road ahead (C1).
  double cruise_speed_at(double power, double slope, double headwind,
                         double cda_factor) const;
  double get_total_mass() const { return state.mass_rider + state.mass_bike; }
  double get_radius() const { return 0.5; }
  double get_bike_len() const { return bike.wheelbase + 2 * bike.wheel_r; }
  double get_heading() const { return heading; }

  // The product is what physics sees; setting writes the draft factor only
  // (drafting callers unchanged — the yaw factor is Rider-internal).
  double get_cda_factor() const { return draft_factor_ * yaw_factor_; }
  void set_cda_factor(double f) { draft_factor_ = f; }
  double get_yaw_factor() const { return yaw_factor_; }

  Vector2d get_pos2d() const;
  void set_pos2d(Vector2d pos);

  RiderConfig get_config() const { return config; }

  double get_lat_pos() const { return lat_pos; }
  double get_lat_vel() const { return lat_vel; }
  std::optional<double> get_lat_target() const { return lat_target; }
  void set_lat_target(std::optional<double> target) { lat_target = target; }
  void clear_lat_target() { lat_target = std::nullopt; }
  void apply_lateral_update(double new_lat_pos, double new_lat_vel,
                            double speed_penalty);

  // friend allows aceesing private/protected members
  friend std::ostream& operator<<(std::ostream& os, const Rider& r);
};

#endif
