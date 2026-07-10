#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include "Eigen/Core"
#include "group.h"
#include "mytypes.h"
#include "pch.hpp"
#include "visualmodel.h"
#include <string>
#include <unordered_map>

enum class PowerTerm : int {
  Aerodynamic = 0,
  Rolling,
  Bearings,
  Gravity,
  Inertia,
  Drivetrain,
  COUNT
};

static constexpr const char* POWER_LABELS[] = {"Aero", "Roll",  "Bear",
                                               "Grav", "Inert", "Drive"};

struct RiderRenderState {
  RiderId id;
  GroupId group_id = kNoGroup;
  GroupRole group_role = GroupRole::Unassigned;
  // C2: who owns the rider's effort right now + the policy's name (empty =
  // no policy).  Stamped by Simulation at snapshot time (the engine alone
  // can't know about schedules/policies).
  EffortSource effort_source = EffortSource::Manual;
  std::string policy;
  std::string name;
  double max_effort;
  double pos;
  double slope;
  double speed;
  double effort;
  double power;
  double wbal_fraction;

  // added
  double heading;

  // interpolated
  double lat_pos;
  Vector2d pos2d;

  int team_id;

  BikeType visual_type;
  // std::array<double, (int)PowerTerm::COUNT> power_breakdown;
};

struct RiderSnapshot {
  RiderId id;
  GroupId group_id;
  GroupRole group_role;
  // C2: see RiderRenderState — stamped by Simulation at snapshot time.
  EffortSource effort_source = EffortSource::Manual;
  std::string policy;
  std::string name;
  double max_effort;
  double pos;
  double slope;
  double heading;
  double speed;
  double effort;
  double power;
  double wbal_fraction;
  double cda_factor; // total CdA multiplier physics sees: draft x yaw
  double yaw_factor; // crosswind yaw component alone; 1.0 = no crosswind

  // interpolated
  double lat_pos;
  Vector2d pos2d;

  int team_id;

  BikeType visual_type;
  // std::array<double, (int)PowerTerm::COUNT> power_breakdown;
};

using SnapshotMap = std::unordered_map<int, RiderSnapshot>;

struct FrameSnapshot {
  double sim_time =
      -1.0;            // -1 means unpopulated, makes sure we publish at time 0
  double sim_dt = 0.0; // seconds per physics step (constant)
  double time_factor = 1.0; // sim_speed / real_speed
  double real_time = 0.0;   // seconds (steady clock / SDL time) when captured

  SnapshotMap riders;
  GroupSnapshot groups;
};

#endif
