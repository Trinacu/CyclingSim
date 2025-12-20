#ifndef SNAPSHOT_H
#define SNAPSHOT_H

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

struct RiderSnapshot {
  const size_t uid;
  std::string name;
  double pos;
  double slope;
  Vector2d pos2d;
  double power;
  double effort;
  double max_effort;
  double speed;
  double km_h;
  double heading;
  int team_id;

  BikeType visual_type;

  std::array<double, (int)PowerTerm::COUNT> power_breakdown;
};

using SnapshotMap = std::unordered_map<int, RiderSnapshot>;

#endif
