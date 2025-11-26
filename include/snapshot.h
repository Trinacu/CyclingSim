#ifndef SNAPSHOT_H
#define SNAPSHOT_H

#include "pch.hpp"
#include <string>
#include <unordered_map>

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
  int team_id;
};

using SnapshotMap = std::unordered_map<int, RiderSnapshot>;

#endif
