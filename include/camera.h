#ifndef CAMERA_H
#define CAMERA_H

#include "course.h"
#include "snapshot.h"
#include <optional>

class Camera {
public:
  Camera(const Course* course, int world_width, Vector2d screensize);

  // --- Transformations ---
  Vector2d world_to_screen(Vector2d world) const;
  MatrixX2d world_to_screen(MatrixX2d world) const;
  Vector2d screen_to_world(Vector2d screen) const;

  // --- Follow logic ---
  void set_target_id(int rider_uid);
  void clear_target();
  bool has_target() const { return target_uid.has_value(); }

  // Update camera position each frame
  void update(const SnapshotMap& snaps);

  // --- Manual controls ---
  void pan(double dx, double dy); // screen space delta
  void zoom(double amount);       // amount > 0 zooms in

  // Direct set (used rarely)
  void set_center(Vector2d p) { pos = p; }

  // Accessors
  Vector2d get_pos() const { return pos; }

private:
  const Course* course;
  const int world_width;
  Vector2d screensize;

  double scale;      // horizontal world→screen scale
  double vert_scale; // vertical world→screen scale

  Vector2d pos; // world coordinate camera center
  std::optional<int> target_uid;

  // smooth follow parameters
  double follow_strength = 0.4; // 0=no follow, 1=teleport
};

#endif
