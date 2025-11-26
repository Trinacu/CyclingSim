#include "camera.h"
#include "pch.hpp"

Camera::Camera(const Course* course_, int world_width_, Vector2d screensize_)
    : course(course_), world_width(world_width_), screensize(screensize_),
      scale(screensize_.x() / (double)world_width_), vert_scale(1.0),
      pos(0.0, 0.0) {}

// ------------------------
// FOLLOWING LOGIC
// ------------------------
void Camera::set_target(int rider_uid) { target_uid = rider_uid; }

void Camera::clear_target() { target_uid.reset(); }

void Camera::update(const SnapshotMap& snaps) {
  if (!target_uid.has_value())
    return;

  int id = *target_uid;
  auto it = snaps.find(id);
  if (it == snaps.end())
    return;

  const RiderSnapshot& snap = it->second;

  // Target position is rider position
  Vector2d target_pos = snap.pos2d;

  // Follow altitude using course
  target_pos.y() = course->get_altitude(target_pos.x());

  // Smooth interpolation toward target
  pos = pos + follow_strength * (target_pos - pos);
}

// ------------------------
// MANUAL CAMERA CONTROLS
// ------------------------
void Camera::pan(double dx, double dy) {
  // User panning breaks following
  clear_target();

  // convert screen dx to world dx
  Vector2d delta(dx / scale, dy / (-vert_scale));

  pos += delta;
}

void Camera::zoom(double amount) {
  // Simple zoom factor
  double factor = 1.0 + amount;

  scale *= factor;
  if (scale < 0.01)
    scale = 0.01;
  if (scale > 5.0)
    scale = 5.0;
}

// ------------------------
// TRANSFORMS
// ------------------------
Vector2d Camera::world_to_screen(Vector2d world) const {
  return ((world - pos) * scale).cwiseProduct(Vector2d(1, -vert_scale)) +
         screensize * 0.5;
}

MatrixX2d Camera::world_to_screen(MatrixX2d w) const {
  MatrixX2d r = w;

  r.rowwise() -= pos;
  r *= scale;
  r.col(1) *= -vert_scale;
  r.rowwise() += screensize * 0.5;

  return r;
}

Vector2d Camera::screen_to_world(Vector2d s) const {
  Vector2d centered = s - screensize * 0.5;

  Vector2d scaled;
  scaled.x() = centered.x() / scale;
  scaled.y() = centered.y() / (-vert_scale);

  return scaled + pos;
}
