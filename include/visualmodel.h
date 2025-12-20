#ifndef VISUALMODEL_H
#define VISUALMODEL_H

namespace {
constexpr double ROAD_WHEEL_RADIUS = 0.311;
constexpr double TT_WHEEL_RADIUS = 0.311;
constexpr double ROAD_WHEELBASE = 0.91;
constexpr double TT_WHEELBASE = 0.99;
constexpr double ROAD_WHEEL_RADIUS_PX = 112.5;
} // namespace

struct Vec2 {
  double x;
  double y;
};

enum class BikeType { Road, TT, Climbing };

struct RiderVisualModel {
  // all in WORLD units (meters)
  double wheel_radius;
  double wheelbase;

  // local offsets relative to rear wheel contact point
  Vec2 front_wheel_offset;
  Vec2 rear_wheel_offset;
  Vec2 body_offset;

  // sprite layout
  int body_frame_count;

  Vec2 front_ground_point;

  float wheel_radius_px;
};

// (0, 0) point is rider.pos
// it is the front edge of the tire at the ground level
inline constexpr RiderVisualModel ROAD_BIKE_VISUAL{
    .wheel_radius = ROAD_WHEEL_RADIUS,
    .wheelbase = ROAD_WHEELBASE,
    .front_wheel_offset = {-ROAD_WHEEL_RADIUS, ROAD_WHEEL_RADIUS},
    .rear_wheel_offset = {-ROAD_WHEELBASE - ROAD_WHEEL_RADIUS,
                          ROAD_WHEEL_RADIUS},
    .body_offset = {-ROAD_WHEEL_RADIUS, ROAD_WHEEL_RADIUS},
    // .body_offset = {0, ROAD_WHEEL_RADIUS},
    .body_frame_count = 24,
    // this is determined by pixels on the actual image
    // .front_wheel_axis = {430.0 / 512.0, 407.0 / 512.0}};
    // pos is wheel radius offset more in either direction (should be 112.5)
    .front_ground_point = {(430.0 + ROAD_WHEEL_RADIUS_PX) / 512.0,
                           (407.0 + ROAD_WHEEL_RADIUS_PX) / 512.0},
    .wheel_radius_px = ROAD_WHEEL_RADIUS_PX};

inline constexpr RiderVisualModel TT_BIKE_VISUAL{
    .wheel_radius = ROAD_WHEEL_RADIUS,
    .wheelbase = TT_WHEELBASE,
    .front_wheel_offset = {-ROAD_WHEEL_RADIUS, 2 * ROAD_WHEEL_RADIUS},
    .rear_wheel_offset = {-TT_WHEELBASE - TT_WHEEL_RADIUS, 2 * TT_WHEEL_RADIUS},
    .body_offset = {0.5, 0.9},
    .body_frame_count = 8,
    .front_ground_point = {430.0 / 512.0, 407.0 / 512.0}};

inline constexpr RiderVisualModel CLIMB_BIKE_VISUAL{
    .wheel_radius = ROAD_WHEEL_RADIUS,
    .wheelbase = ROAD_WHEELBASE,
    .front_wheel_offset = {-ROAD_WHEEL_RADIUS, 2 * ROAD_WHEEL_RADIUS},
    .rear_wheel_offset = {-ROAD_WHEELBASE - ROAD_WHEEL_RADIUS,
                          2 * ROAD_WHEEL_RADIUS},
    .body_offset = {0.5, 0.9},
    .body_frame_count = 8,
    .front_ground_point = {430.0 / 512.0, 407.0 / 512.0}};

inline const RiderVisualModel& resolve_visual_model(BikeType type) {
  switch (type) {
  case BikeType::TT:
    return TT_BIKE_VISUAL;
  case BikeType::Climbing:
    return CLIMB_BIKE_VISUAL;
  case BikeType::Road:
  default:
    return ROAD_BIKE_VISUAL;
  }
}

#endif
