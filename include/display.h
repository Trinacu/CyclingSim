// display.h
#ifndef DISPLAY_H
#define DISPLAY_H

#include "camera.h"
#include "course.h"
#include "snapshot.h"
#include "texturemanager.h"
#include "visualmodel.h"
#include <memory>

#include <SDL3/SDL.h>
#include <unordered_map>

// determines px offset per meter of lateral offset
static constexpr double kLatPxPerM = 20.0;

struct RenderContext {
  SDL_Renderer* renderer;
  std::weak_ptr<Camera> camera_weak;
  ResourceProvider* resources;

  double sim_time = 0.0;
  double time_factor = 1.0;
  double alpha = 1.0;
  double interp_sim_time = 0.0; // for animation sim timing
                                //
  std::unordered_map<int, RiderRenderState> riders;
};

enum class RenderLayer : int { Course = 0, Riders = 1, UI = 2, COUNT };

class Drawable {
public:
  virtual RenderLayer layer() const = 0;
  // Called each frame; the implementer draws itself with the given SDL_Renderer
  virtual void render(const RenderContext* ctx) = 0;
  virtual bool handle_event(const SDL_Event* e) { return false; }
  virtual ~Drawable() = default;
  virtual void render_imgui(const RenderContext* ctx) {} // NEW: default empty
};

class CourseDrawable : public Drawable {
private:
  const Course* course;

public:
  CourseDrawable(const Course* course_);
  RenderLayer layer() const override { return RenderLayer::Course; }

  void render(const RenderContext* ctx) override;
};

struct RiderVisualState {
  double wheel_angle = 0.0;                         // radians
  double anim_phase = std::rand() * 1.0 / RAND_MAX; // 0..1 for sprite animation

  double last_pos = std::numeric_limits<double>::quiet_NaN();
  double last_anim_sim_time = std::numeric_limits<double>::quiet_NaN();
};

class RiderDrawable : public Drawable {
  std::unordered_map<int, RiderVisualState> visuals;
  const RiderVisualModel& model = ROAD_BIKE_VISUAL;

public:
  RiderDrawable() = default;
  RenderLayer layer() const override { return RenderLayer::Riders; }

  void render(const RenderContext* ctx) override;

private:
  struct RiderScreenGeom {
    Vector2d front_ground_screen;
    Vector2d front_wheel_screen;
    Vector2d rear_wheel_screen;
    double tilt_deg;
    double wheel_angle_deg;
  };

  RiderScreenGeom compute_screen_geom(const Camera& cam, const Vector2d& pos2d,
                                      double slope, double lat_pos,
                                      const RiderVisualModel& model,
                                      double wheel_angle) const;

  void update_animation(RiderVisualState& vis, double interp_sim_time,
                        double effort, double max_effort);

  void draw_rider(const RenderContext* ctx, const RiderVisualModel& model,
                  const RiderVisualState& vis, const RiderScreenGeom& geom,
                  const Camera& cam) const;
};

#endif
