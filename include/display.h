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

struct RenderContext {
  SDL_Renderer* renderer;
  std::weak_ptr<Camera> camera_weak;
  ResourceProvider* resources;
  double sim_time;

  const FrameSnapshot* prev_frame = nullptr;
  const FrameSnapshot* curr_frame = nullptr;

  double alpha = 1.0; // 0 ... 1

  const RiderSnapshot* get_snapshot(const size_t id) const {
    auto it = curr_frame->riders.find(id);
    if (it != curr_frame->riders.end())
      return &it->second;
    return nullptr;
  }
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
  double wheel_angle = 0.0; // radians
  double last_pos = 0.0;    // for calculating wheel angle
  double anim_phase = 0.0;  // 0..1 for sprite animation

  // timing for spritesheet animation
  double last_sim_time = 0;
};

class RiderDrawable : public Drawable {
  std::unordered_map<size_t, RiderVisualState> visuals;
  const RiderVisualModel& model = ROAD_BIKE_VISUAL;

public:
  RiderDrawable() = default;
  RenderLayer layer() const override { return RenderLayer::Riders; }

  void render(const RenderContext* ctx) override;
};

#endif
