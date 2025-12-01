// display.h
#ifndef DISPLAY_H
#define DISPLAY_H

#include "camera.h"
#include "course.h"
#include "texturemanager.h"
#include <memory>

#include <SDL3/SDL.h>

struct RenderContext {
  SDL_Renderer* renderer;
  std::weak_ptr<Camera> camera_weak;
  const SnapshotMap* rider_snapshots;
  ResourceProvider* resources;

  const RiderSnapshot* get_snapshot(const size_t id) const {
    auto it = rider_snapshots->find(id);
    if (it != rider_snapshots->end())
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

class RiderDrawable : public Drawable {
public:
  RiderDrawable() = default;
  RenderLayer layer() const override { return RenderLayer::Riders; }

  void render(const RenderContext* ctx) override;
};

#endif
