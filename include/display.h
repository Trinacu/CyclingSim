// display.h
#ifndef DISPLAY_H
#define DISPLAY_H

#include "appstate.h"
#include "camera.h"
#include "course.h"
#include "pch.hpp"
#include "texturemanager.h"
#include <memory>

#include <SDL3/SDL.h>

// class Camera {
// private:
//   const Course* course;
//   int world_width;
//   double scale;
//   double vert_scale;
//   Vector2d pos;
//
//   size_t target_rider_uid;
//
// public:
//   // screensize could be private?
//   Vector2d screensize;
//   Camera(const Course* course_, int world_width_, Vector2d screensize_);
//
//   void follow_course(double x);
//   void update(double x);
//   void _set_center(Vector2d new_pos);
//
//   Vector2d get_pos() { return pos; }
//
//   Vector2d world_to_screen(Vector2d world_pos) const;
//   MatrixX2d world_to_screen(MatrixX2d world_pos_list) const;
//
//   Vector2d screen_to_world(Vector2d world_pos) const;
//   MatrixX2d screen_to_world(MatrixX2d world_pos_list) const;
//
//   MatrixX2d get_visible_points() const;
// };

class DisplayEngine;

struct RenderContext {
  SDL_Renderer* renderer;
  std::weak_ptr<Camera> camera_weak;
  const SnapshotMap* rider_snapshots;
  ResourceProvider* resources;

  // DisplayEngine* engine;

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

class DisplayEngine {
private:
  AppState* app; // Reference to the app state (renderer, resources)
  Camera* camera;

  // Drawables specific to this "scene"
  std::vector<std::unique_ptr<Drawable>> drawables;

  size_t camera_target_id = 0;
  const double target_fps = 60.0;

  std::vector<Vector2d> rider_positions;
  const std::chrono::duration<double> target_frame_duration =
      std::chrono::duration<double>(1.0 / target_fps);
  std::vector<RiderSnapshot> get_rider_snapshots();
  SnapshotMap get_rider_snapshot_map();

public:
  DisplayEngine(AppState* app_, Vector2d screensize, int WORLD_WIDTH);
  ~DisplayEngine(); // No longer destroys SDL
  void add_drawable(std::unique_ptr<Drawable> d);
  void render_frame();
  bool handle_event(const SDL_Event* event);

  void set_target_id(size_t id) { camera_target_id = id; }
  size_t get_target_id() const { return camera_target_id; }

  void handle_click(double screen_x, double screen_y);

  Camera* get_camera() { return camera; }
};
#endif
