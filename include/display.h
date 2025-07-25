// display.h
#ifndef DISPLAY_H
#define DISPLAY_H

#include "course.h"
#include "sim.h"
#include "texturemanager.h"
#include <memory>

#include <SDL3/SDL.h>

class Camera {
  private:
    Course* course;
    int world_width;
    double scale;
    double vert_scale;
    Vector2d pos;
    const Rider* target_rider = nullptr;

  public:
    // screensize could be private?
    Vector2d screensize;
    Camera(Course* course_, int world_width_, Vector2d screensize_);

    void follow_course(double x);
    void update(double x);
    void set_target_rider(const Rider* rider) { target_rider = rider; }
    void _set_pos(Vector2d pos_);

    Vector2d world_to_screen(Vector2d world_pos) const;
    MatrixX2d world_to_screen(MatrixX2d world_pos_list) const;

    Vector2d screen_to_world(Vector2d world_pos) const;
    MatrixX2d screen_to_world(MatrixX2d world_pos_list) const;

    MatrixX2d get_visible_points() const;
};

struct RenderContext {
    SDL_Renderer* renderer;
    Camera* camera;
    const std::vector<RiderSnapshot>* rider_snapshots;
    ResourceProvider* resources;

    const RiderSnapshot* get_snapshot(const void* id) const {
        for (const auto& snap : *rider_snapshots) {
            if (snap.id == id) {
                return &snap;
            }
        }
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
    SDL_Window* window = nullptr;
    SDL_Renderer* renderer = nullptr;
    ResourceProvider* resources = nullptr;
    int width, height;
    Camera* camera;
    const Simulation* sim; // holds the physics engine
    std::vector<std::unique_ptr<Drawable>> drawables;

    // snapshots positions so we can let go of frame_lock for PhysicsEngine
    std::vector<Vector2d> rider_positions;

    std::chrono::steady_clock::time_point last_frame_time;
    const double target_fps = 60.0;
    const std::chrono::duration<double> target_frame_duration =
        std::chrono::duration<double>(1.0 / target_fps);
    std::vector<RiderSnapshot> get_rider_snapshots();

  public:
    DisplayEngine(Simulation* s, int w, int h, Camera* camera_);
    SDL_Texture* img1;
    SDL_Texture* img2;

    bool load_image(const char* id, const char* filename);

    ~DisplayEngine() {
        SDL_DestroyRenderer(renderer);
        SDL_DestroyWindow(window);
        SDL_Quit();
    }

    void set_resources(ResourceProvider* resources);
    ResourceProvider* get_resources() const;

    bool handle_event(const SDL_Event* event);
    void add_drawable(std::unique_ptr<Drawable> d);
    void render_frame();
    SDL_Renderer* get_renderer();
    Camera* get_camera();
};
#endif
