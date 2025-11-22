#include "display.h"
#include "SDL3/SDL_rect.h"
#include "pch.hpp"
#include "texturemanager.h"
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <mutex>
#include <thread>
#include <unordered_map>
#include <vector>

Camera::Camera(const Course* course_, int world_width_, Vector2d screensize_)
    : course(course_), world_width(world_width_), screensize(screensize_) {
  scale = screensize_[0] / (double)world_width_;
  vert_scale = 1.0;
  pos = {0.0, 0.0};
}

void Camera::follow_course(double x) { pos << x, course->get_altitude(x); }

void Camera::update(double x) {
  // if (target_rider) {
  //   pos = target_rider->get_pos2d();
  // } else {
  //   follow_course(x);
  // }
}

void Camera::_set_center(Vector2d new_pos) { this->pos = new_pos; }

Vector2d Camera::world_to_screen(Vector2d world_pos) const {
  // offset from camera pos, multiply by scale, invert y axis
  // and offset (0, 0) to the center of the screen
  return ((world_pos - pos) * scale).cwiseProduct(Vector2d(1, -vert_scale)) +
         Vector2d(screensize[0] / 2, screensize[1] / 2);
}

MatrixX2d Camera::world_to_screen(MatrixX2d world_pts) const {
  world_pts.rowwise() -= pos;
  world_pts *= scale;
  world_pts.col(1) *= -vert_scale;
  world_pts.rowwise() += screensize * 0.5;
  return world_pts;
}

MatrixX2d Camera::get_visible_points() const {
  double half_w = (double)world_width * 0.5;
  MatrixX2d pts = course->get_points(pos[0] + half_w, pos[0] - half_w);
  return world_to_screen(pts);
}

CourseDrawable::CourseDrawable(const Course* course_) : course(course_) {}

void CourseDrawable::render(const RenderContext* ctx) {
  const std::vector<Vector2d>& world_pts = course->visual_points;

  if (world_pts.empty())
    return;

  std::vector<SDL_FPoint> screen_points;
  screen_points.reserve(world_pts.size());

  // this loop can be optimized but apparently for <10k points, its instant
  for (const auto& wp : world_pts) {
    if (abs(wp.x() - ctx->camera->get_pos().x()) > 2000)
      continue;

    Vector2d sp = ctx->camera->world_to_screen(wp);
    screen_points.push_back(SDL_FPoint{(float)sp.x(), (float)sp.y()});
  }
  // MatrixX2d pts = ctx->camera->get_visible_points();
  // float center_x = ctx->camera->screensize[0] * 0.5f;
  // float center_y = ctx->camera->screensize[1] * 0.5f;
  // // maybe i could use vector here?
  // int n = (int)pts.rows();
  // SDL_FPoint* points = new SDL_FPoint[n];
  // for (int i = 0; i < n; ++i) {
  //   points[i].x = pts(i, 0);
  //   points[i].y = pts(i, 1); // Construct pair from row data
  // }

  SDL_SetRenderDrawColor(ctx->renderer, 200, 200, 200, 255);
  // SDL_RenderLines(ctx->renderer, points, pts.rows());
  // delete[] points;
  SDL_RenderLines(ctx->renderer, screen_points.data(), screen_points.size());
  // SDL_SetRenderDrawColor(ctx->renderer, 200, 0, 0, 10);
  // SDL_FRect r{center_x - 2.0f, center_y - 2.0f, 5.0f, 5.0f};
  // SDL_RenderFillRect(ctx->renderer, &r);
};

void RiderDrawable::render(const RenderContext* ctx) {
  if (ctx->rider_snapshots->size() == 0) {
    throw std::runtime_error(
        "rider snapshots are empty (RiderDrawable::render)");
  }

  SDL_SetRenderDrawColor(ctx->renderer, 0, 255, 0, 255);
  // Draw each rider as a small 6×6 filled rect, centered on its screen‐space
  // pos
  for (const auto& [id, rider_snapshot] : *ctx->rider_snapshots) {
    Vector2d screen_pos = ctx->camera->world_to_screen(rider_snapshot.pos2d);
    float w = 128;
    float h = 128;
    float x = static_cast<float>(screen_pos.x()) - w;
    float y = static_cast<float>(screen_pos.y()) - h;
    const SDL_FRect dst = SDL_FRect{x, y, w, h};
    const SDL_FRect src = SDL_FRect{0, 0, 256, 256};
    const SDL_Texture* tex =
        ctx->resources->get_textureManager()->get_texture("player");
    if (!tex) {
      SDL_Log("Missing texture 'player' while rendering tider %ld",
              (long)rider_snapshot.uid);
      continue;
    }
    SDL_RenderTexture(ctx->renderer, const_cast<SDL_Texture*>(tex), &src, &dst);
    SDL_FRect r{x - 3.0f, y - 3.0f, 6.0f, 6.0f};
    SDL_RenderFillRect(ctx->renderer, &r);
  }
}

DisplayEngine::DisplayEngine(Simulation* s, int w, int h, Camera* camera_)
    : sim(s), width(w), height(h), camera(camera_) {
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
  }
  if (!TTF_Init()) {
    SDL_Log("Couldn't initialize TTF: %s", SDL_GetError());
  }
  SDL_CreateWindowAndRenderer("Physics Sim", width, height, 0, &window,
                              &renderer);
}

void DisplayEngine::set_resources(ResourceProvider* resources_) {
  resources = resources_;
}

ResourceProvider* DisplayEngine::get_resources() const {
  if (!resources) {
    SDL_Log("get_resources() called but resources == nullptr!");
  }
  return resources;
}

void DisplayEngine::add_drawable(std::unique_ptr<Drawable> d) {
  drawables.emplace_back(std::move(d));
}

std::vector<RiderSnapshot> DisplayEngine::get_rider_snapshots() {
  std::vector<RiderSnapshot> result;

  std::lock_guard<std::mutex> lock(*sim->get_engine()->get_frame_mutex());
  const std::vector<Rider*>& riders =
      sim->get_engine()->get_riders(); // assuming a getter

  result.reserve(riders.size());
  for (const Rider* rider_ptr : riders) {
    result.push_back(rider_ptr->snapshot()); // copy out minimal state
  }
  return result; // rendering can now proceed without holding any locks
}

SnapshotMap DisplayEngine::get_rider_snapshot_map() {
  SnapshotMap result;

  // Lock is essential!
  std::lock_guard<std::mutex> lock(*sim->get_engine()->get_frame_mutex());

  const std::vector<Rider*>& riders = sim->get_engine()->get_riders();

  result.reserve(riders.size());

  for (const Rider* rider_ptr : riders) {
    // key = id; value = snapshot
    result.emplace(rider_ptr->get_id(), rider_ptr->snapshot());
  }
  return result;
}

bool DisplayEngine::load_image(const char* id, const char* filename) {
  return resources->get_textureManager()->load_texture(id, filename);
}

void DisplayEngine::render_frame() {
  auto start = std::chrono::steady_clock::now();

  SnapshotMap snapshots = get_rider_snapshot_map();

  if (snapshots.count(camera_target_id)) {
    camera->_set_center(snapshots.at(camera_target_id).pos2d);
  }

  SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
  SDL_RenderClear(renderer);

  // Build a RenderContext that hands each Drawable a pointer to our snapshot
  RenderContext ctx{renderer, camera, &snapshots, resources};

  // Render each Drawable in order (CourseDrawable, RiderDrawable, etc.)
  for (auto& d : drawables) {
    d->render(&ctx);
  }

  // Finally present the composed frame
  SDL_RenderPresent(renderer);

  auto end = std::chrono::steady_clock::now();
  std::chrono::duration<double> elapsed = end - start;
  if (elapsed < target_frame_duration) {
    std::this_thread::sleep_for(target_frame_duration - elapsed);
  }

  last_frame_time = std::chrono::steady_clock::now();
}

// what does this do?
bool DisplayEngine::handle_event(const SDL_Event* e) {
  for (auto it = drawables.rbegin(); it != drawables.rend(); ++it) {
    if ((*it)->handle_event(e)) {
      return true; // Event was consumed
    }
  }
  return false;
}

SDL_Renderer* DisplayEngine::get_renderer() { return renderer; }
Camera* DisplayEngine::get_camera() { return camera; }
