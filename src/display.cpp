#include "display.h"
#include "SDL3/SDL_rect.h"
#include "pch.hpp"
#include "texturemanager.h"
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <vector>

CourseDrawable::CourseDrawable(const Course* course_) : course(course_) {}

void CourseDrawable::render(const RenderContext* ctx) {
  const std::vector<Vector2d>& world_pts = course->visual_points;

  if (world_pts.empty())
    return;

  std::vector<SDL_FPoint> screen_points;
  screen_points.reserve(world_pts.size());

  // this loop can be optimized but apparently for <10k points, its instant
  auto camera = ctx->camera_weak.lock();
  if (!camera) {
    SDL_Log("failed to lock camera weak_ptr");
    return;
  }
  for (const auto& wp : world_pts) {
    if (abs(wp.x() - camera->get_pos().x()) > 2000)
      continue;

    Vector2d sp = camera->world_to_screen(wp);
    screen_points.push_back(SDL_FPoint{(float)sp.x(), (float)sp.y()});
  }

  SDL_SetRenderDrawColor(ctx->renderer, 200, 200, 200, 255);
  // SDL_RenderLines(ctx->renderer, points, pts.rows());
  // delete[] points;
  SDL_RenderLines(ctx->renderer, screen_points.data(), screen_points.size());
};

void RiderDrawable::render(const RenderContext* ctx) {
  // if (ctx->rider_snapshots->size() == 0) {
  //   throw std::runtime_error(
  //       "rider snapshots are empty (RiderDrawable::render)");
  // }
  if (!ctx->rider_snapshots || ctx->rider_snapshots->empty()) {
    return;
  }

  SDL_SetRenderDrawColor(ctx->renderer, 0, 255, 0, 255);
  // Draw each rider as a small 6×6 filled rect, centered on its screen‐space
  // pos
  for (const auto& [id, rider_snapshot] : *ctx->rider_snapshots) {
    auto cam = ctx->camera_weak.lock();
    if (!cam) {
      SDL_Log("failed to lock camera weak_ptr");
      return;
    }
    Vector2d screen_pos = cam->world_to_screen(rider_snapshot.pos2d);
    // SDL_Log("%.1f %.1f", rider_snapshot.pos2d.x(), rider_snapshot.pos2d.y());
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
