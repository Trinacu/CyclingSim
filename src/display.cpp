#include "display.h"
#include "SDL3/SDL_log.h"
#include "SDL3/SDL_rect.h"
#include "SDL3/SDL_render.h"
#include "pch.hpp"
#include "texturemanager.h"
#include "visualmodel.h"
#include <SDL3/SDL.h>
#include <SDL3_ttf/SDL_ttf.h>
#include <vector>

double effort_to_freq(double effort, double max_effort) {
  double anim_rpm;
  double min_rpm = 40.0;
  double max_rpm = 130.0;

  if (effort <= 1.0) {
    // 0 → 1  maps to 40 → 100
    anim_rpm = min_rpm + effort * (100.0 - min_rpm);
  } else {
    // 1 → max_effort maps to 100 → 130
    double t = (effort - 1.0) / (max_effort - 1.0);
    anim_rpm = 100.0 + t * (max_rpm - 100.0);
  }
  return anim_rpm / 60;
}

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

// maybe move into helpers or sth?
Vector2d rotate(Vec2 p, double a) {
  return {float(p.x * cos(a) - p.y * sin(a)),
          float(p.x * sin(a) + p.y * cos(a))};
}

void RiderDrawable::render(const RenderContext* ctx) {
  if (ctx->curr_frame->riders.empty() || ctx->prev_frame->riders.empty())
    return;

  auto cam = ctx->camera_weak.lock();
  if (!cam)
    return;

  const auto& prev = ctx->prev_frame->riders;
  const auto& curr = ctx->curr_frame->riders;

  // for (const auto& [id, snap] : *ctx->rider_snapshots) {
  for (auto& [id, s1] : curr) {
    // auto it0 = prev.find(id);
    // if (it0 == prev.end())
    //   continue; // new rider appeared
    // const RiderSnapshot& s0 = it0->second;

    Vector2d pos2d = ctx->view.rider_pos.at(id);
    double slope = ctx->view.rider_slope.at(id);
    double effort = ctx->view.rider_effort.at(id);

    // ---------------------------
    // Resolve visual model
    // ---------------------------
    const RiderVisualModel& model = resolve_visual_model(s1.visual_type);

    auto [it, inserted] = visuals.try_emplace(id);
    SDL_Log("id: %d", id);
    RiderVisualState& vis = it->second;
    SDL_Log("id=%zu inserted=%d vis_ptr=%p last=%.6f interp=%.6f", (size_t)id,
            inserted ? 1 : 0, (void*)&vis, vis.last_anim_sim_time,
            ctx->view.interp_sim_time);

    if (inserted) {
      vis.last_anim_sim_time = ctx->view.interp_sim_time;
      vis.anim_phase = 0.0;
      vis.wheel_angle = 0.0;
    }

    // ---------------------------
    // Camera & orientation
    // ---------------------------
    Vector2d front_ground_world = pos2d;
    Vector2d front_ground_screen = cam->world_to_screen(front_ground_world);

    double tilt_rad = std::atan(slope);
    double tilt_deg = tilt_rad * 180.0 / M_PI;

    // ---------------------------
    // Rotate offsets in world space
    // ---------------------------
    Vector2d front_wheel_world =
        front_ground_world + rotate(model.front_wheel_offset, tilt_rad);

    Vector2d rear_wheel_world =
        front_ground_world + rotate(model.rear_wheel_offset, tilt_rad);

    Vector2d body_world =
        front_ground_world + rotate(model.body_offset, tilt_rad);

    // ---------------------------
    // World → screen
    // ---------------------------
    Vector2d front_wheel_screen = cam->world_to_screen(front_wheel_world);

    Vector2d rear_wheel_screen = cam->world_to_screen(rear_wheel_world);

    Vector2d body_screen = cam->world_to_screen(body_world);

    // ---------------------------
    // Distance-based wheel rotation
    // ---------------------------
    if (std::isnan(vis.last_pos))
      vis.last_pos = pos2d.x();

    double ds = pos2d.x() - vis.last_pos;
    vis.last_pos = pos2d.x();

    vis.wheel_angle += ds / model.wheel_radius;

    // ---------------------------
    // Draw wheels
    // ---------------------------
    SDL_Texture* front_wheel_tex =
        ctx->resources->get_textureManager()->get_texture("wheel_front");
    SDL_Texture* rear_wheel_tex =
        ctx->resources->get_textureManager()->get_texture("wheel_rear");

    float wheel_screen_diameter = cam->get_scale() * 2.0 * model.wheel_radius;

    SDL_FRect front_wheel_dst{
        float(front_wheel_screen.x() - wheel_screen_diameter * 0.5),
        float(front_wheel_screen.y() - wheel_screen_diameter * 0.5),
        wheel_screen_diameter, wheel_screen_diameter};

    SDL_FRect rear_wheel_dst{
        float(rear_wheel_screen.x() - wheel_screen_diameter * 0.5),
        float(rear_wheel_screen.y() - wheel_screen_diameter * 0.5),
        wheel_screen_diameter, wheel_screen_diameter};

    SDL_FPoint wheel_center{wheel_screen_diameter * 0.5f,
                            wheel_screen_diameter * 0.5f};

    double wheel_angle_deg = vis.wheel_angle * 180.0 / M_PI + tilt_deg;

    // ---------------------------
    // Draw rider / bike texture
    // ---------------------------
    SDL_Texture* rider_front_tex =
        ctx->resources->get_textureManager()->get_texture("rider_front");
    SDL_Texture* rider_back_tex =
        ctx->resources->get_textureManager()->get_texture("rider_back");

    // float rider_screen_size =
    //     cam->get_scale() * model.wheelbase * 512.0f / 330.0f;

    float wheel_radius_px = 112.5;

    float scale_from_radius =
        cam->get_scale() * model.wheel_radius / model.wheel_radius_px;

    float rider_screen_size = scale_from_radius * 512.0f;

    // texture-space anchor → screen pixels
    float anchor_x = model.front_ground_point.x * rider_screen_size;
    float anchor_y = model.front_ground_point.y * rider_screen_size;

    SDL_FRect rider_dst{float(front_ground_screen.x() - anchor_x),
                        float(front_ground_screen.y() - anchor_y),
                        rider_screen_size, rider_screen_size};

    SDL_FPoint rider_pivot{anchor_x, anchor_y};

    double prev_sim_time = ctx->prev_frame->sim_time;
    double curr_sim_time = ctx->curr_frame->sim_time;

    if (std::isnan(vis.last_anim_sim_time))
      vis.last_anim_sim_time = ctx->view.interp_sim_time;

    double dt = ctx->view.interp_sim_time - vis.last_anim_sim_time;

    if (dt > 0.0) {
      double hz = effort_to_freq(effort, s1.max_effort);
      vis.anim_phase = std::fmod(vis.anim_phase + hz * dt, 1.0);
      vis.last_anim_sim_time = ctx->view.interp_sim_time;
    }

    constexpr int COLS = 6;
    constexpr int ROWS = 4;
    constexpr int TOTAL_FRAMES = COLS * ROWS;

    int idx = static_cast<int>(vis.anim_phase * TOTAL_FRAMES);
    idx = std::clamp(idx, 0, TOTAL_FRAMES - 1);

    int row = idx / COLS;
    int col = idx % COLS;

    SDL_Log("%.6f", dt);
    SDL_Log("anim_phase, idx, row, col");
    SDL_Log("%.2f %d: %d, %d", vis.anim_phase, idx, row, col);

    SDL_FRect src{static_cast<float>(col * 512), static_cast<float>(row * 512),
                  512, 512};

    // draw the back side first, because wheels appear on top of it
    SDL_RenderTextureRotated(ctx->renderer, rider_back_tex, &src, &rider_dst,
                             -tilt_deg, &rider_pivot, SDL_FLIP_NONE);

    SDL_RenderTextureRotated(ctx->renderer, rear_wheel_tex, nullptr,
                             &rear_wheel_dst, wheel_angle_deg, &wheel_center,
                             SDL_FLIP_NONE);

    SDL_RenderTextureRotated(ctx->renderer, front_wheel_tex, nullptr,
                             &front_wheel_dst, wheel_angle_deg, &wheel_center,
                             SDL_FLIP_NONE);

    SDL_RenderTextureRotated(ctx->renderer, rider_front_tex, &src, &rider_dst,
                             -tilt_deg, &rider_pivot, SDL_FLIP_NONE);

    // show the anchor point
    SDL_SetRenderDrawColor(ctx->renderer, 255, 0, 0, 255);
    SDL_FRect anchor_rect = SDL_FRect{(float)front_ground_screen.x() - 2,
                                      (float)front_ground_screen.y() - 2, 5, 5};
    SDL_RenderRect(ctx->renderer, &anchor_rect);
  }
}
