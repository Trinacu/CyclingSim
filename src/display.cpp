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

// Draws a thin coloured rectangle outline around a screen rect.
// Used as a group membership indicator behind the rider sprite.
static void draw_group_halo(SDL_Renderer* r, SDL_FRect rect, SDL_FColor col,
                            float expand = 4.0f) {
  SDL_FRect halo{
      rect.x - expand,
      rect.y - expand,
      rect.w + 2.0f * expand,
      rect.h + 2.0f * expand,
  };
  SDL_SetRenderDrawColorFloat(r, col.r, col.g, col.b, col.a * 0.7f);
  SDL_RenderRect(r, &halo);
}

CourseDrawable::CourseDrawable(const Course* course_) : course(course_) {}

void CourseDrawable::render(const RenderContext* ctx) {
  const std::vector<CoursePoint>& world_pts = course->points;

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
    if (abs(wp.x - camera->get_pos().x()) > 2000)
      continue;

    Vector2d sp = camera->world_to_screen(wp.vec2());
    screen_points.push_back(SDL_FPoint{(float)sp.x(), (float)sp.y()});
  }

  SDL_SetRenderDrawColor(ctx->renderer, 200, 200, 200, 255);
  SDL_RenderLines(ctx->renderer, screen_points.data(), screen_points.size());
};

// maybe move into helpers or sth?
Vector2d rotate(Vec2 p, double a) {
  return {float(p.x * cos(a) - p.y * sin(a)),
          float(p.x * sin(a) + p.y * cos(a))};
}

void RiderDrawable::render(const RenderContext* ctx) {
  if (ctx->riders.empty())
    return;

  auto cam = ctx->camera_weak.lock();
  if (!cam)
    return;

  const Vector2d cam_pos = cam->get_pos();

  // TODO - sort by lat_pos descending so we draw correct order
  std::vector<std::pair<int, RiderRenderState>> sorted_riders(
      ctx->riders.begin(), ctx->riders.end());

  // Sort the vector by lat_pos in descending order
  std::sort(sorted_riders.begin(), sorted_riders.end(),
            [](const auto& a, const auto& b) {
              return a.second.lat_pos > b.second.lat_pos; // > for descending
            });

  for (const auto& [id, rs] : sorted_riders) {
    const RiderVisualModel& model = resolve_visual_model(rs.visual_type);

    double dist = rs.pos - cam_pos[0];
    double half_world_w = cam->get_world_width() / 2;
    double bike_len = model.wheelbase + 2 * model.wheel_radius;
    if ((dist > (half_world_w + bike_len) || (dist < -half_world_w))) {
      continue;
    }

    auto [it, inserted] = visuals.try_emplace(id);
    RiderVisualState& vis = it->second;

    if (inserted) {
      vis.last_anim_sim_time = ctx->interp_sim_time;
      vis.anim_phase = 0.0;
      vis.wheel_angle = 0.0;
    }

    // Wheel rotation from distance travelled
    if (std::isnan(vis.last_pos))
      vis.last_pos = rs.pos2d.x();
    vis.wheel_angle += (rs.pos2d.x() - vis.last_pos) / model.wheel_radius;
    vis.last_pos = rs.pos2d.x();

    update_animation(vis, ctx->interp_sim_time, rs.effort, rs.max_effort);

    const RiderScreenGeom geom = compute_screen_geom(
        *cam, rs.pos2d, rs.slope, rs.lat_pos, model, vis.wheel_angle);

    // Group membership indicator — drawn behind the sprite
    if (rs.group_id != kNoGroup) {
      const float wheel_diam = cam->get_scale() * 2.0f * model.wheel_radius;
      const SDL_FRect rider_rect{
          float(geom.rear_wheel_screen.x()) - 4.0f,
          float(geom.front_wheel_screen.y()) - wheel_diam,
          float(geom.front_wheel_screen.x() - geom.rear_wheel_screen.x()) +
              8.0f,
          wheel_diam * 2.5f,
      };
      draw_group_halo(ctx->renderer, rider_rect, group_colour(rs.group_id));
    }

    draw_rider(ctx, model, vis, geom, *cam);
  }
}

RiderDrawable::RiderScreenGeom RiderDrawable::compute_screen_geom(
    const Camera& cam, const Vector2d& pos2d, double slope, double lat_pos,
    const RiderVisualModel& model, double wheel_angle) const {

  const double tilt_rad = std::atan(slope);

  Vector2d front_wheel_world =
      pos2d + rotate(model.front_wheel_offset, tilt_rad);
  Vector2d rear_wheel_world = pos2d + rotate(model.rear_wheel_offset, tilt_rad);

  Vector2d fg = cam.world_to_screen(pos2d);
  Vector2d fw = cam.world_to_screen(front_wheel_world);
  Vector2d rw = cam.world_to_screen(rear_wheel_world);
  const double lat_offset_px = lat_pos * kLatPxPerM * cam.get_scale();
  fg.y() -= lat_offset_px;
  fw.y() -= lat_offset_px;
  rw.y() -= lat_offset_px;

  return RiderScreenGeom{
      .front_ground_screen = fg,
      .front_wheel_screen = fw,
      .rear_wheel_screen = rw,
      .tilt_deg = tilt_rad * 180.0 / M_PI,
      .wheel_angle_deg = wheel_angle * 180.0 / M_PI + (tilt_rad * 180.0 / M_PI),
  };
}

void RiderDrawable::update_animation(RiderVisualState& vis,
                                     double interp_sim_time, double effort,
                                     double max_effort) {
  if (std::isnan(vis.last_anim_sim_time))
    vis.last_anim_sim_time = interp_sim_time;

  const double dt = interp_sim_time - vis.last_anim_sim_time;
  if (dt > 0.0) {
    const double hz = effort_to_freq(effort, max_effort);
    vis.anim_phase = std::fmod(vis.anim_phase + hz * dt, 1.0);
    vis.last_anim_sim_time = interp_sim_time;
  }
}

void RiderDrawable::draw_rider(const RenderContext* ctx,
                               const RiderVisualModel& model,
                               const RiderVisualState& vis,
                               const RiderScreenGeom& geom,
                               const Camera& cam) const {
  auto* tex_mgr = ctx->resources->get_textureManager();

  // --- Sprite sheet frame ---
  constexpr int COLS = 6, ROWS = 4, TOTAL_FRAMES = COLS * ROWS;
  const int idx = std::clamp(static_cast<int>(vis.anim_phase * TOTAL_FRAMES), 0,
                             TOTAL_FRAMES - 1);
  const SDL_FRect src{static_cast<float>((idx % COLS) * 512),
                      static_cast<float>((idx / COLS) * 512), 512.f, 512.f};

  // --- Rider dst rect ---
  const float scale =
      cam.get_scale() * model.wheel_radius / model.wheel_radius_px;
  const float size_px = scale * 512.f;
  const float ax = model.front_ground_point.x * size_px;
  const float ay = model.front_ground_point.y * size_px;

  const SDL_FRect rider_dst{float(geom.front_ground_screen.x()) - ax,
                            float(geom.front_ground_screen.y()) - ay, size_px,
                            size_px};
  const SDL_FPoint rider_pivot{ax, ay};

  // --- Wheel dst rects ---
  const float wheel_diam = cam.get_scale() * 2.f * model.wheel_radius;
  const SDL_FPoint wheel_center{wheel_diam * .5f, wheel_diam * .5f};

  auto wheel_rect = [&](Vector2d screen_pos) -> SDL_FRect {
    return {float(screen_pos.x()) - wheel_diam * .5f,
            float(screen_pos.y()) - wheel_diam * .5f, wheel_diam, wheel_diam};
  };

  SDL_FRect front_wheel_dst = wheel_rect(geom.front_wheel_screen);
  SDL_FRect rear_wheel_dst = wheel_rect(geom.rear_wheel_screen);

  // --- Draw: back layer → wheels → front layer ---
  SDL_RenderTextureRotated(ctx->renderer, tex_mgr->get_texture("rider_back"),
                           &src, &rider_dst, -geom.tilt_deg, &rider_pivot,
                           SDL_FLIP_NONE);

  SDL_RenderTextureRotated(ctx->renderer, tex_mgr->get_texture("wheel_rear"),
                           nullptr, &rear_wheel_dst, geom.wheel_angle_deg,
                           &wheel_center, SDL_FLIP_NONE);

  SDL_RenderTextureRotated(ctx->renderer, tex_mgr->get_texture("wheel_front"),
                           nullptr, &front_wheel_dst, geom.wheel_angle_deg,
                           &wheel_center, SDL_FLIP_NONE);

  SDL_RenderTextureRotated(ctx->renderer, tex_mgr->get_texture("rider_front"),
                           &src, &rider_dst, -geom.tilt_deg, &rider_pivot,
                           SDL_FLIP_NONE);
}
