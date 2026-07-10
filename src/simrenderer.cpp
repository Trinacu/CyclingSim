#include "simrenderer.h"
#include "camera.h"
#include "display.h"
#include "sim.h"
#include "snapshot.h"
#include <limits>
#include <memory>

SimulationRenderer::SimulationRenderer(SDL_Renderer* r,
                                       GameResources* resources,
                                       Simulation* sim_,
                                       std::shared_ptr<Camera> cam)
    : CoreRenderer(r, resources), sim(sim_), camera(std::move(cam)) {
  build_and_swap_snapshots();
  // this could probably be removed
  if (frame_curr.riders.count(0)) {
    camera->set_center(frame_curr.riders.at(0).pos2d);
  }
}

void SimulationRenderer::reset() {}

void SimulationRenderer::add_world_drawable(std::unique_ptr<Drawable> d) {
  world_drawables.push_back(std::move(d));
}

void SimulationRenderer::set_ui_root(std::unique_ptr<UIRoot> root) {
  ui_root = std::move(root);
}

void SimulationRenderer::build_and_swap_snapshots() {
  sim->consume_latest_frame_pair(frame_prev, frame_curr);
}

void SimulationRenderer::update() {
  // First gather snapshots
  build_and_swap_snapshots();
}

void SimulationRenderer::render_frame() {
  SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
  SDL_RenderClear(renderer);

  RenderContext ctx;
  ctx.renderer = renderer;
  ctx.resources = resources;
  ctx.camera_weak = camera;
  ctx.sim_time = frame_curr.sim_time;
  // Read the live control value, not the snapshot stamp: while the sim is
  // paused no snapshots are published, but set_time_factor() takes effect
  // immediately (atomic), and the UI should reflect it.
  ctx.time_factor = sim->get_time_factor();

  double now = SDL_GetTicks() / 1000.0;
  const double sim_dt = frame_curr.sim_time - frame_prev.sim_time;
  const double real_since_curr = now - frame_curr.real_time;

  double alpha = 1.0;
  if (sim_dt > 0.0)
    alpha = (real_since_curr * frame_curr.time_factor) / sim_dt;
  ctx.alpha = std::clamp(alpha, 0.0, 1.0);

  ctx.interp_sim_time =
      frame_prev.sim_time * (1.0 - ctx.alpha) + frame_curr.sim_time * ctx.alpha;

  for (const auto& [id, s1] : frame_curr.riders) {
    auto it0 = frame_prev.riders.find(id);
    if (it0 == frame_prev.riders.end())
      continue;
    const RiderSnapshot& s0 = it0->second;

    RiderRenderState rs;
    // Interpolated
    rs.pos2d = s0.pos2d * (1.0 - ctx.alpha) + s1.pos2d * ctx.alpha;
    rs.lat_pos = s0.lat_pos * (1.0 - ctx.alpha) + s1.lat_pos * ctx.alpha;

    // Non-interpolated from curr_frame
    rs.id = s1.id;
    rs.name = s1.name;
    rs.effort = s1.effort;
    rs.max_effort = s1.max_effort;
    rs.pos = s1.pos;
    rs.slope = s1.slope;
    rs.heading = s1.heading;
    rs.speed = s1.speed;
    rs.effort = s1.effort;
    rs.power = s1.power;
    rs.wbal_fraction = s1.wbal_fraction;

    rs.visual_type = s1.visual_type;
    rs.team_id = s1.team_id;

    rs.group_id = s1.group_id;
    rs.group_role = s1.group_role;
    rs.effort_source = s1.effort_source;
    rs.policy = s1.policy;
    // rs.power_breakdown = s1.power_breakdown;

    ctx.riders[id] = std::move(rs);
  }

  ctx.groups = frame_curr.groups;
  // NOTE This is a value copy of a std::vector<Group>. At N ≤ 20 riders this is
  // well under 1 KB. No allocation concern.

  camera->update(ctx.riders);

  // 1. Draw world-space drawables
  for (auto& d : world_drawables) {
    d->render(&ctx);
  }

  if (ui_root) {
    ui_root->render(&ctx);
    ui_root->render_imgui(&ctx);
  }
}

// Picks in screen space against the same sprite rect the rider is drawn with
// (rider_sprite_rect, shared with RiderDrawable), so hits track the visible
// sprite regardless of zoom, slope or lateral display offset.
RiderId SimulationRenderer::pick_rider(double screen_x, double screen_y) const {
  if (!camera)
    return -1;

  const SDL_FPoint p{static_cast<float>(screen_x),
                     static_cast<float>(screen_y)};

  RiderId found_id = -1;
  float best_d2 = std::numeric_limits<float>::max();

  for (const auto& [id, snap] : frame_curr.riders) {
    const RiderVisualModel& model = resolve_visual_model(snap.visual_type);
    const SDL_FRect rect =
        rider_sprite_rect(*camera, model, snap.pos2d, snap.lat_pos);

    if (!SDL_PointInRectFloat(&p, &rect))
      continue;

    // Overlapping sprites: prefer the one whose centre is closest to the
    // click.
    const float cx = rect.x + rect.w * 0.5f;
    const float cy = rect.y + rect.h * 0.5f;
    const float d2 = (cx - p.x) * (cx - p.x) + (cy - p.y) * (cy - p.y);
    if (d2 < best_d2) {
      best_d2 = d2;
      found_id = id;
    }
  }

  if (found_id != -1)
    SDL_Log("Selected rider ID: %d", found_id);
  return found_id;
}

std::vector<RiderId> SimulationRenderer::get_rider_ids() const {
  std::vector<RiderId> ids;
  ids.reserve(frame_curr.riders.size());
  for (const auto& [id, _] : frame_curr.riders)
    ids.push_back(id);

  std::sort(ids.begin(), ids.end(), [&](RiderId a, RiderId b) {
    return frame_curr.riders.at(a).pos >= frame_curr.riders.at(b).pos;
  });
  return ids;
}

bool SimulationRenderer::handle_event(const SDL_Event* e) {
  // UI above world
  if (ui_root && ui_root->handle_event(e))
    return true;

  for (auto& d : world_drawables)
    if (d->handle_event(e))
      return true;

  return false;
}
