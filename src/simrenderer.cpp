#include "simrenderer.h"
#include "camera.h"
#include "display.h"
#include "sim.h"
#include "snapshot.h"
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
  ctx.time_factor = frame_curr.time_factor;

  // ctx.prev_frame = &frame_prev;
  // ctx.curr_frame = &frame_curr;

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
    rs.slope = s0.slope * (1.0 - ctx.alpha) + s1.slope * ctx.alpha;
    rs.effort = s0.effort * (1.0 - ctx.alpha) + s1.effort * ctx.alpha;

    // Non-interpolated from curr_frame
    rs.id = s1.id;
    rs.name = s1.name;
    rs.max_effort = s1.max_effort;
    rs.power = s1.power;
    rs.speed = s1.speed;
    rs.pos = s1.pos;
    rs.visual_type = s1.visual_type;
    rs.team_id = s1.team_id;
    rs.power_breakdown = s1.power_breakdown;

    ctx.riders[id] = std::move(rs);
  }

  camera->update(ctx.riders);

  // 1. Draw world-space drawables
  for (auto& d : world_drawables) {
    d->render(&ctx);
  }

  // 2. Draw UI drawables (inherited from CoreRenderer)
  for (auto& d : drawables) {
    d->render(&ctx);
  }

  for (auto& w : drawables)
    w->render_imgui(&ctx);
}

// TODO - make this safe - there is a suggestion in "Code review feedback" chat
// in chatGPT from 26.11.2025
RiderId SimulationRenderer::pick_rider(double screen_x, double screen_y) const {
  if (!camera)
    return -1;

  Vector2d world_pos = camera->screen_to_world(Vector2d(screen_x, screen_y));

  double min_dist = 20.0;
  bool found = false;
  RiderUid found_id = 0;

  for (auto& [id, snap] : frame_curr.riders) {
    double dx = snap.pos2d.x() - world_pos.x();
    double dy = snap.pos2d.y() - world_pos.y();
    double dist = std::sqrt(dx * dx + dy * dy);

    // you might wanna weight X more strictly if they are packed tight
    if (dist < min_dist) {
      min_dist = dist;
      found_id = id;
      found = true;
    }
  }

  if (found) {
    SDL_Log("Selected rider ID: %d", found_id);
    return found_id;
  }
  return -1;
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
  for (auto& d : drawables)
    if (d->handle_event(e))
      return true;

  for (auto& d : world_drawables)
    if (d->handle_event(e))
      return true;

  return false;
}
