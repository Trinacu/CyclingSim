#include "simulationrenderer.h"
#include "camera.h"
#include "display.h"
#include "sim.h"
#include <memory>
#include <mutex>

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

void SimulationRenderer::add_world_drawable(std::unique_ptr<Drawable> d) {
  world_drawables.push_back(std::move(d));
}

void SimulationRenderer::build_and_swap_snapshots() {
  // Build into frame_back (no lock needed if frame_back is not shared)
  frame_back.riders.clear();

  // sim->fill_snapshots(frame_back.riders); // however you currently do it
  // could extract this into a function (gpt says it lives inside Simulation)
  PhysicsEngine* engine = sim->get_engine();
  {
    // protects only reading rider data
    std::lock_guard<std::mutex> phys_lock(*engine->get_frame_mutex());

    const auto& riders = engine->get_riders();
    for (const auto& r : riders) {
      if (!r.get())
        continue;
      frame_back.riders.emplace(r->get_uid(), r->snapshot());
    }
  }

  frame_back.sim_time = sim->get_sim_seconds();
  frame_back.sim_dt = sim->get_dt();
  frame_back.time_factor = sim->get_time_factor(); // or wherever it lives
  frame_back.real_time = SDL_GetTicks() / 1000.0;

  // Only publish if sim advanced
  {
    std::scoped_lock lock(snapshot_swap_mtx);

    if (!frames_initialized) {
      frame_curr = frame_back;
      frame_prev = frame_back; // identical on purpose
      frames_initialized = true;
      return;
    }

    if (frame_back.sim_time <= frame_curr.sim_time) {
      return; // no new sim step -> keep prev/curr stable for interpolation
    }
    frame_prev = std::move(frame_curr);
    frame_curr = std::move(frame_back);
  }
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
  ctx.sim_time = sim->get_sim_seconds();

  double now = SDL_GetTicks() / 1000.0;

  {
    // IMPORTANT: prevent swap while we compute alpha and set pointers
    std::scoped_lock lock(snapshot_swap_mtx);

    ctx.prev_frame = &frame_prev;
    ctx.curr_frame = &frame_curr;

    const double sim_dt = frame_curr.sim_time - frame_prev.sim_time;

    // time since "curr" was published (real seconds)
    const double real_since_curr = now - frame_curr.real_time;

    double alpha = 1.0;
    if (sim_dt > 0.0) {
      alpha = (real_since_curr * frame_curr.time_factor) / sim_dt;
    }
    ctx.alpha = std::clamp(alpha, 0.0, 1.0);

    InterpolatedFrameView view;
    view.alpha = alpha;
    view.interp_sim_time =
        frame_prev.sim_time * (1.0 - alpha) + frame_curr.sim_time * alpha;

    for (const auto& [id, s1] : frame_curr.riders) {
      auto it0 = frame_prev.riders.find(id);
      if (it0 == frame_prev.riders.end())
        continue;

      const RiderSnapshot& s0 = it0->second;

      view.rider_pos[id] = s0.pos2d * (1.0 - alpha) + s1.pos2d * alpha;

      view.rider_slope[id] = s0.slope * (1.0 - alpha) + s1.slope * alpha;

      view.rider_effort[id] = s0.effort * (1.0 - alpha) + s1.effort * alpha;

      view.rider_lateral[id] =
          s0.lateral_offset * (1.0 - alpha) + s1.lateral_offset * alpha;
    }

    ctx.view = std::move(view);
  }

  camera->update(ctx.view);

  // 1. Draw world-space drawables
  int cnt = 0;
  for (auto& d : world_drawables) {
    d->render(&ctx);
  }

  // 2. Draw UI drawables (inherited from CoreRenderer)
  cnt = 0;
  for (auto& d : drawables) {
    d->render(&ctx);
  }

  for (auto& w : drawables)
    w->render_imgui(&ctx);
}

// TODO - make this safe - there is a suggestion in "Code review feedback" chat
// in chatGPT from 26.11.2025
int SimulationRenderer::pick_rider(double screen_x, double screen_y) const {
  if (!camera)
    return -1;

  Vector2d world_pos = camera->screen_to_world(Vector2d(screen_x, screen_y));

  double min_dist = 20.0;
  bool found = false;
  RiderUid found_uid = 0;
  SDL_Log("\n%.1f, %.1f", world_pos.x(), world_pos.y());

  std::lock_guard<std::mutex> lock(snapshot_swap_mtx);
  for (auto& [id, snap] : frame_curr.riders) {
    double dx = snap.pos2d.x() - world_pos.x();
    double dy = snap.pos2d.y() - world_pos.y();
    double dist = std::sqrt(dx * dx + dy * dy);
    SDL_Log("%s: %.1f", snap.name.c_str(), dist);
    SDL_Log("%.1f", snap.km_h);

    // you might wanna weight X more strictly if they are packed tight
    if (dist < min_dist) {
      min_dist = dist;
      found_uid = id;
      found = true;
    }
  }

  if (found) {
    SDL_Log("Selected rider ID: %lu", (unsigned long)found_uid);
    return found_uid;
  }
  return -1;
}

FramePairView SimulationRenderer::get_frame_pair() const {
  std::scoped_lock lock(snapshot_swap_mtx);

  return FramePairView{.prev = &frame_prev, .curr = &frame_curr};
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
