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
  if (snapshot_front.count(0)) {
    camera->set_center(snapshot_front.at(0).pos2d);
  }
}

void SimulationRenderer::add_world_drawable(std::unique_ptr<Drawable> d) {
  world_drawables.push_back(std::move(d));
}

void SimulationRenderer::build_and_swap_snapshots() {
  snapshot_back.clear();

  PhysicsEngine* engine = sim->get_engine();

  {
    // protects only reading rider data
    std::lock_guard<std::mutex> phys_lock(*engine->get_frame_mutex());

    const auto& riders = engine->get_riders();
    for (const Rider* r : riders) {
      if (!r)
        continue;
      snapshot_back.emplace(r->get_id(), r->snapshot());
    }
  }

  {
    // protects only swapping pointers to maps (instant)
    std::lock_guard<std::mutex> lock(snapshot_swap_mtx);
    snapshot_front.swap(snapshot_back);
  }
}

void SimulationRenderer::update() {
  // First gather snapshots
  build_and_swap_snapshots();
  camera->update(snapshot_front);
}

void SimulationRenderer::render_frame() {
  SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
  SDL_RenderClear(renderer);

  RenderContext ctx;
  ctx.renderer = renderer;
  ctx.resources = resources;
  ctx.camera_weak = camera;
  ctx.rider_snapshots = &snapshot_front;

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

  SDL_RenderPresent(renderer);
}

// TODO - make this safe - there is a suggestion in "Code review feedback" chat
// in chatGPT from 26.11.2025
int SimulationRenderer::pick_rider(double screen_x, double screen_y) const {
  if (!camera)
    return -1;

  Vector2d world_pos = camera->screen_to_world(Vector2d(screen_x, screen_y));

  double min_dist = 20.0;
  bool found = false;
  size_t found_id = 0;
  SDL_Log("\n%.1f, %.1f", world_pos.x(), world_pos.y());

  std::lock_guard<std::mutex> lock(snapshot_swap_mtx);
  for (auto& [id, snap] : snapshot_front) {
    double dx = snap.pos2d.x() - world_pos.x();
    double dy = snap.pos2d.y() - world_pos.y();
    double dist = std::sqrt(dx * dx + dy * dy);
    SDL_Log("%s: %.1f", snap.name.c_str(), dist);

    // you might wanna weight X more strictly if they are packed tight
    if (dist < min_dist) {
      min_dist = dist;
      found_id = id;
      found = true;
    }
  }

  if (found) {
    SDL_Log("Selected rider ID: %lu", (unsigned long)found_id);
    return found_id;
  }
  return -1;
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
