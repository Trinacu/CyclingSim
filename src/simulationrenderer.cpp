#include "simulationrenderer.h"
#include <memory>

SimulationRenderer::SimulationRenderer(SDL_Renderer* r,
                                       GameResources* resources,
                                       Simulation* sim_,
                                       std::shared_ptr<Camera> cam)
    : CoreRenderer(r, resources), sim(sim_), camera(std::move(cam)) {
  build_snapshot_map();
  if (snapshotMap.count(0)) {
    camera->set_center(snapshotMap.at(0).pos2d);
  }
}

void SimulationRenderer::add_world_drawable(std::unique_ptr<Drawable> d) {
  world_drawables.push_back(std::move(d));
}

// Build the snapshotMap once per frame
void SimulationRenderer::build_snapshot_map() {
  snapshotMap.clear();
  PhysicsEngine* engine = sim->get_engine();

  std::lock_guard<std::mutex> lock(*engine->get_frame_mutex());
  auto& riders = engine->get_riders();

  for (const Rider* r : riders) {
    if (!r)
      continue;
    RiderSnapshot snap = r->snapshot();
    // snapshotMap[snap.uid] = snap;
    snapshotMap.emplace(snap.uid, snap);
  }
}

void SimulationRenderer::update() { camera->update(snapshotMap); }

void SimulationRenderer::render_frame() {
  // First gather snapshots
  build_snapshot_map();

  // if (snapshotMap.count(camera_target_id)) {
  //   camera->_set_center(snapshotMap.at(camera_target_id).pos2d);
  // }

  SDL_SetRenderDrawColor(renderer, 30, 30, 30, 255);
  SDL_RenderClear(renderer);

  RenderContext ctx;
  ctx.renderer = renderer;
  ctx.resources = resources;
  ctx.camera_weak = camera;
  ctx.rider_snapshots = &snapshotMap;

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

  SDL_RenderPresent(renderer);
}

int SimulationRenderer::pick_rider(double screen_x, double screen_y) const {
  if (!camera)
    return -1;

  Vector2d world_pos = camera->screen_to_world(Vector2d(screen_x, screen_y));

  // need access to snapshots. since we're in the main thread (event loop)
  // and physics is in another, we should use the thread-safe getter
  // or cache the last frame's snapshots
  // let's reuse the snapshot map logic or fetch fresh:

  double min_dist = 20.0;
  bool found = false;
  size_t found_id = 0;
  SDL_Log("%f, %f", world_pos.x(), world_pos.y());

  for (auto& [id, snap] : snapshotMap) {
    double dx = snap.pos2d.x() - world_pos.x();
    double dy = snap.pos2d.y() - world_pos.y();
    double dist = std::sqrt(dx * dx + dy * dy);
    SDL_Log("%s: %f", snap.name.c_str(), dist);

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

void SimulationRenderer::set_target(int uid) {
  target_id = uid;
  camera->set_target(uid);
}
