#include "corerenderer.h"
#include "display.h"
#include <memory>

CoreRenderer::CoreRenderer(SDL_Renderer* r, GameResources* res)
    : renderer(r), resources(res) {}

void CoreRenderer::add_drawable(std::unique_ptr<Drawable> d) {
  drawables.push_back(std::move(d));
}

void CoreRenderer::clear_drawables() { drawables.clear(); }

void CoreRenderer::render_frame() {
  // No camera, no simulation, no snapshot logic.
  // Only UI drawables.

  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
  SDL_RenderClear(renderer);

  RenderContext ctx;
  ctx.renderer = renderer;
  ctx.resources = resources;
  ctx.camera_weak = std::weak_ptr<Camera>();
  ctx.rider_snapshots = nullptr; // Not used

  for (auto& d : drawables) {
    d->render(&ctx);
  }

  SDL_RenderPresent(renderer);
}
