#ifndef CORE_RENDERER_H
#define CORE_RENDERER_H

#include "display.h"
#include "texturemanager.h"
#include <SDL3/SDL.h>
#include <memory>
#include <vector>

// The fundamental UI-only renderer.
// This draws only 2D/UI Drawables; no camera, no simulation, no world-space.
class CoreRenderer {
public:
  CoreRenderer(SDL_Renderer* renderer, GameResources* resources);
  virtual ~CoreRenderer() = default;

  // Add/remove UI drawables
  void add_drawable(std::unique_ptr<Drawable> d);
  void clear_drawables();

  // The main render entry point (screens call this)
  virtual void render_frame();

  SDL_Renderer* sdl() const { return renderer; }
  GameResources* get_resources() const { return resources; }

protected:
  SDL_Renderer* renderer;   // Raw pointer; owned by AppState
  GameResources* resources; // Raw pointer; owned by AppState
  std::vector<std::unique_ptr<Drawable>> drawables;
};

#endif
