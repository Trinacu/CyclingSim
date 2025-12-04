#ifndef PLOT_RENDERER_H
#define PLOT_RENDERER_H

#include "corerenderer.h"

class PlotRenderer : public CoreRenderer {
public:
  PlotRenderer(SDL_Renderer* r, GameResources* res);

  void render_frame() override;

  bool handle_event(const SDL_Event* e);

private:
  void render_plot_imgui();
};

#endif
