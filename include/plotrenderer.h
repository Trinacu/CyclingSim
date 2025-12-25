#ifndef PLOT_RENDERER_H
#define PLOT_RENDERER_H

#include "corerenderer.h"
#include "sim.h"

class PlotRenderer : public CoreRenderer {
public:
  PlotRenderer(SDL_Renderer* r, GameResources* res);

  void render_frame() override;

  bool handle_event(const SDL_Event* e);

  void set_data(std::vector<PlotSample> samples);

private:
  void render_plot_imgui();
  std::vector<PlotSample> plot_data;
};

#endif
