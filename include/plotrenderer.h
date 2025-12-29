#ifndef PLOT_RENDERER_H
#define PLOT_RENDERER_H

#include "analysis.h"
#include "corerenderer.h"
#include <vector>

class PlotRenderer : public CoreRenderer {
public:
  PlotRenderer(SDL_Renderer* r, GameResources* res);

  void render_frame() override;

  bool handle_event(const SDL_Event* e);

  void set_data(PlotResult result);

private:
  void render_plot_imgui();
  std::string plot_title;
  std::vector<PlotSeries> plot_data;
};

#endif
