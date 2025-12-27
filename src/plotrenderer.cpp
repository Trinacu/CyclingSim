#include "plotrenderer.h"
#include "backends/imgui_impl_sdlrenderer3.h"
#include "display.h"
#include "implot.h"

PlotRenderer::PlotRenderer(SDL_Renderer* r, GameResources* res)
    : CoreRenderer(r, res) {}

void PlotRenderer::render_frame() {
  // Clear screen first
  SDL_SetRenderDrawColor(renderer, 20, 20, 20, 255);
  SDL_RenderClear(renderer);

  // --- Render standard CoreRenderer widgets ---
  RenderContext ctx;
  ctx.renderer = renderer;
  ctx.resources = resources;
  ctx.camera_weak = std::weak_ptr<Camera>();
  // ctx.rider_snapshots = nullptr;

  for (auto& d : drawables)
    d->render(&ctx);

  // --- ImGui pass ---
  ImGui::NewFrame();
  render_plot_imgui();
  ImGui::Render();
  ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), renderer);

  SDL_RenderPresent(renderer);
}

void PlotRenderer::render_plot_imgui() {
  ImGui::Begin("Plot");

  if (plot_data.empty()) {
    ImGui::Text("No data yet...");
  } else {
    if (!ImPlot::BeginPlot("Metrics")) {

      ImPlot::EndPlot();
      return;
    }

    ImPlot::SetupAxes("Time (s)", "");
    ImPlot::SetupAxis(ImAxis_Y2, "W'bal", ImPlotAxisFlags_AuxDefault);

    for (const auto& series : plot_data) {

      if (series.samples.empty())
        continue;

      std::vector<double> xs;
      std::vector<double> ys;
      xs.reserve(series.samples.size());
      ys.reserve(series.samples.size());

      for (const auto& s : series.samples) {
        xs.push_back(s.x);
        ys.push_back(s.y);
      }

      ImPlot::SetAxis(series.y_axis == 1 ? ImAxis_Y2 : ImAxis_Y1);

      ImPlot::PlotLine(series.label.c_str(), xs.data(), ys.data(),
                       static_cast<int>(xs.size()));
    }
    ImPlot::EndPlot();
  }

  ImGui::End();
}

bool PlotRenderer::handle_event(const SDL_Event* e) {
  // UI above world
  for (auto& d : drawables)
    if (d->handle_event(e))
      return true;

  return false;
}

void PlotRenderer::set_data(std::vector<PlotSeries> data) {
  plot_data = std::move(data);
}
