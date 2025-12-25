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
    // Build arrays for ImPlot
    static std::vector<double> xs;
    static std::vector<double> ys;

    xs.clear();
    ys.clear();
    xs.reserve(plot_data.size());
    ys.reserve(plot_data.size());

    for (const auto& s : plot_data) {
      xs.push_back(s.time);
      ys.push_back(s.value);
    }

    if (ImPlot::BeginPlot("Speed vs Time")) {
      ImPlot::SetupAxes("Time (s)", "Speed");
      ImPlot::SetupAxesLimits(
          xs.front(), xs.back(), *std::min_element(ys.begin(), ys.end()),
          *std::max_element(ys.begin(), ys.end()), ImPlotCond_Once);
      ImPlot::PlotLine("Rider", xs.data(), ys.data(),
                       static_cast<int>(xs.size()));
      ImPlot::EndPlot();
    }
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

void PlotRenderer::set_data(std::vector<PlotSample> samples) {
  plot_data = std::move(samples);
}
