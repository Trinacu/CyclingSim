#include "plotrenderer.h"
#include "backends/imgui_impl_sdlrenderer3.h"
#include "display.h"
#include "implot.h"
#include <cmath>

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
  ctx.rider_snapshots = nullptr;

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

  static float xs[100], ys[100];
  static bool init = false;

  if (!init) {
    for (int i = 0; i < 100; ++i) {
      xs[i] = i * 0.1f;
      ys[i] = sinf(xs[i]);
    }
    init = true;
  }

  if (ImPlot::BeginPlot("Example Plot")) {
    ImPlot::PlotLine("sin(x)", xs, ys, 100);
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
