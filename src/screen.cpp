#include "screen.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_sdlrenderer3.h"
#include "imgui.h"
#include "implot.h"
#include "simulationrenderer.h"
#include "snapshot.h"
#include "widget.h"
#include <memory>

void MenuScreen::render() {
  SDL_SetRenderDrawColor(state->renderer, 30, 30, 30, 255);
  SDL_RenderClear(state->renderer);

  // draw a simple "Press ENTER" text
  // you probably have font rendering already via resources

  SDL_RenderPresent(state->renderer);
}

void ResultsScreen::render() {
  SDL_SetRenderDrawColor(state->renderer, 0, 0, 0, 255);
  SDL_RenderClear(state->renderer);

  // show rider stats
  // auto snap1 = state->sim->get_engine()->snapshot_of(state->r1);
  // auto snap2 = state->sim->get_engine()->snapshot_of(state->r2);

  // draw simple text:
  // "Rider1 final pos: 123.4"
  // "Rider2 final pos: 110.2"

  SDL_RenderPresent(state->renderer);
}

SimulationScreen::SimulationScreen(AppState* s) : state(s) {
  Vector2d screensize(s->SCREEN_WIDTH, s->SCREEN_HEIGHT);
  // maybe this could take screensize rather than 2 ints?
  // display = new DisplayEngine(state, screensize, WORLD_WIDTH);
  auto cam = std::make_shared<Camera>(s->course, WORLD_WIDTH, screensize);
  sim_renderer = std::make_unique<SimulationRenderer>(s->renderer, s->resources,
                                                      s->sim, cam);

  TTF_Font* default_font =
      state->resources->get_fontManager()->get_font("default");

  sim_renderer->add_world_drawable(
      std::make_unique<CourseDrawable>(state->sim->get_engine()->get_course()));
  sim_renderer->add_world_drawable(std::make_unique<RiderDrawable>());

  sim_renderer->add_drawable(std::make_unique<Stopwatch>(
      20, 20, state->resources->get_fontManager()->get_font("stopwatch"),
      state->sim));

  sim_renderer->add_drawable(std::make_unique<TimeControlPanel>(
      400, 20, 40, default_font, state->sim));

  // 2. Create the Panel
  auto panel = std::make_unique<RiderPanel>(20, 120, default_font);

  // 3. Add Rows (Using Lambdas for custom logic)

  // SPEED
  panel->add_row("Speed", "km/h", [](const RiderSnapshot& s) {
    return format_number(s.km_h, 2);
  });

  // POWER
  panel->add_row("Power", "W", [](const RiderSnapshot& s) {
    return format_number(s.power, 0); // Precision 0 for watts
  });

  // DISTANCE
  panel->add_row("Dist", "km", [](const RiderSnapshot& s) {
    return format_number(s.pos / 1000.0);
  });

  // GRADIENT (Custom logic inside lambda!)
  panel->add_row("Grad", "%", [](const RiderSnapshot& s) {
    // Assuming you add 'slope' to RiderSnapshot
    // return format_number(s.slope * 100.0);
    return format_number(s.slope * 100.0);
  });

  RiderPanel* p = panel.get();

  sim_renderer->set_rider_panel(p);
  sim_renderer->add_drawable(std::move(panel));

  // display->add_drawable(std::make_unique<ValueField>(
  //     300, 300, 5, resources->get_fontManager()->get_font("default"), 0,
  //     [](const RiderSnapshot& s) -> std::string { return s.name; }));
  // display->add_drawable(std::make_unique<ValueFieldPanel>(
  //     300, 400, state->resources->get_fontManager()->get_font("default"),
  //     r));

  // camera->set_target_rider(state->sim->get_engine()->get_rider(0));
}

void SimulationScreen::update() { sim_renderer->update(); }

// NOTE: You'll need to pass the SDL_Window* or the window size to this function
// A safer way is to use ImGui::GetIO().DisplaySize
void SetupDockspaceWindow() {
  // Get the I/O state for ImGui
  // ImGuiIO& io = ImGui::GetIO();

  // Set the next window position and size to cover the entire screen
  ImGui::SetNextWindowPos(ImVec2(0, 0));
  ImGui::SetNextWindowSize(ImVec2(200, 200));

  // Set the window flags to remove all decorations and interaction
  // ImGuiWindowFlags_NoDecoration: Removes title bar, border, resize handle
  // ImGuiWindowFlags_NoMove: Prevents the window from being moved
  // ImGuiWindowFlags_NoResize: Prevents resizing
  // ImGuiWindowFlags_NoBringToFrontOnFocus: Keeps it in the back
  // ImGuiWindowFlags_NoBackground: (Optional, only if you want a transparent
  // background) ImGuiWindowFlags_NoDocking: Essential if using a dockspace,
  // good practice here.
  ImGuiWindowFlags window_flags =
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
      ImGuiWindowFlags_NoBackground; // Use your SDL clear color as the
                                     // background

  // Begin the full-screen background window
  // The name "SDL3_Background" is just a unique identifier
  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

  ImGui::Begin("SDL3_Background", nullptr, window_flags);

  ImGui::PopStyleVar(3);
}

void RenderSimplePlot() {
  // ... (Your static data generation remains the same) ...
  // Define the data arrays and generate data (as shown in the previous answer)
  static float x_data[100];
  static float y_data[100];
  // ... data generation ...

  // --- Plotting area starts here ---

  // The plot will take up 50% of the background window's width
  float plot_width = 200; // ImGui::GetContentRegionAvail().x * 0.5f;

  // Center the plot horizontally within the invisible background window
  ImGui::SetCursorPosX(20);

  // Start the ImPlot Plot area
  // Use ImVec2(plot_width, 300) to control its size within the window
  if (ImPlot::BeginPlot("My Sine Wave Overlay", "Time (s)", "Amplitude",
                        ImVec2(plot_width, 300))) {
    ImPlot::PlotLine("sin(3x)", x_data, y_data, 100);

    // End the ImPlot Plot area
    ImPlot::EndPlot();
  }
}

void SimulationScreen::render() {
  ImGui_ImplSDLRenderer3_NewFrame();
  ImGui_ImplSDL3_NewFrame();
  ImGui::NewFrame();

  sim_renderer->render_frame();

  // 3) Now have ImGui render its draw data
  ImGui::Render();
  ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), state->renderer);

  // 4) Finally present
  SDL_RenderPresent(state->renderer);
}

bool SimulationScreen::handle_event(const SDL_Event* e) {
  if (sim_renderer->handle_event(e))
    return true;

  switch (e->type) {
  case SDL_EVENT_MOUSE_BUTTON_DOWN:
    if (e->button.button == SDL_BUTTON_LEFT) {
      int uid = sim_renderer->pick_rider(e->button.x, e->button.y);

      if (uid != -1) {
        selected_rider_uid = uid;
        // sim_renderer->get_camera()->set_target(uid);
        sim_renderer->get_camera()->set_target_id(uid);
        sim_renderer->get_rider_panel()->set_rider_id(uid);
        return true;
      }
    }
    if (e->button.button == SDL_BUTTON_RIGHT) {
      // Start dragging for camera pan
      dragging = true;
      drag_start_x = e->button.x;
      drag_start_y = e->button.y;
      return true;
    }
    break;

    // button released
  case SDL_EVENT_MOUSE_BUTTON_UP:
    if (e->button.button == SDL_BUTTON_RIGHT) {
      dragging = false;
      return true;
    }
    break;

  case SDL_EVENT_MOUSE_WHEEL:
    // sim_renderer->get_camera()->zoom(e.wheel.y * 0.1);
    // return true;
    break;

  case SDL_EVENT_KEY_DOWN:
    if (e->key.key == SDLK_ESCAPE)
      state->switch_screen(ScreenType::Menu);
    break;
  }
  return true;
}

void PlotScreen::render() {
  SDL_SetRenderDrawColor(state->renderer, 20, 20, 20, 255);
  SDL_RenderClear(state->renderer);

  // You MUST start a frame before any ImGui
  ImGui::NewFrame();

  ImGui::Begin("My Plot");

  if (ImPlot::BeginPlot("Example Plot")) {
    static float xs[100], ys[100];
    static bool initialized = false;

    if (!initialized) {
      for (int i = 0; i < 100; ++i) {
        xs[i] = i * 0.1f;
        ys[i] = sinf(xs[i]);
      }
      initialized = true;
    }

    ImPlot::PlotLine("sin(x)", xs, ys, 100);
    ImPlot::EndPlot();
  }

  ImGui::End();

  // Render ImGui
  ImGui::Render();
  ImGui_ImplSDLRenderer3_RenderDrawData(ImGui::GetDrawData(), state->renderer);

  SDL_RenderPresent(state->renderer);
}

bool PlotScreen::handle_event(const SDL_Event* e) {
  // feed events to imgui first
  ImGui_ImplSDL3_ProcessEvent(e);

  if (e->type == SDL_EVENT_KEY_DOWN && e->key.key == SDLK_ESCAPE) {
    state->switch_screen(ScreenType::Simulation);
    return true;
  }
  return false;
}
