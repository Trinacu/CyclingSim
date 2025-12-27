#include "screen.h"
#include "SDL3/SDL_log.h"
#include "appstate.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_sdlrenderer3.h"
#include "imgui.h"
#include "plotrenderer.h"
#include "plotting.h"
#include "screenmanager.h"
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

  // TODO - fix this to set the right uid effort, not just fix to uid=0
  auto num = std::make_unique<EditableNumberField>(
      200, 400, 80, 26, default_font, state->window, [&](double v) {
        state->sim->get_engine()->set_rider_effort(0, v);
        const Rider* r = state->sim->get_engine()->get_rider_by_idx(0);
        SDL_Log("%s effort set to %d %%", r->name.c_str(), int(100 * v));
      });

  sim_renderer->add_drawable(std::move(num));

  auto name_field = std::make_unique<EditableStringField>(
      200, 450, 120, 26, default_font, state->window,
      [&](const std::string& s) { SDL_Log("%s", s.c_str()); });

  sim_renderer->add_drawable(std::move(name_field));
}

SimulationScreen::~SimulationScreen() = default;

void SimulationScreen::update() { sim_renderer->update(); }

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

  case SDL_EVENT_MOUSE_WHEEL: {
    constexpr double ZOOM_SENSITIVITY = 0.1;
    sim_renderer->get_camera()->zoom(e->wheel.y * ZOOM_SENSITIVITY);
    return true;
  }

  case SDL_EVENT_KEY_DOWN:
    switch (e->key.key) {
    case SDLK_LEFT:
      cycle_rider(-1);
      return true;

    case SDLK_RIGHT:
      cycle_rider(+1);
      return true;

    case SDLK_ESCAPE:
      state->screens->replace(ScreenType::Menu);
      return true;
    }
    break;
  }
  return false;
}

void SimulationScreen::cycle_rider(int direction) {
  const auto& riders = sim_renderer->get_frame_pair().curr->riders;
  if (riders.empty())
    return;

  // Extract IDs into a sorted list for stable cycling
  std::vector<int> ids;
  ids.reserve(riders.size());
  for (const auto& [id, _] : riders)
    ids.push_back(id);

  std::sort(ids.begin(), ids.end());

  // Find current index
  auto it = std::find(ids.begin(), ids.end(), selected_rider_uid);
  int idx = (it == ids.end()) ? 0 : std::distance(ids.begin(), it);

  // Cycle with wrap-around
  idx = (idx + direction + ids.size()) % ids.size();

  selected_rider_uid = ids[idx];

  // Apply selection
  sim_renderer->get_camera()->set_target_id(selected_rider_uid);
  sim_renderer->get_rider_panel()->set_rider_id(selected_rider_uid);
}

PlotScreen::PlotScreen(AppState* s) : state(s) {
  renderer = std::make_unique<PlotRenderer>(state->renderer, state->resources);

  TTF_Font* f = state->resources->get_fontManager()->get_font("default");

  renderer->add_drawable(std::make_unique<Button>(
      20, 20, 120, 30, "Back to simulation", f,
      [this]() { state->screens->replace(ScreenType::Simulation); }));

  renderer->add_drawable(
      std::make_unique<Button>(20, 60, 120, 30, "Pause", f, [this]() {
        auto sim = state->sim;
        if (sim->is_paused())
          sim->resume();
        else
          sim->pause();
      }));
}

PlotScreen::~PlotScreen() = default;

void PlotScreen::update() {
  if (!running && !result.has_value()) {
    running = true;

    future = std::async(std::launch::async, run_plot_simulation,
                        std::cref(*state->course),
                        std::cref(state->rider_configs), target_uid);
  }

  // Poll completion (non-blocking)
  if (running && future.wait_for(std::chrono::milliseconds(0)) ==
                     std::future_status::ready) {
    result = future.get();
    running = false;

    renderer->set_data(result->series);
  }
}

void PlotScreen::render() { renderer->render_frame(); }

bool PlotScreen::handle_event(const SDL_Event* e) {
  if (renderer->handle_event(e))
    return true;

  // primitive but it prevents changing screen while sim is running
  if (running) {
    SDL_Log("can't change screen - sim is runnning.");
    return true; // consume input
  }

  if (e->type == SDL_EVENT_KEY_DOWN && e->key.key == SDLK_ESCAPE) {
    state->screens->replace(ScreenType::Simulation);
    return true;
  }
  return false;
}

// void PlotScreen::run_simulation() {
//   auto sim = std::make_unique<Simulation>(state->course);
//
//   // copy riders/config
//   for (const auto& cfg : state->rider_configs)
//     sim->get_engine()->add_rider(cfg);
//
//   OfflineSimulationRunner runner(std::move(sim));
//
//   auto plot_obs = std::make_unique<RiderValuePlotObserver>(0);
//   runner.add_observer(plot_obs.get());
//
//   runner.run();
//
//   renderer->set_data(plot_obs->data());
// }
