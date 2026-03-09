#include "screen.h"
#include "SDL3/SDL_log.h"
#include "appstate.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_sdlrenderer3.h"
#include "imgui.h"
#include "pch.hpp"
#include "plotrenderer.h"
#include "plotting.h"
#include "screenmanager.h"
#include "sim.h"
#include "simrenderer.h"
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
  auto cam = std::make_shared<Camera>(s->course.get(), WORLD_WIDTH, screensize);
  sim_renderer = std::make_unique<SimulationRenderer>(
      s->renderer, s->resources.get(), s->sim.get(), cam);

  TTF_Font* default_font =
      state->resources->get_fontManager()->get_font("default");

  sim_renderer->add_world_drawable(
      std::make_unique<CourseDrawable>(state->sim->get_engine()->get_course()));
  sim_renderer->add_world_drawable(std::make_unique<RiderDrawable>());

  sim_renderer->add_drawable(std::make_unique<Stopwatch>(
      20, 20, state->resources->get_fontManager()->get_font("stopwatch"),
      state->sim.get()));

  Vector2d scr_size = s->get_window_size();

  int map_w = 400;
  int map_h = 100;
  int pos_x = scr_size[0] - map_w - 8;
  int pos_y = scr_size[1] - map_h - 8;
  sim_renderer->add_drawable(std::make_unique<MinimapWidget>(
      pos_x, pos_y, map_w, map_h, s->course.get()));

  sim_renderer->add_drawable(std::make_unique<TimeControlPanel>(
      400, 20, 40, default_font, state->sim.get()));

  // 2. Create the Panel
  auto panel =
      std::make_unique<RiderPanel>(20, 120, default_font, sim_renderer.get());

  // 3. Add Rows (Using Lambdas for custom logic)

  // SPEED
  panel->add_row("Speed", "km/h", [](const RiderRenderState& s) {
    return format_number(3.6 * s.speed, 2);
  });

  // POWER
  panel->add_row("Power", "W", [](const RiderRenderState& s) {
    return format_number(s.power, 0); // Precision 0 for watts
  });

  // DISTANCE
  panel->add_row("Dist", "km", [](const RiderRenderState& s) {
    return format_number(s.pos / 1000.0, 3);
  });

  // GRADIENT (Custom logic inside lambda!)
  panel->add_row("Grad", "%", [](const RiderRenderState& s) {
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
        state->sim->set_rider_effort(selected_rider, v);
        const Rider* r = state->sim->get_engine()->get_rider_by_id(0);
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

void SimulationScreen::reset() { sim_renderer->reset(); }

bool SimulationScreen::handle_event(const SDL_Event* e) {
  if (sim_renderer->handle_event(e))
    return true;

  switch (e->type) {
  case SDL_EVENT_MOUSE_BUTTON_DOWN:
    if (e->button.button == SDL_BUTTON_LEFT) {
      RiderUid uid = sim_renderer->pick_rider(e->button.x, e->button.y);

      if (uid != -1) {
        select_rider(uid);
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

  std::vector<RiderId> ids;
  ids.reserve(riders.size());

  for (const auto& [id, _] : riders)
    ids.push_back(id);

  std::sort(ids.begin(), ids.end());

  // Find current index
  auto it = std::find(ids.begin(), ids.end(), selected_rider);
  int idx = (it == ids.end()) ? 0 : std::distance(ids.begin(), it);

  idx = (idx + direction + ids.size()) % ids.size();

  select_rider(ids[idx]);
}

void SimulationScreen::select_rider(RiderId id) {
  selected_rider = id;
  sim_renderer->get_camera()->set_target_id(id);
  sim_renderer->get_rider_panel()->set_rider_id(id);
}

PlotScreen::PlotScreen(AppState* s) : state(s) {
  renderer =
      std::make_unique<PlotRenderer>(state->renderer, state->resources.get());

  TTF_Font* f = state->resources->get_fontManager()->get_font("default");

  renderer->add_drawable(std::make_unique<Button>(
      20, 20, 120, 30, "Back to simulation", f,
      [this]() { state->screens->replace(ScreenType::Simulation); }));

  renderer->add_drawable(
      std::make_unique<Button>(20, 60, 120, 30, "Pause", f, [this]() {
        if (state->sim->is_paused())
          state->sim->resume();
        else
          state->sim->pause();
      }));
}

PlotScreen::~PlotScreen() = default;

void PlotScreen::update() {
  if (!running && !result.has_value()) {
    running = true;

    future = std::async(std::launch::async, run_plot_simulation,
                        std::cref(*state->course),
                        std::cref(state->rider_configs), target_id);
  }

  // Poll completion (non-blocking)
  if (running && future.wait_for(std::chrono::milliseconds(0)) ==
                     std::future_status::ready) {
    result = future.get();
    running = false;

    if (result.has_value())
      renderer->set_data(*result);
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

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

static std::string fmt_time(double seconds) {
  int total_tenths = static_cast<int>(round(seconds * 10));
  int tenths = total_tenths % 10;
  int total_secs = total_tenths / 10;
  int secs = total_secs % 60;
  int total_mins = total_secs / 60;
  int mins = total_mins % 60;
  int hours = total_mins / 60;
  char buf[16];
  snprintf(buf, sizeof(buf), "%02d:%02d:%02d.%d", hours, mins, secs, tenths);
  return buf;
}

static void log_results(const TimeTrialResult& r) {
  SDL_Log("=== TIME TRIAL RESULTS ===");

  // Header: checkpoints as distances in km
  std::string header = "Rank  Name                ";
  for (double d : r.checkpoint_distances) {
    char col[16];
    snprintf(col, sizeof(col), "  %5.1f km", d / 1000.0);
    header += col;
  }
  SDL_Log("%s", header.c_str());

  // One row per rider (already sorted by finish time)
  for (int i = 0; i < (int)r.riders.size(); ++i) {
    const auto& rider = r.riders[i];

    std::string row;
    char rank_name[32];
    snprintf(rank_name, sizeof(rank_name), "%-4d  %-20s", i + 1,
             rider.name.c_str());
    row += rank_name;

    // Build a map of checkpoint_distance -> race_time for quick lookup
    std::unordered_map<double, double> splits;
    for (const auto& entry : rider.timeline)
      splits[entry.checkpoint_distance] = entry.race_time;

    for (double d : r.checkpoint_distances) {
      auto it = splits.find(d);
      if (it != splits.end()) {
        row += "  " + fmt_time(it->second);
      } else {
        row += "       DNF  ";
      }
    }

    // Gap to leader at finish line (last checkpoint)
    if (i > 0 && !r.riders[0].timeline.empty() && !rider.timeline.empty()) {
      double leader_finish = r.riders[0].timeline.back().race_time;
      double my_finish = rider.timeline.back().race_time;
      double gap = my_finish - leader_finish;
      char gap_buf[16];
      snprintf(gap_buf, sizeof(gap_buf), "  +%.1fs", gap);
      row += gap_buf;
    } else if (i == 0) {
      row += "  LEADER";
    }

    SDL_Log("%s", row.c_str());
  }

  SDL_Log("==========================");
}

// ---------------------------------------------------------------------------
// TimeTrialScreen
// ---------------------------------------------------------------------------

TimeTrialScreen::TimeTrialScreen(AppState* s) : state(s) {}

void TimeTrialScreen::update() {
  // Kick off the async run once
  if (!running && !result.has_value()) {
    running = true;
    future = std::async(
        std::launch::async, run_time_trial, std::cref(*state->course),
        std::cref(state->rider_configs), 60.0 /* start gap seconds */);
  }

  // Poll for completion (non-blocking)
  if (running && future.wait_for(std::chrono::milliseconds(0)) ==
                     std::future_status::ready) {
    result = future.get();
    running = false;
    log_results(*result);
  }
}

void TimeTrialScreen::render() {
  // Placeholder — just clear the screen while running / after logging
  SDL_SetRenderDrawColor(state->renderer, 20, 20, 20, 255);
  SDL_RenderClear(state->renderer);

  // TODO: replace with ImGui results table
}

bool TimeTrialScreen::handle_event(const SDL_Event* e) {
  if (running) {
    SDL_Log("Time trial running, please wait...");
    return true;
  }

  if (e->type == SDL_EVENT_KEY_DOWN && e->key.key == SDLK_ESCAPE) {
    state->screens->replace(ScreenType::Simulation);
    return true;
  }

  return false;
}
