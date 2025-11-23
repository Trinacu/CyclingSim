#include "screen.h"
#include "rider.h"
#include "widget.h"

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
  camera = new Camera(state->course, WORLD_WIDTH, screensize);
  // maybe this could take screensize rather than 2 ints?
  display =
      new DisplayEngine(state->sim, s->SCREEN_WIDTH, s->SCREEN_HEIGHT, camera);

  resources = new GameResources(display->get_renderer());
  display->set_resources(resources);

  display->add_drawable(std::make_unique<CourseDrawable>(state->course));
  display->add_drawable(std::make_unique<RiderDrawable>());
  display->add_drawable(std::make_unique<Stopwatch>(
      10, 10, resources->get_fontManager()->get_font("stopwatch"), state->sim));

  display->add_drawable(std::make_unique<ValueField>(
      300, 360, 200, 40, resources->get_fontManager()->get_font("default"), 0,
      [](const RiderSnapshot& s) -> std::string {
        return format_number(s.km_h, 1);
      }));

  // 2. Create the Panel
  auto panel = std::make_unique<RiderPanel>(
      20, 200, "Live Data", 0,
      resources->get_fontManager()->get_font("default"));

  // 3. Add Rows (Using Lambdas for custom logic)

  // SPEED
  panel->add_row("Speed", "km/h",
                 [](const RiderSnapshot& s) { return format_number(s.km_h); });

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
  display->add_drawable(std::move(panel));

  // display->add_drawable(std::make_unique<ValueField>(
  //     300, 300, 5, resources->get_fontManager()->get_font("default"), 0,
  //     [](const RiderSnapshot& s) -> std::string { return s.name; }));
  // state->display->add_drawable(std::make_unique<ValueFieldPanel>(
  //     300, 400, state->resources->get_fontManager()->get_font("default"),
  //     r));

  // camera->set_target_rider(state->sim->get_engine()->get_rider(0));
}

void SimulationScreen::render() { display->render_frame(); }
