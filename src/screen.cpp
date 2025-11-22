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
      300, 360, 5, resources->get_fontManager()->get_font("default"), 0,
      [](const RiderSnapshot& s) -> std::string {
        return format_number(s.km_h, 1);
      }));
  display->add_drawable(std::make_unique<ValueField>(
      300, 300, 5, resources->get_fontManager()->get_font("default"), 0,
      [](const RiderSnapshot& s) -> std::string { return s.name; }));
  // state->display->add_drawable(std::make_unique<ValueFieldPanel>(
  //     300, 400, state->resources->get_fontManager()->get_font("default"),
  //     r));

  // camera->set_target_rider(state->sim->get_engine()->get_rider(0));
}

void SimulationScreen::render() { display->render_frame(); }
