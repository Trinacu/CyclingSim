// screen.h
#ifndef SCREEN_H
#define SCREEN_H

#include "SDL3/SDL_events.h"
#include "appstate.h"
#include "display.h"

enum class ScreenType { Menu, Simulation, Result };

class IScreen {
public:
  virtual ~IScreen() {}
  virtual void update() = 0;
  virtual void render() = 0;

  // virtual void handle_event(SDL_Event* e) = 0;
  void handle_event(SDL_Event* e) {
    if (e->type == SDL_EVENT_KEY_DOWN) {
      if (e->key.key == SDLK_RETURN) {
        // start simulation
        // state->switch_screen(ScreenType::Simulation);
      }
    }
  }
};

class MenuScreen : public IScreen {
public:
  AppState* state;

  MenuScreen(AppState* s) : state(s) {}

  void update() override {}
  void render() override;
};

class ResultsScreen : public IScreen {
public:
  AppState* state;
  ResultsScreen(AppState* s) : state(s) {}

  void update() override {}

  void render() override;
};

class SimulationScreen : public IScreen {
public:
  AppState* state;
  Camera* camera = nullptr;
  DisplayEngine* display = nullptr;
  ResourceProvider* resources = nullptr;
  int WORLD_WIDTH = 200;

  SimulationScreen(AppState* s);
  void update() override {}

  // void handle_event(SDL_Event* e) override { state->display->handle_event(e);
  // }

  void render() override;

  ~SimulationScreen() {
    delete display;
    delete resources;
    delete camera;
  }
};

#endif
