// screen.h
#ifndef SCREEN_H
#define SCREEN_H

#include "SDL3/SDL_events.h"
#include "widget.h"
#include <memory>

class AppState;
class TTF_Font;
class PlotRenderer;
class SimulationRenderer;

enum class ScreenType { Menu, Simulation, Result, Plot };

class IScreen {
public:
  virtual ~IScreen() {}
  virtual void update() = 0;
  virtual void render() = 0;

  // virtual void handle_event(SDL_Event* e) = 0;
  virtual bool handle_event(const SDL_Event* e) {
    if (e->type == SDL_EVENT_KEY_DOWN) {
      if (e->key.key == SDLK_RETURN) {
        // start simulation
        // state->switch_screen(ScreenType::Simulation);
        return true;
      }
    }
    return false;
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

  void update() override {};

  void render() override;
};

class SimulationScreen : public IScreen {
public:
  // DisplayEngine* display = nullptr;
  int WORLD_WIDTH = 500;

  SimulationScreen(AppState* s);
  ~SimulationScreen(); // delete sim_renderer; }

  void update() override;
  void render() override;

  bool handle_event(const SDL_Event* e) override;

private:
  AppState* state;
  std::unique_ptr<SimulationRenderer> sim_renderer;

  int selected_rider_uid = 0;

  // Camera interaction state
  bool dragging = false;
  int drag_start_x = 0;
  int drag_start_y = 0;

  RiderPanel* rider_panel = nullptr;
};

class PlotScreen : public IScreen {
public:
  PlotScreen(AppState* s);
  ~PlotScreen();

  void update() override;
  void render() override;
  bool handle_event(const SDL_Event* e) override;

private:
  AppState* state;
  std::unique_ptr<PlotRenderer> renderer;
};

#endif
