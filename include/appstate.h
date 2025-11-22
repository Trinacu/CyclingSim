// appstate.h
#ifndef APPSTATE_H
#define APPSTATE_H

#include "SDL3/SDL_video.h"
#include "sim.h"
#include <SDL3/SDL.h>
#include <thread>

enum class ScreenType;

struct AppState {
  int SCREEN_WIDTH = 1000;
  int SCREEN_HEIGHT = 640;

  SDL_Window* window = nullptr;
  SDL_Renderer* renderer = nullptr;

  // model layer
  Course* course = nullptr;
  Simulation* sim = nullptr;
  std::thread* physics_thread = nullptr;

  // view layer
  class IScreen* screen = nullptr;
  ScreenType current_screen;

  void switch_screen(ScreenType type);
};

#endif
