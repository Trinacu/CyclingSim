// include/appstate.h
#ifndef APPSTATE_H
#define APPSTATE_H

#include "pch.hpp"
#include "sim.h"
#include "texturemanager.h" // For GameResources
#include <SDL3/SDL.h>
#include <chrono>
#include <thread>

// Forward declarations
class ScreenManager;

class AppState {
public:
  std::vector<RiderConfig> rider_configs;
  // Constants
  const int SCREEN_WIDTH = 1000;
  const int SCREEN_HEIGHT = 640;

  const int FPS = 60;

  // Core Hardware/SDL Resources (Owned by AppState)
  SDL_Window* window = nullptr;
  SDL_Renderer* renderer = nullptr;

  std::chrono::steady_clock::time_point last_frame_time;

  // Logic & Assets (Owned by AppState)
  GameResources* resources = nullptr;
  Course* course = nullptr;
  Simulation* sim = nullptr;
  std::thread* physics_thread = nullptr;

  // View State
  ScreenManager* screens;

  // this can be done more elegantly
  void start_realtime_tt(double gap_seconds = 10.0);

  AppState();
  ~AppState();

  bool load_image(const char* id, const char* filename);

  // Helper to get window size dynamically if needed
  Vector2d get_window_size() const {
    int w, h;
    SDL_GetWindowSize(window, &w, &h);
    return Vector2d(w, h);
  }
};

#endif
