// src/appstate.cpp
#include "appstate.h"
#include "SDL3_ttf/SDL_ttf.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_sdlrenderer3.h"
#include "implot.h"
#include "screen.h"
#include "screenmanager.h"

AppState::AppState() {
  // 1. Initialize SDL Core
  if (!SDL_Init(SDL_INIT_VIDEO)) {
    SDL_Log("Couldn't initialize SDL: %s", SDL_GetError());
    throw std::runtime_error("SDL Init Failed");
  }
  if (!TTF_Init()) {
    SDL_Log("Couldn't initialize TTF: %s", SDL_GetError());
    throw std::runtime_error("TTF Init Failed");
  }

  // 2. Create Window and Renderer ONCE
  if (!SDL_CreateWindowAndRenderer("Cycle Sim", SCREEN_WIDTH, SCREEN_HEIGHT, 0,
                                   &window, &renderer)) {
    SDL_Log("CreateWindowAndRenderer failed: %s", SDL_GetError());
    throw std::runtime_error("Window Creation Failed");
  }

  // init imgui
  // 1) Create ImGui context
  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImPlot::CreateContext(); // Don't forget ImPlot's context too
  ImGui::StyleColorsDark();
  ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer3_Init(renderer);

  // 3. Initialize Shared Resources
  resources = new GameResources(renderer);

  // 4. Initialize Simulation
  course = new Course(Course::create_endulating());
  sim = new Simulation(course); // Sim now owns the course
  sim->set_time_factor(0.1);

  screens = new ScreenManager(this);
  screens->push(ScreenType::Simulation);

  // (Optional) Setup default riders here or in Main
}

AppState::~AppState() {
  // Cleanup in reverse order of creation
  if (physics_thread) {
    sim->stop();
    physics_thread->join();
    delete physics_thread;
  }

  if (sim)
    delete sim;
  if (course)
    delete course;
  if (resources)
    delete resources;
  if (screens)
    delete screens;

  if (renderer)
    SDL_DestroyRenderer(renderer);
  if (window)
    SDL_DestroyWindow(window);

  TTF_Quit();
  SDL_Quit();
}

bool AppState::load_image(const char* id, const char* filename) {
  return resources->get_textureManager()->load_texture(id, filename);
}
