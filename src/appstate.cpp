// src/appstate.cpp
#include "appstate.h"
#include "SDL3_ttf/SDL_ttf.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_sdlrenderer3.h"
#include "implot.h"
#include "screen.h"

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

  // ImGuiIO& io = ImGui::GetIO();
  // (void)io;
  // (optionally set io.ConfigFlags, fonts, etc.)

  // 2) Set ImGui style
  ImGui::StyleColorsDark();

  // 3) Initialize backends
  ImGui_ImplSDL3_InitForSDLRenderer(window, renderer);
  ImGui_ImplSDLRenderer3_Init(renderer);

  // 3. Initialize Shared Resources
  resources = new GameResources(renderer);

  // 4. Initialize Simulation
  course = new Course(Course::create_endulating());
  sim = new Simulation(course); // Sim now owns the course
  sim->set_time_factor(5.0);

  // (Optional) Setup default riders here or in Main
}

AppState::~AppState() {
  // Cleanup in reverse order of creation
  if (physics_thread) {
    sim->stop();
    physics_thread->join();
    delete physics_thread;
  }

  if (current_screen_ptr)
    delete current_screen_ptr;
  if (sim)
    delete sim;
  if (course)
    delete course;
  if (resources)
    delete resources;

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

void AppState::switch_screen(ScreenType type) {
  if (current_type == ScreenType::Simulation && sim) {
    sim->pause();
  }

  if (current_screen_ptr) {
    delete current_screen_ptr;
    current_screen_ptr = nullptr;
  }

  current_type = type;

  switch (type) {
  case ScreenType::Menu:
    current_screen_ptr = new MenuScreen(this);
    break;
  case ScreenType::Simulation:
    sim->resume();
    current_screen_ptr = new SimulationScreen(this);
    break;
  case ScreenType::Result:
    current_screen_ptr = new ResultsScreen(this);
    break;
  case ScreenType::Plot:
    current_screen_ptr = new PlotScreen(this);
    break;
  }
}
