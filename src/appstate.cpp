// src/appstate.cpp
#include "appstate.h"
#include "SDL3/SDL_render.h"
#include "SDL3_ttf/SDL_ttf.h"
#include "backends/imgui_impl_sdl3.h"
#include "backends/imgui_impl_sdlrenderer3.h"
#include "collision_params.h"
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
  if (!SDL_CreateWindowAndRenderer("Cycle Sim", 1000, 640, 0, &window,
                                   &renderer)) {
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
  resources = std::make_unique<GameResources>(renderer);
  course = std::make_unique<Course>(Course::create_endulating());
  sim = std::make_unique<Simulation>(course.get());
  screens = std::make_unique<ScreenManager>(this);

  sim->set_time_factor(0.2);

  Team team("Team1");
  RiderConfig cfg = {0,   "Pedro", 320, 6,   2,     0.05,
                     700, 3.5,     90,  0.5, 24000, Bike::create_road(),
                     team};
  rider_configs.push_back(cfg);
  cfg = {1,   "Power", 300, 6,   2,     0.05,
         700, 3.5,     88,  0.5, 24000, Bike::create_road(),
         team};
  rider_configs.push_back(cfg);
  cfg = {2,     "AccelForce",        300, 6, 2, 0.05, 700, 3.5, 88, 0.5,
         24000, Bike::create_road(), team};
  rider_configs.push_back(cfg);
  cfg = {3,     "AccelEnergy",       300, 6, 2, 0.05, 700, 3.5, 88, 0.5,
         24000, Bike::create_road(), team};
  rider_configs.push_back(cfg);
  cfg = {4,   "Mario", 310, 6,   2,     0.05,
         700, 3.5,     88,  0.5, 24000, Bike::create_road(),
         team};
  rider_configs.push_back(cfg);
  cfg = {5,   "Kojo", 320, 6,   2,     0.05,
         700, 3.5,    88,  0.5, 24000, Bike::create_road(),
         team};
  rider_configs.push_back(cfg);
  cfg = {6,   "Hari", 400, 6,   2,     0.05,
         700, 3.5,    88,  0.5, 24000, Bike::create_road(),
         team};
  rider_configs.push_back(cfg);
  cfg = {7,   "Luka", 320, 6,   2,     0.05,
         700, 3.5,    88,  0.5, 24000, Bike::create_road(),
         team};
  rider_configs.push_back(cfg);
  // cfg = {8, "Buggy", 500, 6, 700, 88, 0.5, 24000, Bike::create_road(), team};
  // rider_configs.push_back(cfg);
  sim->add_riders(rider_configs);

  screens = std::make_unique<ScreenManager>(this);
  screens->push(ScreenType::Simulation);

  auto schedule = std::make_shared<StepEffortSchedule>(std::vector<EffortBlock>{
      {20.0, 0.0},  // easy
      {40.0, 1.3},  // hard
      {600.0, 0.8}, // recovery
      {300.0, 0.9}  // sprint
  });

  sim->set_effort_schedule(0, schedule);

  // (Optional) Setup default riders here or in Main
}

AppState::~AppState() {
  // Cleanup in reverse order of creation
  if (physics_thread.joinable()) {
    sim->stop();
    physics_thread.join();
  }

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

void AppState::start_realtime_tt(double gap_seconds) {
  sim->stop();
  if (physics_thread.joinable())
    physics_thread.join();

  sim->reset();

  if (auto* s = dynamic_cast<SimulationScreen*>(screens->top()))
    s->reset();
  else
    screens->replace(ScreenType::Simulation);

  auto offsets = build_start_offsets(rider_configs, gap_seconds);
  setup_tt_schedules(sim.get(), rider_configs, offsets);

  physics_thread = std::thread([this]() { sim->start_realtime(); });
}
