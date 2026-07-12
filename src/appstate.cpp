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
#include <cmath>

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
  // Modest angled wind (~60 deg off the course axis) for the rotation demo:
  // expect echelon stagger, consistently windward swings, slightly lower
  // line speed.  Interactive gate for the B2 yaw constants.
  course->set_wind({M_PI / 3.0, 3.5});
  sim = std::make_unique<Simulation>(course.get());
  runner = std::make_unique<RealtimeSimRunner>(sim.get());

  runner->set_time_factor(0.2);

  TeamId team = sim->get_engine()->add_team("Team1");
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

  // C3.0 feel-check playground: riders 0-6 start in a tight line, Luka (7)
  // alone off the back.  No schedules, rotation, or policies — every rider
  // stays in Manual mode, so the effort slider is live for all of them.
  // The initial setpoint is each rider's cruise effort at a common target
  // speed, so the formation more or less holds until adjusted by hand.
  constexpr double kTargetSpeed = 8.0;  // m/s, common to everyone
  constexpr double kLineSpacing = 3.0;  // m nose-to-nose (~1.3 m wheel gap)
  constexpr double kLonerGap = 30.0;    // Luka to the back of the line

  const auto& riders = sim->get_engine()->get_riders();
  for (int id = 0; id <= 6; ++id)
    riders.at(id)->set_start_pos(kLonerGap + (6 - id) * kLineSpacing);
  // Luka stays at pos 0.

  // One physics tick to populate each rider's env (density, slope, wind
  // projection) before the cruise-effort query — direct engine access is
  // safe here, the runner hasn't started.  The effort itself comes from the
  // what-if query with cda_factor 1: at standstill the live yaw factor is
  // capped-out garbage and draft factors are meaningless.
  sim->get_engine()->update(0.01);
  for (const auto& [id, r] : riders) {
    const auto [wind_dir, wind_speed] = course->get_wind(r->get_pos());
    const double headwind =
        wind_speed * std::cos(wind_dir - r->get_heading());
    const double power = r->cruise_power_at(
        kTargetSpeed, course->get_slope(r->get_pos()), headwind, 1.0);
    r->set_effort(power / r->get_ftp());
  }
}

AppState::~AppState() {
  // Cleanup in reverse order of creation
  if (runner)
    runner->stop();

  // Widgets and the texture/font managers own SDL_Textures/TTF_Fonts, so they
  // must be destroyed while the renderer and TTF are still alive.  Members are
  // otherwise destroyed after this body runs — too late — hence the explicit
  // resets here.
  screens.reset();
  resources.reset();

  ImGui_ImplSDLRenderer3_Shutdown();
  ImGui_ImplSDL3_Shutdown();
  ImPlot::DestroyContext();
  ImGui::DestroyContext();

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
  runner->stop();

  sim->reset();

  if (auto* s = dynamic_cast<SimulationScreen*>(screens->top()))
    s->reset();
  else
    screens->replace(ScreenType::Simulation);

  auto offsets = build_start_offsets(rider_configs, gap_seconds);
  setup_tt_schedules(sim.get(), rider_configs, offsets);

  runner->start();
}
