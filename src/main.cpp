#include "SDL3/SDL_events.h"
#include "SDL3/SDL_init.h"
#include "appstate.h"
#include "sim.h"
#include <exception>
#define SDL_MAIN_USE_CALLBACKS 1 /* use the callbacks instead of main() */
#include "screen.h"
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>

int SCREEN_WIDTH = 1000;
int SCREEN_HEIGHT = 640;
int WORLD_WIDTH = 400;

SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[]) {
  try {
    auto* state = new AppState();

    Team team("Team1");
    Rider* r = Rider::create_generic(team);
    state->sim->get_engine()->add_rider(r);

    Rider* r2 = new Rider("Pedro", 300, 80, 0.3, Bike::create_generic(), team);
    r2->pos = 20;
    state->sim->get_engine()->add_rider(r2);

    state->physics_thread =
        new std::thread([sim = state->sim]() { sim->start_realtime(); });

    // state->window =
    //     SDL_CreateWindow("Cycling Sim", SCREEN_WIDTH, SCREEN_HEIGHT, 0);
    // state->renderer = SDL_CreateRenderer(state->window, nullptr);

    state->switch_screen(ScreenType::Simulation);

    *appstate = state;
    return SDL_APP_CONTINUE;
  } catch (const std::exception& e) {
    SDL_Log("Application init failed: %s", e.what());
    return SDL_APP_FAILURE;
  }
}

/* This function runs when a new event (mouse input, keypresses, etc) occurs. */
SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
  auto* state = static_cast<AppState*>(appstate);
  if (event->type == SDL_EVENT_QUIT) {
    return SDL_APP_SUCCESS; /* end the program, reporting success */
  }

  // Pass event to current screen
  if (state->current_screen_ptr) {
    state->current_screen_ptr->handle_event(event);
  }

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

/* This function runs once per frame, and is the heart of the program. */
SDL_AppResult SDL_AppIterate(void* appstate) {
  auto* state = static_cast<AppState*>(appstate);
  if (state->current_screen_ptr) {
    state->current_screen_ptr->update();
    state->current_screen_ptr->render();
  }

  return SDL_APP_CONTINUE; /* carry on with the program! */
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void* appstate, SDL_AppResult result) {
  auto* state = static_cast<AppState*>(appstate);

  delete state;
}
