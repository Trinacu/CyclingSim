#define SDL_MAIN_USE_CALLBACKS 1 /* use the callbacks instead of main() */
#include "course.h"
#include "display.h"
#include "rider.h"
#include "sim.h"
#include "widget.h"
#include <SDL3/SDL.h>
#include <SDL3/SDL_main.h>
#include <thread>

struct AppState {
    Simulation* sim;
    DisplayEngine* display;
    std::thread* physics_thread;
    SDL_Window* window;
    SDL_Renderer* renderer;
    ResourceProvider* resources;
};

int SCREEN_WIDTH = 1000;
int SCREEN_HEIGHT = 640;
int WORLD_WIDTH = 400;

/* This function runs once at startup. */
SDL_AppResult SDL_AppInit(void** appstate, int argc, char* argv[]) {
    int i;
    AppState* state = new AppState();

    SDL_SetAppMetadata("Example Renderer Primitives", "1.0",
                       "com.example.renderer-primitives");

    // 1) Build physics course & sim
    Course* course = new Course({{200, 0.05f}, {1000, 0.0f}, {1000, 0.1f}});
    Simulation* sim = new Simulation(course);
    sim->set_time_factor(5.0);

    // camera isnt necessarily same screensize as window!
    Vector2d screensize(SCREEN_WIDTH, SCREEN_HEIGHT);
    Camera* camera = new Camera(course, WORLD_WIDTH, screensize);

    state->display = new DisplayEngine(sim, SCREEN_WIDTH, SCREEN_HEIGHT, camera);
    state->resources = new GameResources(state->display->get_renderer());
    state->display->set_resources(state->resources);

    // Team team = new Team("test team");
    Team team("Team1");
    Rider* r = Rider::create_generic(team, course);
    sim->get_engine()->add_rider(r);
    camera->set_target_rider(sim->get_engine()->get_rider(0));

    Rider* r2 = new Rider("Pedro", 200, 80, 0.3, Bike::create_generic(), team, course);
    r2->pos = 20;
    sim->get_engine()->add_rider(r2);

    // 4) Add your drawables (example)
    state->display->add_drawable(std::make_unique<CourseDrawable>(course));
    state->display->add_drawable(std::make_unique<RiderDrawable>());
    state->display->add_drawable(std::make_unique<Stopwatch>(
        10, 10, state->resources->get_fontManager()->get_font("stopwatch"), sim));

    state->display->add_drawable(std::make_unique<ValueField>(
        300, 300, 5, state->resources->get_fontManager()->get_font("default"), r2,
        &RiderSnapshot::pos));
    state->display->add_drawable(std::make_unique<ValueFieldPanel>(
        300, 400, state->resources->get_fontManager()->get_font("default"), r));

    // 2) Start physics thread
    state->physics_thread = new std::thread([sim = sim]() { sim->start(); });

    state->sim = sim;

    *appstate = state;
    return SDL_APP_CONTINUE; /* carry on with the program! */
}

/* This function runs when a new event (mouse input, keypresses, etc) occurs. */
SDL_AppResult SDL_AppEvent(void* appstate, SDL_Event* event) {
    auto* state = static_cast<AppState*>(appstate);
    if (event->type == SDL_EVENT_QUIT) {
        return SDL_APP_SUCCESS; /* end the program, reporting success to the OS.
                                 */
    }

    else {
        state->display->handle_event(event);
    }
    return SDL_APP_CONTINUE; /* carry on with the program! */
}

/* This function runs once per frame, and is the heart of the program. */
SDL_AppResult SDL_AppIterate(void* appstate) {
    auto* state = static_cast<AppState*>(appstate);
    // state->x += 0.5;
    // state->display->get_camera()->update(state->x);
    state->display->render_frame();

    return SDL_APP_CONTINUE; /* carry on with the program! */
}

/* This function runs once at shutdown. */
void SDL_AppQuit(void* appstate, SDL_AppResult result) {
    auto* state = static_cast<AppState*>(appstate);
    state->sim->stop();
    state->physics_thread->join();
    delete state->physics_thread;
    delete state->display;
    delete state->sim;
    SDL_DestroyWindow(state->window);
    SDL_DestroyRenderer(state->renderer);
    delete state;
    /* SDL will clean up the window/renderer for us. */
}
