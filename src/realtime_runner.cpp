// src/realtime_runner.cpp
#include "realtime_runner.h"
#include "sim.h"
#include <SDL3/SDL_log.h>
#include <chrono>

RealtimeSimRunner::RealtimeSimRunner(Simulation* sim) : sim(sim) {}

RealtimeSimRunner::~RealtimeSimRunner() { stop(); }

void RealtimeSimRunner::start() {
  if (thread_.joinable())
    return; // already running

  paused = false;
  physics_error = false;
  physics_error_message.clear();

  running = true;
  thread_ = std::thread([this]() { loop(); });
}

void RealtimeSimRunner::stop() {
  running = false;
  if (thread_.joinable())
    thread_.join();
}

void RealtimeSimRunner::set_rider_effort(RiderId id, double effort) {
  sim->set_rider_effort(id, effort);
}

void RealtimeSimRunner::set_time_factor(double f) { sim->set_time_factor(f); }

void RealtimeSimRunner::pause() { paused = true; }

void RealtimeSimRunner::resume() { paused = false; }

bool RealtimeSimRunner::is_paused() const { return paused; }

void RealtimeSimRunner::toggle_pause() { paused = !paused; }

void RealtimeSimRunner::loop() {
  double accumulator = 0.0;

  auto t_prev = std::chrono::steady_clock::now();

  while (running) {
    if (paused) {
      t_prev = std::chrono::steady_clock::now();
      std::this_thread::sleep_for(std::chrono::milliseconds(1));
      continue;
    }

    auto t_now = std::chrono::steady_clock::now();
    double frame_time = std::chrono::duration<double>(t_now - t_prev).count();
    t_prev = t_now;

    accumulator += frame_time * sim->get_time_factor();
    if (accumulator > 0.25) {
      SDL_Log("accumulator %f > 0.25 s. Setting to 0.25s.", accumulator);
      accumulator = 0.25;
    }

    const double dt = sim->get_dt();
    while (accumulator >= dt) {
      auto step_start = std::chrono::steady_clock::now();

      try {
        sim->step_fixed(dt);
      } catch (const std::exception& e) {
        // write message before setting flag, due to race condition
        physics_error_message = e.what();
        physics_error = true;
        running = false;
      }
      accumulator -= dt;

      // what follows is only to check for exceeding the time
      auto step_end = std::chrono::steady_clock::now();
      double step_time =
          std::chrono::duration<double>(step_end - step_start).count();
      if (step_time > dt) {
        SDL_Log("Hey! engine.update(dt=%f) took %f. spiral of death!", dt,
                step_time);
      }
    }

    // After you’ve done zero or more physics steps,
    // you can sleep a tiny bit (to release thread?)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}
