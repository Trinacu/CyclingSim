#include "sim.h"
#include <thread>

PhysicsEngine::PhysicsEngine(const Course *c) : course(c) {}

void PhysicsEngine::add_rider(Rider *r) {
  std::lock_guard<std::mutex> lock(frame_mtx);
  riders.push_back(r);
}

void PhysicsEngine::update(double dt) {
  for (Rider *r : riders) {
    r->update(dt);
    // std::cout << "pos: " << r->pos << "\tm\nspeed: " << 3.6 * r->speed <<
    // "\tkm/h\n" << std::endl;
  }
}

// Expose a way for the render thread to grab the same mutex.
// We need this so that rendering can “lock frame_mtx” before reading any Rider
// state.
std::mutex *PhysicsEngine::get_frame_mutex() const { return &frame_mtx; }

const std::vector<Rider *> PhysicsEngine::get_riders() const { return riders; }

const Rider *PhysicsEngine::get_rider(int idx) const { return riders.at(idx); }

PhysicsEngine::~PhysicsEngine() {
  for (Rider *r : riders) {
    delete r;
  }
}

Simulation::Simulation(const Course *c) : engine(c) {}

void Simulation::start() {
  running = true;
  const float dt = 0.1; // 10 Hz physics
  double accumulator = 0.0;
  double sim_step;
  auto t_prev = std::chrono::steady_clock::now();

  while (running) {
    auto t_now = std::chrono::steady_clock::now();
    double frame_time = std::chrono::duration<double>(t_now - t_prev).count();
    t_prev = t_now;

    sim_step = frame_time * time_factor;
    accumulator += sim_step;
    sim_seconds += sim_step;

    while (accumulator >= dt) {
      auto step_start = std::chrono::steady_clock::now();
      {
        std::lock_guard<std::mutex> phys_lock(*engine.get_frame_mutex());
        engine.update(dt);
      }
      accumulator -= dt;

      // what follows is only to check for exceeding the time
      auto step_end = std::chrono::steady_clock::now();
      double step_time =
          std::chrono::duration<double>(step_end - step_start).count();
      if (step_time > dt) {
        SDL_Log("Hey! engine.update(%f) took %f. spiral of death!", dt,
                step_time);
      }
    }
    // After you’ve done zero or more physics steps,
    // you can sleep a tiny bit (to release thread?)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void Simulation::stop() { running = false; }

const double Simulation::get_sim_seconds() const { return sim_seconds; }

const PhysicsEngine *Simulation::get_engine() const { return &engine; }
PhysicsEngine *Simulation::get_engine() { return &engine; }
