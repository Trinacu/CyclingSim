#include "sim.h"
#include "rider.h"
#include <chrono>
#include <memory>
#include <thread>

PhysicsEngine::PhysicsEngine(const Course* c) : course(c) {}

void PhysicsEngine::add_rider(const RiderConfig cfg) {
  auto r = std::make_unique<Rider>(cfg);

  for (const auto& r0 : riders) {
    // now with creating riders here we can't check with uid
    // maybe add check with name + some stats?
    // if (r->get_id() == r0->get_id()) {
    //   SDL_Log("Tried to add rider who is already in the list! %s",
    //           r->name.c_str());
    //   return;
    // }
  }

  std::lock_guard<std::mutex> lock(frame_mtx);

  r->set_course(course);
  riders.push_back(std::move(r));
}

void PhysicsEngine::update(double dt) {
  for (const auto& r : riders) {
    r->update(dt);
  }
}

// Expose a way for the render thread to grab the same mutex.
// We need this so that rendering can “lock frame_mtx” before reading any Rider
// state.
std::mutex* PhysicsEngine::get_frame_mutex() const { return &frame_mtx; }

const std::vector<std::unique_ptr<Rider>>& PhysicsEngine::get_riders() const {
  return riders;
}

// this is (now) only used to set camera to first rider... kinda useless if
// fixed
const Rider* PhysicsEngine::get_rider_by_idx(int idx) const {
  return riders.at(idx).get();
}

const Rider* PhysicsEngine::get_rider_by_uid(int uid) const {
  for (auto& r : get_riders()) {
    SDL_Log("%d", r->get_uid());
    if (r->get_uid() == uid)
      return r.get();
    // TODO - do we just raise error here?
  }
  SDL_Log("uid %d did not match any rider!", uid);
  return nullptr;
}

void PhysicsEngine::set_rider_effort(int uid, double effort) {
  riders.at(uid).get()->set_effort(effort);
}

// SIMULATION

Simulation::Simulation(const Course* c) : engine(c) {}

void Simulation::start_realtime() {
  running = true;
  double accumulator = 0.0;
  double sim_step;

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

    accumulator += frame_time * time_factor;
    if (accumulator > 0.25) {
      SDL_Log("accumulator %f > 0.25 s. Setting to 0.25s.", accumulator);
      accumulator = 0.25;
    }

    while (accumulator >= dt) {
      auto step_start = std::chrono::steady_clock::now();

      try {
        step_fixed(dt);
      } catch (const std::exception& e) {
        physics_error = true;
        physics_error_message = e.what();
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
    // interp_alpha = accumulator / dt;

    // After you’ve done zero or more physics steps,
    // you can sleep a tiny bit (to release thread?)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void Simulation::set_effort_schedule(int rider_uid,
                                     std::shared_ptr<EffortSchedule> schedule) {
  effort_schedules[rider_uid] = std::move(schedule);
}

void Simulation::clear_effort_schedule(int rider_uid) {
  effort_schedules.erase(rider_uid);
}

void Simulation::step_fixed(double dt) {
  std::lock_guard<std::mutex> phys_lock(*engine.get_frame_mutex());

  double t = sim_seconds;

  for (auto& [uid, sched] : effort_schedules) {
    double effort = sched->effort_at(t);
    engine.set_rider_effort(uid, effort);
  }

  engine.update(dt);
  sim_seconds += dt;
}

// void Simulation::run_max_speed(const SimulationCondition& cond) {
//   using namespace std::chrono;
//
//   const int secs = 180;
//   const auto timeout = seconds(secs);
//   const auto start_time = steady_clock::now();
//   int iteration = 1;
//
//   while (!cond.is_met(*this)) {
//     try {
//       step_fixed(dt);
//     } catch (const std::exception& e) {
//       physics_error = true;
//       physics_error_message = e.what();
//       break;
//     }
//
//     if (++iteration % 100 == 0) {
//       if (steady_clock::now() - start_time > timeout) {
//         SDL_Log("Timeout %d seconds after %d iterations", secs, iteration);
//         break;
//       }
//     }
//   }
// }

void Simulation::pause() { paused = true; }

void Simulation::resume() { paused = false; }

bool Simulation::is_paused() const { return paused; }

void Simulation::stop() { running = false; }

const double Simulation::get_sim_seconds() const { return sim_seconds; }

// class TimeReached : public SimulationCondition {
//   double limit;
//
// public:
//   explicit TimeReached(double limit_seconds) : limit(limit_seconds) {}
//
//   bool is_met(const Simulation& sim) const override {
//     return sim.get_sim_seconds() >= limit;
//   }
// };

const PhysicsEngine* Simulation::get_engine() const { return &engine; }
PhysicsEngine* Simulation::get_engine() { return &engine; }

// MetricFn speed = [](const Simulation& sim) {
//   return sim.get_engine()->get_rider(0)->km_h();
// };
//
// MetricFn power = [](const Simulation& sim) {
//   return sim.get_engine()->get_rider(0)->get_power();
// };
