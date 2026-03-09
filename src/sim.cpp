#include "sim.h"
#include "rider.h"
#include "snapshot.h"
#include <chrono>
#include <memory>
#include <thread>
#include <unordered_map>

PhysicsEngine::PhysicsEngine(const Course* c) : course(c) {}

bool PhysicsEngine::add_rider(const RiderConfig cfg) {
  auto r = std::make_unique<Rider>(cfg);

  // could even raise here?
  if (riders.count(r->get_id()) > 0) {
    SDL_Log("PhysicsEngine::add_rider: Tried to add rider who is already in "
            "the list! %s",
            r->name.c_str());
    return false;
  }
  std::lock_guard<std::mutex> lock(frame_mtx);
  r->set_course(course);
  riders.emplace(cfg.rider_id, std::move(r));
  return true;
}

void PhysicsEngine::update(double dt) {
  for (const auto& [id, r] : riders) {
    r->update(dt);
  }
}

void PhysicsEngine::step_and_snapshot(double dt, FrameSnapshot& out) {
  std::lock_guard<std::mutex> lock(frame_mtx);
  update(dt);
  fill_snapshot(out);
}

void PhysicsEngine::fill_snapshot(FrameSnapshot& out) const {
  // this needs to be called under phys_lock, but we lock in sim::step_fixed
  out.riders.clear();
  for (const auto& [id, r] : riders)
    out.riders.emplace(id, r->snapshot());
}

const std::unordered_map<RiderId, std::unique_ptr<Rider>>&
PhysicsEngine::get_riders() const {
  return riders;
}

// this is (now) only used to set camera to first rider... kinda useless if
// fixed
const Rider* PhysicsEngine::get_rider_by_id(RiderId id) const {
  auto it = riders.find(id);
  if (it == riders.end()) {
    SDL_Log("Engine::get_rider_by_id: id %d not found", id);
    return nullptr;
  }
  return it->second.get();
}

void PhysicsEngine::set_rider_effort(int id, double effort) {
  auto it = riders.find(id);
  if (it == riders.end()) {
    SDL_Log("Engine::set_rider_effort: id %d not found", id);
    return;
  }
  it->second->set_effort(effort);
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
        // write messagae before setting flag, due to race condition
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
    // interp_alpha = accumulator / dt;

    // After you’ve done zero or more physics steps,
    // you can sleep a tiny bit (to release thread?)
    std::this_thread::sleep_for(std::chrono::milliseconds(1));
  }
}

void Simulation::add_riders(const std::vector<RiderConfig>& configs) {
  for (const auto& cfg : configs) {
    assert(cfg.rider_id >= 0);
    if (!engine.add_rider(cfg))
      SDL_Log("add_riders: engine rejected rider_id %d", cfg.rider_id);
  }
}

void Simulation::publish_snapshot() {
  snap_back.real_time = SDL_GetTicks() / 1000.0;

  std::scoped_lock lock(snapshot_swap_mtx);

  if (snap_back.sim_time <= snap_curr.sim_time)
    return; // physics didn't advance (shouldn't happen in step_fixed, but safe)

  snap_prev = std::move(snap_curr);
  snap_curr = std::move(snap_back);
}

bool Simulation::consume_latest_frame_pair(FrameSnapshot& out_prev,
                                           FrameSnapshot& out_curr) {
  std::scoped_lock lock(snapshot_swap_mtx);

  if (snap_curr.sim_time < 0.0)
    return false; // nothing published yet

  out_prev = snap_prev;
  out_curr = snap_curr;
  return true;
}

void Simulation::step_fixed(double dt) {
  for (auto& [id, sched] : effort_schedules)
    engine.set_rider_effort(id, sched->effort_at(sim_seconds));

  engine.step_and_snapshot(dt, snap_back);

  sim_seconds += dt;
  snap_back.sim_time = sim_seconds;
  snap_back.sim_dt = dt;
  snap_back.time_factor = time_factor;

  publish_snapshot(); // acquires snapshot_swap_mtx
}

void Simulation::set_effort_schedule(int rider_id,
                                     std::shared_ptr<EffortSchedule> schedule) {
  effort_schedules[rider_id] = std::move(schedule);
}

void Simulation::clear_effort_schedule(RiderId rider_id) {
  effort_schedules.erase(rider_id);
}

void Simulation::set_rider_effort(RiderId rider_id, double effort) {
  engine.set_rider_effort(rider_id, effort);
}

void Simulation::pause() { paused = true; }

void Simulation::resume() { paused = false; }

bool Simulation::is_paused() const { return paused; }

void Simulation::stop() { running = false; }

// Must be called only while the physics thread is stopped (after sim->stop()
// and thread join). No locking needed because there's no concurrent access.
void Simulation::reset() {
  sim_seconds = 0.0;
  paused = false;
  physics_error = false;
  physics_error_message.clear();
  effort_schedules.clear();

  // get_riders() returns const ref, but unique_ptr<Rider> still lets us
  // call non-const methods on the Rider through the pointer.
  for (const auto& [id, r] : engine.get_riders())
    r->reset();
}

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
