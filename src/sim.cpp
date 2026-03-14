#include "sim.h"
#include "lateral_solver.h"
#include "rider.h"
#include "snapshot.h"
#include <chrono>
#include <memory>
#include <thread>
#include <unordered_map>

PhysicsEngine::PhysicsEngine(const Course* c)
    : course(c), lateral_solver_(params) {}

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

// --- Update pipeline ---
//
// Ordering guarantee for the snapshot:
//   step_and_snapshot() calls update(dt), then fill_snapshot().
//   update() calls all four phases, including step_lateral_apply() which writes
//   new lat_pos into each Rider.
//   fill_snapshot() reads lat_pos via Rider::snapshot() after the apply.
//   Therefore the snapshot always reflects the fully-resolved lateral state for
//   the current step — no off-by-one between longitudinal and lateral.

void PhysicsEngine::update(double dt) {
  step_longitudinal(dt);
  step_lateral_behavior();
  step_lateral_solve(dt);
  step_lateral_apply();
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

// --- Behavior management ---

void PhysicsEngine::set_rider_behavior(
    RiderId id, std::shared_ptr<ILateralBehavior> behavior) {
  if (behavior)
    behaviors_[id] = std::move(behavior);
  else
    behaviors_.erase(id);
}

void PhysicsEngine::clear_rider_behavior(RiderId id) { behaviors_.erase(id); }

// Phase 1: advance each rider's longitudinal physics independently.
void PhysicsEngine::step_longitudinal(double dt) {
  for (const auto& [id, r] : riders)
    r->update(dt);
}

// Phase 2: query each assigned behavior for an optional lateral target and
// write the result into the rider.  Also builds lat_states_ for reuse by
// step_lateral_solve().
//
// Why build lat_states_ here rather than in step_lateral_solve():
//   build_context() needs to filter nearby riders from lat_states_ while
//   behaviors are running.  Building the vector first, then calling behaviors,
//   means all proximity queries are consistent within the same step.
void PhysicsEngine::step_lateral_behavior() {
  // Build the shared state snapshot from post-longitudinal rider state.
  lat_states_.clear();
  lat_states_.reserve(riders.size());

  for (const auto& [id, r] : riders) {
    lat_states_.push_back(LateralRiderState{
        .id = id,
        .lon_pos = r->get_pos(),
        .speed = r->get_speed(),
        .lat_pos = r->get_lat_pos(),
        .lat_vel = r->get_lat_vel(),
        .lat_target = r->get_lat_target(),
        .w_prime_frac = r->get_energy_fraction(),
        .surplus_power = compute_surplus_power(*r),
        .mass = r->get_total_mass(),
        .road_width = course->get_road_width(r->get_pos()),
    });
  }

  // Call each assigned behavior.
  for (const auto& [id, behavior] : behaviors_) {
    auto rider_it = riders.find(id);
    if (rider_it == riders.end())
      continue; // stale entry — rider was removed

    const LateralContext ctx = build_context(id);
    const std::optional<double> target = behavior->compute_lat_target(ctx);

    Rider& r = *rider_it->second;
    if (target.has_value())
      r.set_lat_target(target.value());
    else
      r.clear_lat_target();

    // Keep lat_states_ in sync so subsequent build_context() calls within
    // this step see the updated target.
    for (auto& s : lat_states_) {
      if (s.id == id) {
        s.lat_target = r.get_lat_target();
        break;
      }
    }
  }
}

// Phase 3: run the lateral solver over lat_states_ built in Phase 2.
void PhysicsEngine::step_lateral_solve(double dt) {
  // lat_states_ was populated (and behavior-updated) in
  // step_lateral_behavior(). If there are no assigned behaviors, it still
  // contains all riders with their current lat_target (typically nullopt) — the
  // solver handles that correctly.
  lat_updates_ = lateral_solver_.solve(lat_states_, dt);
}

// Phase 4: write solver output back into Rider objects.
//
// speed_penalty is applied inside Rider::apply_lateral_update() as
// state.speed *= speed_penalty.  This happens after the longitudinal solve
// for this step (step 1) and before the snapshot capture (fill_snapshot),
// so the penalised speed is visible in the snapshot without disturbing the
// Newton convergence that already ran.
void PhysicsEngine::step_lateral_apply() {
  for (const auto& upd : lat_updates_) {
    auto it = riders.find(upd.id);
    if (it == riders.end())
      continue;
    it->second->apply_lateral_update(upd.new_lat_pos, upd.new_lat_vel,
                                     upd.speed_penalty);
  }
}

// --- Private helpers ---

// Approximate power consumed by longitudinal resistance at current speed.
// Uses a simplified model intentionally — surplus_power is used only to size
// the shove budget, not to drive physics.  Exact values are not required.
double PhysicsEngine::compute_surplus_power(const Rider& r) const {
  const double v = r.get_speed();
  const double rho = 1.2234;
  const double g = 9.80665;
  const double cda = r.get_config().cda;
  const double m = r.get_total_mass();
  const double slope = course->get_slope(r.get_pos());

  const double P_aero = 0.5 * rho * cda * v * v * v;
  const double P_roll = 0.006 * m * g * v; // crr ≈ 0.006
  const double P_grav = m * g * slope * v;

  return std::max(0.0, r.get_power() - (P_aero + P_roll + P_grav));
}

// Build a LateralContext for one rider from the current lat_states_ snapshot.
LateralContext PhysicsEngine::build_context(RiderId id) const {
  const LateralRiderState* own = nullptr;
  for (const auto& s : lat_states_) {
    if (s.id == id) {
      own = &s;
      break;
    }
  }
  if (!own)
    return {};

  LateralContext ctx;
  ctx.own_lat_pos = own->lat_pos;
  ctx.own_lat_vel = own->lat_vel;
  ctx.own_speed = own->speed;
  ctx.own_w_prime_frac = own->w_prime_frac;
  ctx.road_width = own->road_width;

  for (const auto& s : lat_states_) {
    if (s.id == id)
      continue;
    const double lon_offset = s.lon_pos - own->lon_pos;
    if (std::fabs(lon_offset) <= params.x_lookahead) {
      ctx.nearby.push_back(NearbyRider{
          .lon_offset = lon_offset,
          .lat_pos = s.lat_pos,
          .speed = s.speed,
          .w_prime_frac = s.w_prime_frac,
      });
    }
  }

  return ctx;
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
