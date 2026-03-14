// sim.h
#ifndef SIM_H
#define SIM_H

#include "collision_params.h"
#include "course.h"
#include "effortschedule.h"
#include "lateral_behavior.h"
#include "lateral_solver.h"
#include "rider.h"
#include "snapshot.h"
#include <memory>
#include <mutex>

#include <atomic>
#include <unordered_map>
#include <vector>

class PhysicsEngine {
private:
  const Course* course;
  mutable std::mutex frame_mtx;
  std::unordered_map<RiderId, std::unique_ptr<Rider>> riders;

  void fill_snapshot(FrameSnapshot& out) const;

  CollisionParams params;
  LateralSolver lateral_solver_; // stateless; holds a copy of params

  // Optional per-rider behavior: absent → purely force-driven (lat_target
  // nullopt).
  // shared_ptr because behavior instances may be shared across riders
  // (e.g. every team member uses the same BlockBehavior instance).
  std::unordered_map<RiderId, std::shared_ptr<ILateralBehavior>> behaviors_;

  // lat_states_ is built once in step_lateral_behavior() and reused by
  // step_lateral_solve().  lat_updates_ is written by the solver and consumed
  // by step_lateral_apply().
  std::vector<LateralRiderState> lat_states_;
  std::vector<LateralUpdate> lat_updates_;

  void step_longitudinal(double dt);
  void step_lateral_behavior();       // builds lat_states_, queries behaviors
  void step_lateral_solve(double dt); // calls lateral_solver_.solve()
  void step_lateral_apply();          // writes lat_updates_ back into riders

  // Approximate surplus power: rider output minus resistive losses at current
  // speed.  Used to populate LateralRiderState::surplus_power.
  // Intentionally a rough estimate — it is used only to size the shove budget,
  // not to drive longitudinal physics.
  double compute_surplus_power(const Rider& r) const;

  // Build a LateralContext for one rider from the current lat_states_ snapshot.
  // Nearby riders are filtered to those within params.x_lookahead.
  LateralContext build_context(RiderId id) const;

public:
  explicit PhysicsEngine(const Course* c);
  bool add_rider(const RiderConfig cfg);
  void update(double dt);

  const Course* get_course() const { return course; }
  const double get_course_length() const { return course->get_total_length(); }

  // do these returns need to/should be const?
  const std::unordered_map<RiderId, std::unique_ptr<Rider>>& get_riders() const;
  const Rider* get_rider_by_id(RiderId id) const;

  // physicsengine mutates rider state
  void set_rider_effort(int id, double effort);

  void step_and_snapshot(double dt, FrameSnapshot& out);

  // Replaces any previously assigned behavior.  nullptr → clear_rider_behavior.
  void set_rider_behavior(RiderId id,
                          std::shared_ptr<ILateralBehavior> behavior);
  void clear_rider_behavior(RiderId id);

  ~PhysicsEngine() = default;
};

// forward declare
class SimulationCondition;

// Your main simulation loop (runs in its own thread or fixed-step driver)
class Simulation {
private:
  PhysicsEngine engine;
  // when this is false, we exit and kill the thread
  std::atomic<bool> running{false};
  // this is different because it pauses
  std::atomic<bool> paused{false};

  double time_factor = 1.0;
  double sim_seconds = 0.0;

  double interp_alpha = 0.0;

  double dt = 0.01; // 100 Hz physics

  std::unordered_map<int, std::shared_ptr<EffortSchedule>> effort_schedules;

  // Double buffers
  FrameSnapshot snap_prev; // published previous
  FrameSnapshot snap_curr; // published current
  FrameSnapshot snap_back; // build buffer
                           //
  // Called at the end of step_fixed(), while frame_mtx is still held
  void publish_snapshot();

  mutable std::mutex snapshot_swap_mtx;

public:
  Simulation(const Course* c);

  void start_realtime();

  void add_riders(const std::vector<RiderConfig>& configs);

  void run_max_speed(const SimulationCondition& cond);

  void pause();
  void resume();
  bool is_paused() const;
  void stop();

  void reset();

  void step_fixed(double dt);

  void set_time_factor(double f) { time_factor = f; }

  double get_dt() { return dt; }
  void set_dt(double dt_) { dt = dt_; }
  double get_interp_alpha() { return interp_alpha; }

  const double get_sim_seconds() const;
  const PhysicsEngine* get_engine() const;
  PhysicsEngine* get_engine();

  std::atomic<bool> physics_error{false};
  std::string physics_error_message;

  // Simulation decides when and why the mutation happens
  void set_effort_schedule(int rider_id,
                           std::shared_ptr<EffortSchedule> schedule);
  void clear_effort_schedule(RiderId rider_id);
  void set_rider_effort(RiderId rider_id, double effort);

  // Called by the renderer each frame; returns false if no new frame
  bool consume_latest_frame_pair(FrameSnapshot& out_prev,
                                 FrameSnapshot& out_curr);
};

class SimulationEndCondition {
public:
  virtual ~SimulationEndCondition() = default;
  virtual bool should_stop(const Simulation& sim) const = 0;
};

class FinishLineCondition : public SimulationEndCondition {
public:
  FinishLineCondition() {}

  bool should_stop(const Simulation& sim) const override {
    for (const auto& [id, r] : sim.get_engine()->get_riders()) {
      if (!r->finished()) {
        // SDL_Log("%s: %.1f", r->name.c_str(), r->pos);
        return false;
      }
    }
    SDL_Log("all riders finished");
    return true;
  }
};

class TimeLimitCondition : public SimulationEndCondition {
public:
  TimeLimitCondition(double max_time) : t(max_time) {}

  bool should_stop(const Simulation& sim) const override {
    return sim.get_sim_seconds() >= t;
  }

private:
  double t;
};

class OrEndCondition : public SimulationEndCondition {
public:
  OrEndCondition(std::unique_ptr<SimulationEndCondition> a,
                 std::unique_ptr<SimulationEndCondition> b)
      : lhs(std::move(a)), rhs(std::move(b)) {}

  bool should_stop(const Simulation& sim) const override {
    return lhs->should_stop(sim) || rhs->should_stop(sim);
  }

private:
  std::unique_ptr<SimulationEndCondition> lhs, rhs;
};

#endif
