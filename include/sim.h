// sim.h
#ifndef SIM_H
#define SIM_H

#include "course.h"
#include "effortschedule.h"
#include "rider.h"
#include "simulationrenderer.h"
#include <mutex>

#include <atomic>
#include <unordered_map>
#include <vector>

class IRiderDataSource {
public:
  virtual ~IRiderDataSource() = default;

  virtual const RiderSnapshot* get_rider_snapshot(RiderId id) const = 0;
};

class PhysicsEngine {
private:
  const Course* course;
  mutable std::mutex frame_mtx;
  std::vector<std::unique_ptr<Rider>> riders;

public:
  explicit PhysicsEngine(const Course* c);
  RiderUid add_rider(const RiderConfig cfg);
  void update(double dt);

  const Course* get_course() const { return course; }
  const double get_course_length() const { return course->get_total_length(); }

  // do these returns need to/should be const?
  const std::vector<std::unique_ptr<Rider>>& get_riders() const;
  const Rider* get_rider_by_id(RiderId id) const;
  // const Rider* get_rider_by_uid(RiderUid uid) const;
  std::mutex* get_frame_mutex() const;

  // physicsengine mutates rider state
  void set_rider_effort(int id, double effort);

  ~PhysicsEngine() = default;
};

// forward declare
class SimulationCondition;

// Your main simulation loop (runs in its own thread or fixed-step driver)
class Simulation : public IRiderDataSource {
private:
  PhysicsEngine engine;
  // when this is false, we exit and kill the thread
  std::atomic<bool> running{false};
  // this is different because it pauses
  std::atomic<bool> paused{false};

  double time_factor = 1.0;
  double sim_seconds = 0.0;

  double interp_alpha = 0.0;

  const float dt = 0.0001; // 100 Hz physics

  std::unordered_map<int, std::shared_ptr<EffortSchedule>> effort_schedules;
  std::unordered_map<RiderId, RiderUid> rider_id_to_uid;

  const ISnapshotSource* snapshot_source = nullptr;

public:
  Simulation(const Course* c);

  void start_realtime();

  void add_riders(const std::vector<RiderConfig>& configs);

  void run_max_speed(const SimulationCondition& cond);

  void pause();
  void resume();
  bool is_paused() const;
  void stop();

  void step_fixed(double dt);

  double get_time_factor() { return time_factor; }
  void set_time_factor(double f) { time_factor = f; }

  double get_dt() { return dt; }
  double get_interp_alpha() { return interp_alpha; }

  const double get_sim_seconds() const;
  const PhysicsEngine* get_engine() const;
  PhysicsEngine* get_engine();

  RiderId resolve_rider_id(RiderUid uid) const;

  std::atomic<bool> physics_error{false};
  std::string physics_error_message;

  // Simulation decides when and why the mutation happens
  void set_effort_schedule(int rider_id,
                           std::shared_ptr<EffortSchedule> schedule);
  void clear_effort_schedule(int rider_id);

  void set_rider_effort(RiderId rider_id, double effort);

  void set_snapshot_source(const ISnapshotSource* src);
  const RiderSnapshot* get_rider_snapshot(RiderId id) const override;
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
    for (const auto& r : sim.get_engine()->get_riders()) {
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
