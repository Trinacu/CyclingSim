// sim.h
#ifndef SIM_H
#define SIM_H

#include "course.h"
#include "rider.h"
#include <mutex>

#include <atomic>
#include <vector>

class PhysicsEngine {
private:
  const Course* course;
  mutable std::mutex frame_mtx;
  std::vector<std::unique_ptr<Rider>> riders;

public:
  explicit PhysicsEngine(const Course* c);
  void add_rider(const RiderConfig cfg);
  void update(double dt);

  const Course* get_course() const { return course; }
  const double get_course_length() const { return course->get_total_length(); }

  // do these returns need to/should be const?
  const std::vector<std::unique_ptr<Rider>>& get_riders() const;
  const Rider* get_rider(int idx) const;
  const Rider* get_rider_by_uid(int uid) const;
  std::mutex* get_frame_mutex() const;

  void set_rider_effort(int uid, double effort);

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

  const float dt = 0.1; // 100 Hz physics

public:
  Simulation(const Course* c);

  void start_realtime();

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

  std::atomic<bool> physics_error{false};
  std::string physics_error_message;
};

class SimulationObserver {
public:
  virtual ~SimulationObserver() = default;

  // Called every fixed step
  virtual void on_step(const Simulation& sim) = 0;

  // Called once at start (optional)
  virtual void on_start(const Simulation& sim) {}

  // Called once at end (optional)
  virtual void on_finish(const Simulation& sim) {}

  // Return true to stop simulation early
  virtual bool should_stop(const Simulation& sim) const { return false; }
};

struct PlotSample {
  double time;
  double value;
};

using MetricFn = std::function<double(const Simulation&)>;

class MetricObserver : public SimulationObserver {
public:
  MetricObserver(MetricFn fn) : metric(fn) {}

  void on_step(const Simulation& sim) override {
    samples.push_back({sim.get_sim_seconds(), metric(sim)});
  }

  const auto& data() const { return samples; }

private:
  MetricFn metric;
  std::vector<PlotSample> samples;
};

struct PlotResult {
  std::vector<PlotSample> samples;
};

class RiderValuePlotObserver : public SimulationObserver {
public:
  RiderValuePlotObserver(size_t rider_uid_) : rider_uid(rider_uid_) {}

  void on_step(const Simulation& sim) override {
    const Rider* r = sim.get_engine()->get_rider(rider_uid);
    samples.push_back({
        sim.get_sim_seconds(),
        r->km_h() // or power, cadence, etc.
    });
  }

  const std::vector<PlotSample>& data() const { return samples; }

private:
  size_t rider_uid;
  std::vector<PlotSample> samples;
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
        SDL_Log("%s: %.1f", r->name.c_str(), r->pos);
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

class OfflineSimulationRunner {
public:
  OfflineSimulationRunner(std::unique_ptr<Simulation> sim);

  void add_observer(SimulationObserver* obs);
  void set_end_condition(std::unique_ptr<SimulationEndCondition> cond);

  void run();

private:
  std::unique_ptr<Simulation> sim;
  std::vector<SimulationObserver*> observers;
  std::unique_ptr<SimulationEndCondition> end_condition;
};

PlotResult run_plot_simulation(const Course& course,
                               const std::vector<RiderConfig>& riders,
                               int target_uid);
#endif
