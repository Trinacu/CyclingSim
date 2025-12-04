// sim.h
#ifndef SIM_H
#define SIM_H

#include "course.h"
#include "rider.h"
#include <mutex>

#include <atomic>

class PhysicsEngine {
private:
  const Course* course;
  mutable std::mutex frame_mtx;
  std::vector<Rider*> riders;

public:
  explicit PhysicsEngine(const Course* c);
  void add_rider(Rider* r);
  void update(double dt);

  const Course* get_course() const { return course; }
  double get_course_length() const { return course->total_length; }

  // do these returns need to/should be const?
  const std::vector<Rider*>& get_riders() const;
  const Rider* get_rider(int idx) const;
  std::mutex* get_frame_mutex() const;

  ~PhysicsEngine();
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

  const float dt = 0.1; // 100 Hz physics

  void step_fixed(double dt);

public:
  Simulation(const Course* c);

  void start_realtime();

  void run_max_speed(const SimulationCondition& cond);

  void pause();
  void resume();
  bool is_paused() const;
  void stop();

  double get_time_factor() { return time_factor; }
  void set_time_factor(double f) { time_factor = f; }

  const double get_sim_seconds() const;
  const PhysicsEngine* get_engine() const;
  PhysicsEngine* get_engine();

  std::atomic<bool> physics_error{false};
  std::string physics_error_message;
};

class SimulationCondition {
public:
  virtual ~SimulationCondition() = default;
  virtual bool is_met(const Simulation& sim) const = 0;
};

#endif
