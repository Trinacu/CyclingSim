// sim.h
#ifndef SIM_H
#define SIM_H

#include "course.h"
#include "rider.h"
#include <mutex>

class PhysicsEngine {
  private:
    const Course* course;
    mutable std::mutex frame_mtx;
    std::vector<Rider*> riders;

  public:
    explicit PhysicsEngine(const Course* c);
    void add_rider(Rider* r);
    void update(double dt);
    // do these returns need to/should be const?
    const std::vector<Rider*> get_riders() const;
    const Rider* get_rider(int idx) const;
    std::mutex* get_frame_mutex() const;

    ~PhysicsEngine();
};

// Your main simulation loop (runs in its own thread or fixed-step driver)
class Simulation {
  private:
    PhysicsEngine engine;
    std::atomic<bool> running{false};
    double time_factor = 1.0;
    double sim_seconds = 0.0;

  public:
    Simulation(const Course* c);

    void start();
    void stop();

    void set_time_factor(double f) { time_factor = f; }

    const double get_sim_seconds() const;
    const PhysicsEngine* get_engine() const;
    PhysicsEngine* get_engine();
};
#endif
