// realtime_runner.h
#ifndef REALTIME_RUNNER_H
#define REALTIME_RUNNER_H

#include "simcontrol.h"
#include <atomic>
#include <string>
#include <thread>

class Simulation;

// Realtime thread driver for a passive Simulation: owns the physics thread,
// the pacing accumulator, pause state and error reporting.  Symmetric with
// OfflineSimulationRunner (analysis.h), which drives the same Simulation
// as fast as possible on the calling thread.
//
// The UI talks to the runner (via ISimControl), never to Simulation directly;
// rider/time-factor writes are delegated to Simulation's command queue, which
// step_fixed() drains at step boundaries.
class RealtimeSimRunner : public ISimControl {
public:
  explicit RealtimeSimRunner(Simulation* sim);
  ~RealtimeSimRunner(); // calls stop()

  RealtimeSimRunner(const RealtimeSimRunner&) = delete;
  RealtimeSimRunner& operator=(const RealtimeSimRunner&) = delete;

  void start(); // spawns the physics thread; no-op if already running
  void stop();  // signals the loop and joins the thread

  // ISimControl
  void set_rider_effort(RiderId id, double effort) override;
  void set_time_factor(double f) override;
  void pause() override;
  void resume() override;
  bool is_paused() const override;
  void toggle_pause();

  // Written by the physics thread (message first, then flag), read by the
  // render loop.
  std::atomic<bool> physics_error{false};
  std::string physics_error_message;

private:
  void loop();

  Simulation* sim; // not owned
  std::thread thread_;
  std::atomic<bool> running{false};
  std::atomic<bool> paused{false};
};

#endif
