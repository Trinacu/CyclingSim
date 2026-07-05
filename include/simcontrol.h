// simcontrol.h
#ifndef SIMCONTROL_H
#define SIMCONTROL_H

#include "mytypes.h"

// Narrow write-interface the UI layer uses to control the simulation.
//
// Widgets read sim state exclusively through RenderContext snapshots and
// write exclusively through this interface — they never see sim.h.
// Simulation implements it by queueing commands that the physics thread
// drains at the start of each fixed step, so all sim mutation stays
// single-threaded.
class ISimControl {
public:
  virtual ~ISimControl() = default;

  virtual void set_rider_effort(RiderId id, double effort) = 0;
  virtual void set_time_factor(double f) = 0;
  virtual void pause() = 0;
  virtual void resume() = 0;
  virtual bool is_paused() const = 0;
};

#endif
