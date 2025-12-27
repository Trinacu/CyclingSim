#include "analysis.h"
#include "sim.h"

void MetricObserver::on_step(const Simulation& sim) {
  samples.push_back({sim.get_sim_seconds(), metric(sim)});
}

OfflineSimulationRunner::OfflineSimulationRunner(std::unique_ptr<Simulation> s)
    : sim(std::move(s)) {}

void OfflineSimulationRunner::add_observer(SimulationObserver* obs) {
  observers.push_back(obs);
}

void OfflineSimulationRunner::set_end_condition(
    std::unique_ptr<SimulationEndCondition> cond) {
  end_condition = std::move(cond);
}

void OfflineSimulationRunner::run() {
  for (auto* o : observers)
    o->on_start(*sim);

  while (true) {
    sim->step_fixed(sim->get_dt());

    for (auto* o : observers)
      o->on_step(*sim);

    // bool stop = false;
    // for (auto* o : observers) {
    //   if (o->should_stop(*sim)) {
    //     stop = true;
    //     break;
    //   }
    // }
    if (end_condition && end_condition->should_stop(*sim))
      break;
  }

  for (auto* o : observers)
    o->on_finish(*sim);
}
