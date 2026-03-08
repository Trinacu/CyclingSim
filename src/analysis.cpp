#include "analysis.h"
#include "sim.h"

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

    if (end_condition && end_condition->should_stop(*sim))
      break;
  }

  for (auto* o : observers)
    o->on_finish(*sim);
}

void MetricObserver::on_step(const Simulation& sim) {
  samples.push_back({sim.get_sim_seconds(), metric(sim)});
}

TimelineObserver::TimelineObserver(std::vector<double> checkpoints,
                                   std::map<RiderId, double> offsets)
    : checkpoints(std::move(checkpoints)), start_offsets(std::move(offsets)) {
  // prime the next_checkpoint index to 0 for every rider
  for (const auto& [id, _] : start_offsets)
    next_checkpoint_idx[id] = 0;
}

void TimelineObserver::on_step(const Simulation& sim) {
  double sim_time = sim.get_sim_seconds();
  for (const auto& rider : sim.get_engine()->get_riders()) {
    RiderId id = rider->get_id();

    // lazy init
    if (next_checkpoint_idx.find(id) == next_checkpoint_idx.end())
      next_checkpoint_idx[id] = 0;

    int& idx = next_checkpoint_idx[id];
    if (idx >= (int)checkpoints.size())
      continue;

    if (rider->get_pos() >= checkpoints[idx]) {
      double offset = 0.0;
      auto it = start_offsets.find(id);
      if (it != start_offsets.end())
        offset = it->second;

      timeline[id].push_back({checkpoints[idx], sim_time - offset});
      ++idx;
    }
  }
}

const std::map<RiderId, std::vector<RiderTimelineEntry>>&
TimelineObserver::data() const {
  return timeline;
}
