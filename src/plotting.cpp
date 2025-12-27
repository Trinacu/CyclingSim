#include "plotting.h"
#include "sim.h"

PlotResult run_plot_simulation(const Course& course,
                               const std::vector<RiderConfig>& riders,
                               int target_uid) {
  auto sim = std::make_unique<Simulation>(&course);

  for (const auto& cfg : riders)
    sim->get_engine()->add_rider(cfg);

  auto schedule = std::make_shared<StepEffortSchedule>(std::vector<EffortBlock>{
      {0.0, 60.0, 0.1},    // easy
      {60.0, 120.0, 1.3},  // hard
      {120.0, 240.0, 0.8}, // recovery
      {240.0, 300.0, 1.5}  // sprint
  });

  sim->set_effort_schedule(0, schedule);

  OfflineSimulationRunner runner(std::move(sim));

  MetricObserver speed_obs([target_uid](const Simulation& s) {
    const Rider* r = s.get_engine()->get_rider_by_idx(0);
    return r ? r->km_h() : 0.0;
  });

  MetricObserver wbal_obs([target_uid](const Simulation& sim) {
    const Rider* r = sim.get_engine()->get_rider_by_idx(0);
    return r->get_energy();
  });

  runner.add_observer(&speed_obs);
  runner.add_observer(&wbal_obs);
  runner.set_end_condition(std::make_unique<FinishLineCondition>());
  runner.run();

  PlotResult result;

  result.series.push_back(
      {.label = "Speed (km/h)", .samples = speed_obs.data(), .y_axis = 0});

  result.series.push_back(
      {.label = "W'bal (J)", .samples = wbal_obs.data(), .y_axis = 1});

  return result;
}
