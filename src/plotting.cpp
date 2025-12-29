#include "plotting.h"
#include "sim.h"

PlotResult run_plot_simulation(const Course& course,
                               const std::vector<RiderConfig>& riders,
                               int target_uid) {
  auto sim = std::make_unique<Simulation>(&course);

  sim->add_riders(riders);

  auto schedule = std::make_shared<StepEffortSchedule>(std::vector<EffortBlock>{
      {1200.0, 1.2}, // easy
      {300.0, 0.8},  // hard
      {240.0, 0.2},  // recovery
      {60.0, 2},
      {60.0, 0.5} // sprint
  });

  RiderId rider_id = 0;
  std::string rider_name = sim->get_engine()->get_rider_by_id(rider_id)->name;

  sim->set_effort_schedule(rider_id, schedule);

  OfflineSimulationRunner runner(std::move(sim));

  MetricObserver effort_obs([target_uid, rider_id](const Simulation& s) {
    const Rider* r = s.get_engine()->get_rider_by_id(rider_id);
    return r ? r->target_effort : 0.0;
  });

  MetricObserver effortlimit_obs([target_uid, rider_id](const Simulation& s) {
    const Rider* r = s.get_engine()->get_rider_by_id(rider_id);
    return r ? r->get_effort_limit() : 0.0;
  });

  MetricObserver speed_obs([target_uid, rider_id](const Simulation& s) {
    const Rider* r = s.get_engine()->get_rider_by_id(rider_id);
    return r ? r->get_km_h() : 0.0;
  });

  MetricObserver wbal_fraction_obs(
      [target_uid, rider_id](const Simulation& sim) {
        const Rider* r = sim.get_engine()->get_rider_by_id(rider_id);
        return r->get_energy_fraction();
      });

  runner.add_observer(&effort_obs);
  runner.add_observer(&effortlimit_obs);
  runner.add_observer(&speed_obs);
  runner.add_observer(&wbal_fraction_obs);
  runner.set_end_condition(std::make_unique<FinishLineCondition>());
  runner.run();

  PlotResult result;

  result.title = rider_name;

  result.series.push_back({.label = "Effort limit (%)",
                           .samples = effortlimit_obs.data(),
                           .y_axis = 0});

  result.series.push_back(
      {.label = "Effort (%)", .samples = effort_obs.data(), .y_axis = 0});

  result.series.push_back(
      {.label = "Speed (km/h)", .samples = speed_obs.data(), .y_axis = 0});

  result.series.push_back(
      {.label = "W'bal (%)", .samples = wbal_fraction_obs.data(), .y_axis = 1});

  return result;
}
