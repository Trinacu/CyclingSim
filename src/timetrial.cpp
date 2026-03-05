#include "timetrial.h"
#include "analysis.h"
#include "sim.h"

static const double RACE_WINDOW = 7200.0; // 2h ceiling

std::map<RiderId, double>
build_start_offsets(const std::vector<RiderConfig>& ridercfgs,
                    double gap_seconds) {
  std::map<RiderId, double> offsets;
  for (int i = 0; i < (int)ridercfgs.size(); ++i)
    offsets[ridercfgs[i].rider_id] = i * gap_seconds;
  return offsets;
}

void setup_tt_schedules(Simulation* sim,
                        const std::vector<RiderConfig>& ridercfgs,
                        const std::map<RiderId, double>& start_offsets) {
  for (const auto& cfg : ridercfgs) {
    double offset = start_offsets.at(cfg.rider_id);
    if (offset > 0.0) {
      sim->set_effort_schedule(
          cfg.rider_id,
          std::make_shared<StepEffortSchedule>(std::vector<EffortBlock>{
              {offset, 0.0},      // waiting at start gate
              {RACE_WINDOW, 1.0}, // racing
          }));
    } else {
      sim->set_effort_schedule(
          cfg.rider_id,
          std::make_shared<StepEffortSchedule>(std::vector<EffortBlock>{
              {RACE_WINDOW, 1.0},
          }));
    }
  }
}

TimeTrialResult run_time_trial(const Course& course,
                               const std::vector<RiderConfig>& ridercfgs,
                               double start_gap_seconds) {
  // --- Build checkpoints: 25%, 50%, 75%, finish ---
  const double length = course.get_total_length();
  std::vector<double> checkpoints = {
      length * 0.25,
      length * 0.50,
      length * 0.75,
      length,
  };

  // --- Assign start offsets: rider[0] goes at t=0, rider[1] at t=gap, etc. ---
  auto start_offsets = build_start_offsets(ridercfgs, start_gap_seconds);

  // --- Build simulation ---
  auto sim = std::make_unique<Simulation>(&course);
  sim->set_dt(0.1);
  sim->add_riders(ridercfgs);
  setup_tt_schedules(sim.get(), ridercfgs, start_offsets);

  // --- Attach timeline observer ---
  TimelineObserver timeline_obs(checkpoints, start_offsets);

  OfflineSimulationRunner runner(std::move(sim));
  runner.add_observer(&timeline_obs);
  runner.set_end_condition(std::make_unique<FinishLineCondition>());
  runner.run();

  // --- Assemble results ---
  const auto& raw = timeline_obs.data();

  TimeTrialResult result;
  result.checkpoint_distances = checkpoints;

  for (const auto& cfg : ridercfgs) {
    RiderTimeTrialResult r;
    r.rider_id = cfg.rider_id;
    r.name = cfg.name;

    auto it = raw.find(cfg.rider_id);
    if (it != raw.end()) {
      r.timeline = it->second;
    }
    result.riders.push_back(std::move(r));
  }

  // Sort by finish time (last checkpoint = finish line).
  // Riders with incomplete timelines (DNF) sort to the end.
  std::sort(result.riders.begin(), result.riders.end(),
            [&](const RiderTimeTrialResult& a, const RiderTimeTrialResult& b) {
              bool a_finished = !a.timeline.empty() &&
                                a.timeline.back().checkpoint_distance >= length;
              bool b_finished = !b.timeline.empty() &&
                                b.timeline.back().checkpoint_distance >= length;
              if (a_finished && b_finished)
                return a.timeline.back().race_time <
                       b.timeline.back().race_time;
              if (a_finished)
                return true;
              return false;
            });

  return result;
}
