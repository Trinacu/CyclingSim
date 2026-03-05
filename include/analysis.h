#include "rider.h"
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

class Simulation;
class SimulationEndCondition;

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

struct PlotSample {
  double x;
  double y;
};

struct PlotSeries {
  std::string label;
  std::vector<PlotSample> samples;
  int y_axis = 0; // 0 = left, 1 = right
};

using MetricFn = std::function<double(const Simulation&)>;

class MetricObserver : public SimulationObserver {
public:
  MetricObserver(MetricFn fn) : metric(fn) {}

  void on_step(const Simulation& sim) override;

  const auto& data() const { return samples; }

private:
  MetricFn metric;
  std::vector<PlotSample> samples;
};

struct PlotResult {
  std::string title = "";
  std::vector<PlotSeries> series;
};

struct RiderTimelineEntry {
  double
      checkpoint_distance; // which gate (0.0 = start, course_length = finish)
  double race_time;        // sim_time - rider's start_offset
};

class TimelineObserver : public SimulationObserver {
public:
  // checkpoint_distances: sorted list of positions to record, including finish
  // start_offsets: map of rider_id -> sim_time when that rider was released
  TimelineObserver(std::vector<double> checkpoints,
                   std::map<RiderId, double> start_offsets);

  void on_step(const Simulation& sim) override;

  const std::map<RiderId, std::vector<RiderTimelineEntry>>& data() const;

private:
  std::vector<double> checkpoints;
  std::map<RiderId, double> start_offsets;
  std::map<RiderId, int> next_checkpoint_idx; // tracks progress per rider
  std::map<RiderId, std::vector<RiderTimelineEntry>> timeline;
};
