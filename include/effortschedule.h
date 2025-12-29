#include <vector>

class EffortSchedule {
public:
  virtual ~EffortSchedule() = default;

  // t = simulation time in seconds
  virtual double effort_at(double t) const = 0;
};

struct EffortBlock {
  double duration;
  double effort; // relative (e.g. 0.7, 1.2, 1.5)
};

class StepEffortSchedule : public EffortSchedule {
public:
  explicit StepEffortSchedule(std::vector<EffortBlock> segments_)
      : segments(std::move(segments_)) {

    double t = 0.0;
    for (const auto& s : segments) {
      ranges.push_back({t, t + s.duration, s.effort});
      t += s.duration;
    }
    total_duration = t;
  }

  double effort_at(double t) const override {
    if (ranges.empty())
      return 0.0;

    for (const auto& r : ranges) {
      if (t < r.t_end)
        return r.effort;
    }

    // after schedule ends, hold last effort
    return ranges.back().effort;
  }

  double get_total_duration() const { return total_duration; }

private:
  struct Range {
    double t_start;
    double t_end;
    double effort;
  };

  std::vector<EffortBlock> segments;
  std::vector<Range> ranges;
  double total_duration = 0.0;
};
