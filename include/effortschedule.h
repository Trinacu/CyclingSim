#include <vector>

class EffortSchedule {
public:
  virtual ~EffortSchedule() = default;

  // t = simulation time in seconds
  virtual double effort_at(double t) const = 0;
};

struct EffortBlock {
  double t_start;
  double t_end;
  double effort; // relative (e.g. 0.7, 1.2, 1.5)
};

class StepEffortSchedule : public EffortSchedule {
public:
  StepEffortSchedule(std::vector<EffortBlock> blocks_)
      : blocks(std::move(blocks_)) {}

  double effort_at(double t) const override {
    for (const auto& b : blocks) {
      if (t >= b.t_start && t < b.t_end)
        return b.effort;
    }
    return blocks.empty() ? 0.0 : blocks.back().effort;
  }

private:
  std::vector<EffortBlock> blocks;
};
