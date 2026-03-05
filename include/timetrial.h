#ifndef TIMETRIAL_H
#define TIMETRIAL_H

#include <map>
#include <string>
#include <vector>

class Simulation;

// Forward declarations — keep sim.h out of the header
class Course;
struct RiderConfig;
struct RiderTimelineEntry;
using RiderId = int;

// Per-rider result: their timeline entries at each checkpoint + finish
struct RiderTimeTrialResult {
  RiderId rider_id;
  std::string name;
  std::vector<RiderTimelineEntry> timeline; // ordered: chk1, chk2, ..., finish
};

struct TimeTrialResult {
  std::vector<RiderTimeTrialResult> riders; // sorted by finish time, DNFs last
  std::vector<double> checkpoint_distances; // the distances that were measured
};

// Builds start offsets map: rider[i] starts at i * gap_seconds.
std::map<RiderId, double>
build_start_offsets(const std::vector<RiderConfig>& riders, double gap_seconds);

// Applies staggered effort schedules to an already-populated Simulation.
// Each rider gets effort=0 during their wait, then effort=1 for the race.
// The sim must already have the riders added before calling this.
void setup_tt_schedules(Simulation* sim, const std::vector<RiderConfig>& riders,
                        const std::map<RiderId, double>& start_offsets);

// Runs a time trial offline.
// Riders are staggered by `start_gap_seconds` in order of their position in
// the configs vector. Checkpoints are auto-generated at 25/50/75/100% of the
// course length (finish line is always the last checkpoint).
TimeTrialResult run_time_trial(const Course& course,
                               const std::vector<RiderConfig>& riders,
                               double start_gap_seconds = 60.0);

#endif
