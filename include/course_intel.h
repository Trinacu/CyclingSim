// course_intel.h — C1 course knowledge ("the course book").
//
// A one-time digest of the course profile, built at sim start and shared
// const by every rider: perfect whole-course knowledge for now (a per-team
// noisy digest is the future hook for imperfect knowledge).  Kept separate
// from Course on purpose — Course stays the physics ground truth, CourseIntel
// answers the questions the decision layer asks.  Checkpoints are *not* its
// job (they are Course data, consumed by RaceClock in C0); CourseIntel only
// digests the profile.
//
// Climb index: consecutive uphill segments (slope >= min_gradient) merge into
// one climb; a non-uphill stretch shorter than max_dip_len inside doesn't
// split it (short dips and false-flat shoulders read as one climb, the way a
// rider thinks of it).  Merged runs shorter than min_climb_len or diluted
// below min_gradient overall are dropped.

#ifndef COURSE_INTEL_H
#define COURSE_INTEL_H

#include "course.h"
#include <optional>
#include <vector>

struct Climb {
  double start = 0.0;        // m
  double length = 0.0;       // m, start -> crest
  double avg_gradient = 0.0; // rise/run over [start, crest]
  double crest_pos = 0.0;    // start + length
};

struct CourseIntelParams {
  double min_gradient = 0.02; // below this a segment doesn't read as uphill
  double max_dip_len = 200.0; // non-uphill stretch this short doesn't split
  double min_climb_len = 200.0;
};

class CourseIntel {
public:
  explicit CourseIntel(const Course& course, CourseIntelParams params = {});

  double total_length() const { return total_length_; }
  double distance_to_finish(double pos) const { return total_length_ - pos; }

  // Altitude difference over distance — O(1) via Course::get_altitude.
  double avg_gradient(double from, double to) const;

  // The first climb whose crest is still ahead of pos: a rider midway up a
  // climb gets that climb.  nullopt past the last crest.
  std::optional<Climb> next_climb(double pos) const;
  std::optional<double> distance_to_crest(double pos) const;

  const std::vector<Climb>& climbs() const { return climbs_; }

private:
  const Course* course_;
  double total_length_;
  std::vector<Climb> climbs_; // sorted by start
};

#endif
