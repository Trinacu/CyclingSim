#include "course_intel.h"
#include <algorithm>

CourseIntel::CourseIntel(const Course& course, CourseIntelParams params)
    : course_(&course), total_length_(course.get_total_length()) {
  // Uphill runs, merged across short non-uphill dips.  Segments are
  // contiguous and sorted, so one forward pass suffices: extend the open
  // climb while the gap back to its last uphill stays within max_dip_len.
  double run_start = -1.0; // start of the open climb, -1 = none
  double run_end = 0.0;    // end of its last *uphill* segment

  auto close_run = [&]() {
    if (run_start < 0.0)
      return;
    const double length = run_end - run_start;
    const double avg =
        (course_->get_altitude(run_end) - course_->get_altitude(run_start)) /
        length;
    if (length >= params.min_climb_len && avg >= params.min_gradient)
      climbs_.push_back({run_start, length, avg, run_end});
    run_start = -1.0;
  };

  for (const Segment& s : course.get_segments()) {
    if (s.slope >= params.min_gradient) {
      if (run_start >= 0.0 && s.start_x - run_end > params.max_dip_len)
        close_run(); // dip too long: the previous climb ends at its crest
      if (run_start < 0.0)
        run_start = s.start_x;
      run_end = s.start_x + s.length;
    }
  }
  close_run();
}

double CourseIntel::avg_gradient(double from, double to) const {
  if (to <= from)
    return 0.0;
  return (course_->get_altitude(to) - course_->get_altitude(from)) /
         (to - from);
}

std::optional<Climb> CourseIntel::next_climb(double pos) const {
  auto it = std::upper_bound(
      climbs_.begin(), climbs_.end(), pos,
      [](double p, const Climb& c) { return p < c.crest_pos; });
  if (it == climbs_.end())
    return std::nullopt;
  return *it;
}

std::optional<double> CourseIntel::distance_to_crest(double pos) const {
  const auto c = next_climb(pos);
  if (!c)
    return std::nullopt;
  return c->crest_pos - pos;
}
