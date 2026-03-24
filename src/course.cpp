#include "course.h"
#include "SDL3/SDL_log.h"
#include "pch.hpp"
#include <stdexcept>

// make CourseSegment easier to print
std::ostream& operator<<(std::ostream& os, const Segment& cs) {
  os << cs.length << " m at " << cs.slope * 100 << "%";
  return os;
}

Course::Course(const std::vector<std::array<double, 5>> segments_,
               double altitude_) {
  double x = 0.0, y = altitude_;

  for (auto [len, slope, crr, heading, road_width] : segments_) {
    segments.push_back({x, len, slope, crr, heading, road_width});
    points.push_back({x, y, slope});

    x += len;
    y += len * slope;
    // altitudes.push_back(y);
    // visual_points.push_back(Vector2d(x, y));
  }
  points.push_back({x, y, 0.0}); // final endpoint

  total_length_ = x;
  SDL_Log("Total course length: %.1f m", total_length_);
}

Course
Course::from_segments(const std::vector<std::array<double, 5>> segments) {
  return Course(segments, 0.0);
}

Course Course::create_flat() {
  std::vector<std::array<double, 5>> v = {{30000, 0, 0, 0, 8}};
  return Course(v, 0.0);
}

Course Course::create_flat_short() {
  std::vector<std::array<double, 5>> v = {{1000, 0, 0, 0, 8}};
  return Course(v, 0.0);
}

Course Course::create_endulating() {
  std::vector<std::array<double, 5>> v = {
      {100, 0, 0, 0, 8},    {200, 0.1, 0, 0, 8},  {200, 0, 0, 0, 8},
      {2000, 0.1, 0, 0, 8}, {2000, 0, 0, 0, 8},   {2000, 0.1, 0, 0, 8},
      {2000, 0, 0, 0, 8},   {5000, 0.05, 0, 0, 8}};
  return Course(v, 1000.0);
}

double Course::get_altitude(double pos) const {
  int idx = find_segment(pos);
  return points[idx].y + points[idx].slope * (pos - points[idx].x);
  // return altitudes[idx] + segments[idx].altitude_at(pos);
}

double Course::get_slope(double pos) const {
  return points[find_segment(pos)].slope;
}

double Course::get_crr(double pos) const {
  return segments[find_segment(pos)].crr;
}

double Course::get_road_width(double pos) const {
  return segments[find_segment(pos)].road_width;
}

// TODO - fix this to query whatever holds the values?
Wind Course::get_wind(double pos) const { return Wind{0, 1}; }

MatrixX2d Course::get_points(double x_min, double x_max) const {
  if (x_min > x_max)
    std::swap(x_min, x_max);
  if (x_min < 0)
    x_min = 0;
  int i0 = find_segment(x_min);
  int i1 = find_segment(x_max);

  // count points: endpoints + internal breaks
  int count = 2; // for x_min and x_max
  for (int i = i0 + 1; i <= i1; ++i) {
    double bx = segments[i].start_x;
    if (bx > x_min && bx < x_max)
      ++count;
  }

  MatrixX2d pts(count, 2);
  int row = 0;
  // x_min
  pts(row, 0) = x_min;
  pts(row, 1) = get_altitude(x_min);
  ++row;

  // internal breaks
  for (int i = i0 + 1; i <= i1; ++i) {
    double bx = segments[i].start_x;
    if (bx > x_min && bx < x_max) {
      pts(row, 0) = bx;
      // pts(row, 1) = altitudes[i];
      pts(row, 1) = points[i].y;
      ++row;
    }
  }

  // x_max
  pts(row, 0) = x_max;
  pts(row, 1) = get_altitude(x_max);
  return pts;
}

int Course::find_segment(double x) const {
  if (x > total_length_) {
    // SDL_Log("Trying to find segment for x > total_length (%.1f > %f.1f) in "
    //         "Course::find segment()",
    //         x, total_length_);
    return segments.size() - 1;
  }
  int lo = 0, hi = segments.size() - 1;
  while (lo <= hi) {
    int mid = (lo + hi) / 2;
    if (x < segments[mid].start_x)
      hi = mid - 1;
    else if (x >= (segments[mid].start_x + segments[mid].length))
      lo = mid + 1;
    else
      return mid;
  }
  throw std::out_of_range("x out of course bounds");
}

void Course::print() {
  std::cout << "Course:" << std::endl;
  for (const Segment& course_segment : segments) {
    std::cout << course_segment << std::endl;
  }
  for (int i = 0; i < segments.size(); i++) {
    // std::cout << segments[i].start_x << ", " << altitudes[i] << std::endl;
    std::cout << segments[i].start_x << ", " << points[i].y << std::endl;
  }
  // for (const std::array<double, 4> &seg_range : segment_ranges) {
  //   std::cout << "x_start: " << seg_range[0] << ",\tx_end:" << seg_range[1]
  //             << ",\tslope: " << seg_range[2] * 100 << "%" << std::endl;
  // }
}
