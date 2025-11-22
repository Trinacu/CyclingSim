#include "course.h"
#include "pch.hpp"
#include <stdexcept>

// make CourseSegment easier to print
std::ostream& operator<<(std::ostream& os, const Segment& cs) {
  os << cs.length << " m at " << cs.slope * 100 << "%";
  return os;
}

Course::Course(const std::vector<std::array<double, 3>> segments_) {
  double x = 0.0, y = 0.0;
  visual_points.push_back(Vector2d(x, y));

  for (auto [len, slope, heading] : segments_) {
    segments.push_back({x, len, slope, x + len});
    altitudes.push_back(y);

    x += len;
    y += len * slope;
    visual_points.push_back(Vector2d(x, y));
  }
  total_length = x;
  printf("%f", total_length);
}

Course
Course::from_segments(const std::vector<std::array<double, 3>> segments) {
  return Course(segments);
}

Course Course::create_flat() {
  std::vector<std::array<double, 3>> v = {{300000, 0, 0}};
  return Course(v);
}

Course Course::create_endulating() {
  std::vector<std::array<double, 3>> v = {
      {100, 0, 0}, {200, 0.1, 0}, {200, 0.15, 0}, {500, 0, 0}};
  return Course(v);
}

double Course::get_altitude(double pos) const {
  int idx = find_segment(pos);
  return altitudes[idx] + segments[idx].altitude_at(pos);
}

double Course::get_slope(double pos) const {
  return segments[find_segment(pos)].slope;
}

Wind Course::get_wind(double pos) const { return Wind{1, 0}; }

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
      pts(row, 1) = altitudes[i];
      ++row;
    }
  }

  // x_max
  pts(row, 0) = x_max;
  pts(row, 1) = get_altitude(x_max);
  return pts;
}

int Course::find_segment(double x) const {
  int lo = 0, hi = segments.size() - 1;
  while (lo <= hi) {
    int mid = (lo + hi) / 2;
    if (x < segments[mid].start_x)
      hi = mid - 1;
    else if (x >= segments[mid].end_x)
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
    std::cout << segments[i].start_x << ", " << altitudes[i] << std::endl;
  }
  // for (const std::array<double, 3> &seg_range : segment_ranges) {
  //   std::cout << "x_start: " << seg_range[0] << ",\tx_end:" << seg_range[1]
  //             << ",\tslope: " << seg_range[2] * 100 << "%" << std::endl;
  // }
}
