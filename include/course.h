// course.h
#ifndef COURSE_H
#define COURSE_H

#include "pch.hpp"
#include <iostream>
#include <string>
#include <vector>

// Wind over a course.  `heading` is the direction the wind blows *from*, in
// the same angular convention as Segment::heading: wind.heading == rider
// heading means a full headwind (positive env.headwind via the cos projection
// in Rider::update); the matching sin projection gives the signed crosswind
// that the drafting wake axis and the rotation swing side consume.
struct Wind {
  double heading;
  double speed;
};

struct Segment {
  double start_x;
  double length;
  double slope;
  double crr;
  double heading;
  double road_width;
};

struct CoursePoint {
  double x;
  double y;
  double slope;

  Vector2d vec2() const { return Vector2d(x, y); }
};

// Named timing point (C0) — course data, not derived intel: TT timechecks,
// KOM lines, the finish.  RaceClock captures exact per-rider crossing times
// for each (they never go through the interpolated gap grid).
struct Checkpoint {
  double pos;
  std::string label;
};

std::ostream& operator<<(std::ostream& os, const Segment& seg);

class ICourseView {
public:
  virtual double get_slope(double pos) const = 0;
  virtual double get_altitude(double pos) const = 0;
  virtual double get_crr(double pos) const = 0;
  virtual double get_road_width(double pos) const = 0;
  virtual double get_heading(double pos) const = 0;
  virtual Wind get_wind(double pos) const = 0;
  // virtual bool isCheckpoint(double pos) const = 0;
  virtual ~ICourseView() = default;

  double virtual get_total_length() const { return total_length_; }

protected:
  double total_length_;
};

class Course : public ICourseView {
private:
  std::vector<Segment> segments;
  Wind wind_{0.0, 0.0};
  // Sorted by pos; the finish (at total_length) is implicit — every course
  // ends with it.
  std::vector<Checkpoint> checkpoints_;
  // std::vector<double> altitudes; // y at each segment start

public:
  std::vector<CoursePoint> points;
  // std::vector<Vector2d> visual_points;

  Course(const std::vector<std::array<double, 5>> segments,
         double starting_alt);
  static Course
  from_segments(const std::vector<std::array<double, 5>> segments);

  double get_altitude(double pos) const override;
  double get_slope(double pos) const override;
  double get_crr(double pos) const override;
  double get_road_width(double pos) const override;
  double get_heading(double pos) const override;
  Wind get_wind(double pos) const override;
  void set_wind(Wind w);

  // Inserts sorted by pos (ahead of the implicit finish).
  void add_checkpoint(double pos, std::string label);
  const std::vector<Checkpoint>& get_checkpoints() const {
    return checkpoints_;
  }

  // Raw profile access for CourseIntel's one-time digest (C1).
  const std::vector<Segment>& get_segments() const { return segments; }

  int find_segment(double pos) const;

  MatrixX2d get_points(double x_min, double x_max) const;

  static Course create_flat();
  static Course create_flat_short();
  static Course create_endulating();

  void print();
};
#endif
