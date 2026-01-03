// course.h
#ifndef COURSE_H
#define COURSE_H

#include "pch.hpp"
#include <iostream>

struct Wind {
  double heading;
  double speed;
};

struct Segment {
  double start_x;
  double length;
  double slope;
  double end_x;
  double heading;
  double road_width;

  double altitude_at(double x) const { return slope * (x - start_x); }
};

std::ostream& operator<<(std::ostream& os, const Segment& seg);

class ICourseView {
public:
  virtual double get_slope(double pos) const = 0;
  virtual double get_altitude(double pos) const = 0;
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
  std::vector<double> altitudes; // y at each segment start

public:
  std::vector<Vector2d> visual_points;

  Course(const std::vector<std::array<double, 4>> segments);
  static Course
  from_segments(const std::vector<std::array<double, 4>> segments);

  double get_altitude(double pos) const override;
  double get_slope(double pos) const override;
  Wind get_wind(double pos) const override;

  int find_segment(double pos) const;

  MatrixX2d get_points(double x_min, double x_max) const;

  static Course create_flat();
  static Course create_flat_short();
  static Course create_endulating();

  void print();
};
#endif
