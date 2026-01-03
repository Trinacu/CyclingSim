#include "course.h"

int main() {
  std::vector<std::array<double, 4>> segments;
  segments = {{1000.0, 0.0, 0, 8},   {5000.0, 0.1, 0, 8},
              {1000.0, 0.15, 0, 8},  {1000.0, 0.0, 0, 8},
              {1000.0, -0.05, 0, 8}, {10000.0, 0.09, 0, 8}};
  Course course = Course::from_segments(segments);
  course.print();
  std::vector<double> xs = {0, 1000, 5000, 6500, 7000, 8000, 8100, 9000};
  for (double x : xs) {
    std::cout << x << ": " << course.get_altitude(x) << " @ "
              << course.get_slope(x) * 100 << "%" << std::endl;
  }

  MatrixX2d pts = course.get_points(1000, 6600);
  std::cout << pts << std::endl;
  std::cout << "----" << std::endl;
  pts = course.get_points(-1000, 6600);
  std::cout << pts << std::endl;
}
