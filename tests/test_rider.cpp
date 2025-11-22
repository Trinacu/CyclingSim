#include "rider.h"
#include <cmath>
#include <iostream>

#include <cmath>
#include <iostream>

// Numerically estimate the derivative using central differences
double numerical_derivative(const std::function<double(double)>& func, double x,
                            double h = 1e-5) {
  return (func(x + h) - func(x - h)) / (2 * h);
}

// Check if the candidate matches the numerical derivative
bool verify_derivative(const std::function<double(double)>& f,
                       const std::function<double(double)>& df_candidate,
                       double x, double tol = 1e-6) {
  double df_numerical = numerical_derivative(f, x);
  double df_candidate_val = df_candidate(x);
  std::cout << df_numerical << std::endl;
  std::cout << df_candidate_val << std::endl;
  return std::abs(df_numerical - df_candidate_val) < tol;
}

int main() {
  const Course course = Course::create_flat();
  Team team("test team");
  Rider* rider = Rider::create_generic(team);
  rider->set_course(&course);
  double dt = 1;
  std::cout << *rider << std::endl;
  rider->update(dt);
  std::cout << *rider << std::endl;
  rider->update(dt);
  std::cout << *rider << std::endl;
  rider->update(dt);

  for (int i = 0; i < 100; i++) {
    rider->update(dt);
  }
  std::cout << *rider << std::endl;

  double test_point = 1.0;
  auto f_lambda = [&rider](double x) { return rider->pow_speed(x); };
  auto df_lambda = [&rider](double x) { return rider->pow_speed_prime(x); };
  auto d2f_lambda = [&rider](double x) {
    return rider->pow_speed_double_prime(x);
  };

  bool is_correct = verify_derivative(f_lambda, df_lambda, test_point);
  if (is_correct) {
    std::cout << "The candidate is likely the correct derivative!" << std::endl;
  } else {
    std::cout << "The candidate is NOT the correct derivative." << std::endl;
  }
  is_correct = verify_derivative(df_lambda, d2f_lambda, test_point);
  if (is_correct) {
    std::cout << "The candidate is likely the correct derivative!" << std::endl;
  } else {
    std::cout << "The candidate is NOT the correct derivative." << std::endl;
  }
}
