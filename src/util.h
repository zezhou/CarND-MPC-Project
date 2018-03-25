#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/QR"
#include <vector>
#include <math.h>
#include <iostream>

struct Point {
    double x;
    double y;
};

using Trajectory = std::vector<Point>;

// Evaluate a polynomial.
double polyeval(Eigen::VectorXd coeffs, double x) {
  double result = 0.0;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * pow(x, i);
  }
  return result;
}

// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
double deg2rad(double x) { return x * pi() / 180; }
double rad2deg(double x) { return x * 180 / pi(); }

// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals,
                        int order) {  // @NOTE
  assert(xvals.size() == yvals.size());
  assert(order >= 1 && order <= xvals.size() - 1);
  Eigen::MatrixXd A(xvals.size(), order + 1);

  for (int i = 0; i < xvals.size(); i++) {
    A(i, 0) = 1.0;
  }

  for (int j = 0; j < xvals.size(); j++) {
    for (int i = 0; i < order; i++) {
      A(j, i + 1) = A(j, i) * xvals(j);
    }
  }

  auto Q = A.householderQr();
  auto result = Q.solve(yvals);
  return result;
}

Point world_coordinate2vehicle_coordinate(const Point& world_coordinate_point,
                        const Point& car_point, const double theta) {
    Point point;
    double dx = world_coordinate_point.x - car_point.x;
    double dy = world_coordinate_point.y - car_point.y;
    point.x = dx * cos(-theta) - dy * sin(-theta);
    point.y = dx * sin(-theta) + dy * cos(-theta);
    return point;
}

void log(std::vector<double> vars) {
    std::cout << "x = " << vars[0] << std::endl;
    std::cout << "y = " << vars[1] << std::endl;
    std::cout << "psi = " << vars[2] << std::endl;
    std::cout << "v = " << vars[3] << std::endl;
    std::cout << "cte = " << vars[4] << std::endl;
    std::cout << "epsi = " << vars[5] << std::endl;
    std::cout << "delta = " << vars[6]/deg2rad(25) << std::endl;
    std::cout << "a = " << vars[7] << std::endl;
    std::cout << std::endl;
}

template<typename T>
void print_vector(std::vector<T> v) {
  for (auto item : v) {
    std::cout << item << std::endl;
  }
}