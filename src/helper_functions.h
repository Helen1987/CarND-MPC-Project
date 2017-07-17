#ifndef SRC_HELPER_FUNCTIONS_H_
#define SRC_HELPER_FUNCTIONS_H_

#include <math.h>
#include "Eigen-3.3/Eigen/QR"

const size_t N = 6;
const double dt = 0.4;

// This value assumes the model presented in the classroom is used.
//
// It was obtained by measuring the radius formed by running the vehicle in the
// simulator around in a circle with a constant steering angle and velocity on a
// flat terrain.
//
// Lf was tuned until the the radius formed by the simulating the model
// presented in the classroom matched the previous radius.
//
// This is the length from front to CoG that has a similar radius.
const double Lf = 2.67;
const double MPH_to_MPS = 0.44704;

// Both the reference cross track and orientation errors are 0.
// The reference velocity is set to 40 mph.
const double ref_v = 25;

// For converting back and forth between radians and degrees.
inline constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }


// Fit a polynomial.
// Adapted from
// https://github.com/JuliaMath/Polynomials.jl/blob/master/src/Polynomials.jl#L676-L716
inline Eigen::VectorXd polyfit(Eigen::VectorXd xvals, Eigen::VectorXd yvals, int order) {
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

// Evaluate a polynomial.
template <typename T>
T polyeval(Eigen::VectorXd coeffs, T x) {
  T result = 0.0;
  T power = 1;
  for (int i = 0; i < coeffs.size(); i++) {
    result += coeffs[i] * power;
    power *= x;
  }
  return result;
}

// Evaluate a derivative of polynomial.
template <typename T>
T d_polyeval(Eigen::VectorXd coeffs, T x) {
  T result = 0.0;
  T power = 1;
  for (int i = 1; i < coeffs.size(); i++) {
    result += i * coeffs[i] * power;
    power *= x;
  }
  return result;
}

#endif  // SRC_HELPER_FUNCTIONS_H_