/*
 * Copyright 2017 Maeve Automation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to
 * deal in the Software without restriction, including without limitation the
 * rights to use, copy, modify, merge, publish, distribute, sublicense, and/or
 * sell copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS
 * IN THE SOFTWARE.
 */
#include "maeve_automation_core/maeve_geometry/polynomial.h"

#include <algorithm>
#include <cmath>
#include <limits>
#include <sstream>
#include <stdexcept>

#include "boost/io/ios_state.hpp"

#include "maeve_automation_core/maeve_geometry/comparisons.h"
#include "maeve_automation_core/maeve_geometry/powers.h"

namespace maeve_automation_core {
namespace {
const auto NaN = std::numeric_limits<double>::quiet_NaN();
const auto Inf = std::numeric_limits<double>::infinity();
}  // namespace

//------------------------------------------------------------------------------

Polynomial::Polynomial(const double a, const double b, const double c)
    : coefficients_({a, b, c}), dx_coefficients_({2.0 * a, b}) {
  if (!Polynomial::valid(*this)) {
    std::stringstream ss;
    ss << "Attempted to consruct an invalid polynomial: " << *this;
    throw std::runtime_error(ss.str());
  }
}

//------------------------------------------------------------------------------

Polynomial::Polynomial(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
  // Allocate coefficients.
  auto a = 0.0;
  auto b = NaN;
  auto c = NaN;

  // Compute coefficient values.
  const Eigen::Vector2d d = (p2 - p1);
  if (d.x() == 0.0) {
    b = std::copysign(Inf, (p2.y() - p1.y()));
  } else {
    b = (d.y() / d.x());
    c = (p2.y() - b * p2.x());
  }

  // Poor man's delegated constructor
  *this = Polynomial(a, b, c);
}

//------------------------------------------------------------------------------

boost::optional<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>
Polynomial::findConstrainedCriticalPoints(const Eigen::Vector2d& p,
                                          const double y_critical,
                                          const double ddx) {
  // Attempt to get roots.
  const auto A = ddx;
  const auto B = (-2.0 * ddx * p.x());
  const auto C = (ddx * square(p.x()) + y_critical - p.y());
  double x1_critical, x2_critical;
  if (const auto roots = Polynomial::roots(A, B, C)) {
    std::tie(x1_critical, x2_critical) = *roots;
  } else {
    return boost::none;
  }

  // Construct critical points.
  const Eigen::Vector2d p1(x1_critical, y_critical);
  const Eigen::Vector2d p2(x2_critical, y_critical);

  // Done.
  return std::make_tuple(p1, p2);
}

//------------------------------------------------------------------------------

bool Polynomial::valid(const Polynomial& polynomial) {
  double a, b, c;
  std::tie(a, b, c) = Polynomial::coefficients(polynomial);
  const auto all_finite =
      (std::isfinite(a) && std::isfinite(b) && std::isfinite(c));

  const auto a_is_zero = (a == 0.0);
  const auto b_is_zero = (b == 0.0);
  const auto c_is_zero = (c == 0.0);
  const auto well_formed = !(a_is_zero && b_is_zero && c_is_zero);

  return (all_finite && well_formed);
}

//------------------------------------------------------------------------------

boost::optional<Eigen::Vector2d> Polynomial::uniqueCriticalPoint(
    const Polynomial& polynomial) {
  // Find unique critical point of a quadratic equation.
  if (std::abs(Polynomial::a(polynomial)) > 0.0) {
    const auto x_critical =
        (-polynomial.dx_coefficients_[1] / polynomial.dx_coefficients_[0]);
    const auto y_critical = polynomial(x_critical);
    return Eigen::Vector2d(x_critical, y_critical);
  }

  // This function is not defined for other polynomials.
  return boost::none;
}

//------------------------------------------------------------------------------

Polynomial Polynomial::fromPointWithDerivatives(const Eigen::Vector2d& p,
                                                const double dx,
                                                const double ddx) {
  const auto a = ddx;
  const auto b = dx - 2.0 * a * p.x();
  const auto c = p.y() + p.x() * (a * p.x() - dx);
  return Polynomial(a, b, c);
}

//------------------------------------------------------------------------------

Eigen::Vector2d Polynomial::quadraticPointAtDerivative(const Polynomial& P,
                                                       const double dx) {
  const auto x = ((dx - Polynomial::b(P)) / (2.0 * Polynomial::a(P)));
  return Eigen::Vector2d(x, P(x));
}

//------------------------------------------------------------------------------

boost::optional<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>
Polynomial::tangentRaysThroughPoint(const Polynomial& polynomial,
                                    const Eigen::Vector2d& p_r,
                                    const double epsilon) {
  // Capture coefficients.
  double a, b, c;
  std::tie(a, b, c) = Polynomial::coefficients(polynomial);

  // Compute solving equation.
  const auto A = a;
  const auto B = -2.0 * a * p_r.x();
  const auto C = -b * p_r.x() + p_r.y() - c;

  // Attempt to find the roots.
  double r1, r2;
  if (const auto roots = Polynomial::roots(A, B, C)) {
    std::tie(r1, r2) = *roots;
  } else {
    return boost::none;
  }

  // Compute tangent points.
  Eigen::Vector2d p1(r1, polynomial(r1));
  Eigen::Vector2d p2(r2, polynomial(r2));

  // Done.
  return std::make_tuple(p1, p2);
}

//------------------------------------------------------------------------------

double Polynomial::dx(const Polynomial& polynomial, const double x) {
  return polynomial.dx_coefficients_[0] * x + polynomial.dx_coefficients_[1];
}

//------------------------------------------------------------------------------

double Polynomial::ddx(const Polynomial& polynomial) {
  return polynomial.coefficients_[0];
}

//------------------------------------------------------------------------------

double Polynomial::a(const Polynomial& polynomial) {
  return polynomial.coefficients_[0];
}

//------------------------------------------------------------------------------

double Polynomial::b(const Polynomial& polynomial) {
  return polynomial.coefficients_[1];
}

//------------------------------------------------------------------------------

double Polynomial::c(const Polynomial& polynomial) {
  return polynomial.coefficients_[2];
}

//------------------------------------------------------------------------------

double Polynomial::operator()(const double x) const {
  return x * (coefficients_[0] * x + coefficients_[1]) + coefficients_[2];
}

//------------------------------------------------------------------------------

bool Polynomial::is_constant(const Polynomial& p) {
  double a, b, c;
  std::tie(a, b, c) = Polynomial::coefficients(p);

  const auto a_is_zero = (a == 0.0);
  const auto b_is_zero = (b == 0.0);
  const auto c_is_not_zero = (c != 0.0);
  return (a_is_zero && b_is_zero && c_is_not_zero);
}

//------------------------------------------------------------------------------

std::tuple<double, double, double> Polynomial::coefficients(
    const Polynomial& polynomial) {
  return std::make_tuple(Polynomial::a(polynomial), Polynomial::b(polynomial),
                         Polynomial::c(polynomial));
}

//------------------------------------------------------------------------------

boost::optional<std::tuple<double, double>> Polynomial::roots(
    const Polynomial& polynomial) {
  double a, b, c;
  std::tie(a, b, c) = Polynomial::coefficients(polynomial);
  return Polynomial::roots(a, b, c);
}

//------------------------------------------------------------------------------

boost::optional<std::tuple<double, double>> Polynomial::roots(
    const double a, const double b, const double c, const double tolerance) {
  // Not quadratic: either indeterminate or linear.
  if (a == 0.0) {
    // Indeterminate.
    if (b == 0.0) {
      return boost::none;
    }

    // Linear.
    const auto x = -c / b;
    return std::make_tuple(x, x);
  }

  // Compute the discriminant.
  const auto discriminant = clampToZero((b * b - 4.0 * a * c), tolerance);

  // Roots are complex, the range of this function is real, so no solution.
  if (discriminant < 0.0) {
    return boost::none;
  }

  // Choose the sign that avoids catastrophic cancellation.
  const auto discriminant_root = std::copysign(std::sqrt(discriminant), b);

  // Compute the first root.
  const auto r1 = ((-b - discriminant_root) / (2.0 * a));

  // Compute the second root with Vieta's formula.
  // 'r1' will always be nonzero due to sign choice above.
  // Check for zero discriminant to ensure unique roots are exactly identical.
  const auto r2 = ((discriminant == 0.0) ? r1 : (c / (a * r1)));

  // Order the roots lowest and highest.
  std::tuple<double, double> roots = std::minmax(r1, r2);

  // Done.
  return roots;
}

//------------------------------------------------------------------------------

std::ostream& operator<<(std::ostream& os, const Polynomial& polynomial) {
  static const auto PRECISION = 5;

  // Restore stream state on exit.
  boost::io::ios_all_saver guard(os);

  // Temporarily set desired flags and precision.
  os.setf(std::ios::fixed, std::ios::floatfield);
  os.precision(PRECISION);

  return os << "{\"a\": " << Polynomial::a(polynomial)
            << ", \"b\": " << Polynomial::b(polynomial)
            << ", \"c\": " << Polynomial::c(polynomial) << "}";
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
