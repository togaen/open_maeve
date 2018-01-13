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

#include "maeve_automation_core/maeve_geometry/comparisons.h"

namespace maeve_automation_core {
namespace {
const auto NaN = std::numeric_limits<double>::quiet_NaN();
const auto Inf = std::numeric_limits<double>::infinity();
}  // namespace

Polynomial::Polynomial() : coefficients_({NaN, NaN, NaN}) {}

Polynomial::Polynomial(const double a, const double b, const double c)
    : coefficients_({a, b, c}), dx_coefficients_({2.0 * a, b}) {}

Polynomial::Polynomial(const Eigen::Vector2d& p1, const Eigen::Vector2d& p2) {
  // Allocate coefficients.
  double a = 0.0;
  double b = NaN;
  double c = NaN;

  // Compute coefficient values.
  const Eigen::Vector2d d = (p2 - p1);
  if (d.x() == 0.0) {
    b = std::copysign(Inf, (p2.y() - p1.y()));
  } else {
    b = (d.y() / d.x());
    c = (p2.y() - b * p2.x());
  }

  // Store.
  coefficients_ = {a, b, c};
  dx_coefficients_ = {0.0, b};
}

Polynomial Polynomial::fromPointWithDerivatives(const Eigen::Vector2d& p,
                                                const double dx,
                                                const double ddx) {
  const auto a = ddx;
  const auto b = dx - 2.0 * a * p.x();
  const auto c = p.y() + p.x() * (a * p.x() - dx);
  return Polynomial(a, b, c);
}

boost::optional<std::tuple<Polynomial, Polynomial>>
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
  const auto q_r = Polynomial(A, B, C);

  // Solve.
  double r1, r2;
  std::tie(r1, r2) = Polynomial::roots(q_r);

  // Both or neither roots must be valid.
  if (std::isnan(r1)) {
    return boost::none;
  }

  // Compute tangent points.
  Eigen::Vector2d p1(r1, polynomial(r1));
  Eigen::Vector2d p2(r2, polynomial(r2));

  // Construct rays.
  Polynomial ray1(p_r, p1);
  Polynomial ray2(p_r, p2);

#if 0  // These checks should not be needed.
  // Get coincident information.
  const auto ray1_coincident = approxEq(ray1(r1), p1.y(), epsilon);
  const auto ray2_coincident = approxEq(ray2(r2), p2.y(), epsilon);

  // Get tangency information.
  const auto p_dx1 = Polynomial::dx(polynomial, p1.x());
  const auto ray1_dx = Polynomial::dx(ray1, p1.x());
  const auto ray1_tangent = approxEq(ray1_dx, p_dx1, epsilon);

  const auto p_dx2 = Polynomial::dx(polynomial, p2.x());
  const auto ray2_dx = Polynomial::dx(ray2, p2.x());
  const auto ray2_tangent = approxEq(ray2_dx, p_dx2, epsilon);

  // Validate rays.
  const auto ray1_okay = (ray1_coincident && ray1_tangent);
  const auto ray2_okay = (ray2_coincident && ray2_tangent);

  // Check and return.
  if (!ray1_okay && !ray2_okay) {
    return boost::none;
  }

  if (!ray1_okay) {
    return std::make_tuple(ray2, ray2);
  }

  if (!ray2_okay) {
    return std::make_tuple(ray1, ray1);
  }
#endif
  return std::make_tuple(ray1, ray2);
}

double Polynomial::dx(const Polynomial& polynomial, const double x) {
  return polynomial.dx_coefficients_[0] * x + polynomial.dx_coefficients_[1];
}

double Polynomial::ddx(const Polynomial& polynomial) {
  return polynomial.coefficients_[0];
}

double Polynomial::a(const Polynomial& polynomial) {
  return polynomial.coefficients_[0];
}

double Polynomial::b(const Polynomial& polynomial) {
  return polynomial.coefficients_[1];
}

double Polynomial::c(const Polynomial& polynomial) {
  return polynomial.coefficients_[2];
}

double Polynomial::operator()(const double x) const {
  return x * (coefficients_[0] * x + coefficients_[1]) + coefficients_[2];
}

std::tuple<double, double, double> Polynomial::coefficients(
    const Polynomial& polynomial) {
  return std::make_tuple(Polynomial::a(polynomial), Polynomial::b(polynomial),
                         Polynomial::c(polynomial));
}

std::tuple<double, double> Polynomial::roots(const Polynomial& polynomial) {
  // Capture coefficients
  double a, b, c;
  std::tie(a, b, c) = Polynomial::coefficients(polynomial);

  // Not polynomial.
  if (a == 0.0) {
    // Indeterminate form.
    if (b == 0.0) {
      return std::make_tuple(NaN, NaN);
    }

    // Linear (first form).
    const auto x = -c / b;
    return std::make_tuple(x, x);
  }

  // Linear (second form).
  if (c == 0.0) {
    const auto x = -b / a;
    return std::make_tuple(x, x);
  }

  // Capture term under the square root.
  const auto discriminant = (b * b - 4.0 * a * c);

  // Complex.
  if (discriminant < 0.0) {
    return std::make_tuple(NaN, NaN);
  }

  // Compute one of the roots.
  const auto discriminant_root = std::copysign(std::sqrt(discriminant), b);
  const auto r1 = ((-b + discriminant_root) / (2.0 * a));

  // Compute the other.
  const auto r2 = (c / (a * r1));

  // Done.
  return std::minmax(r1, r2);
}

std::ostream& operator<<(std::ostream& os, const Polynomial& polynomial) {
  return os << "{a: " << Polynomial::a(polynomial)
            << ", b:" << Polynomial::b(polynomial)
            << ", c:" << Polynomial::c(polynomial) << "}";
}
}  // namespace maeve_automation_core
