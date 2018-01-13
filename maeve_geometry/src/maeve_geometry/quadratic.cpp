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
#include "maeve_automation_core/maeve_geometry/quadratic.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace maeve_automation_core {
namespace {
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

Quadratic::Quadratic() : coefficients_({NaN, NaN, NaN}) {}

Quadratic::Quadratic(const double a, const double b, const double c)
    : coefficients_({a, b, c}), dx_coefficients_({2.0 * a, b}) {}

Quadratic Quadratic::fromPointWithDerivatives(const Eigen::Vector2d& p,
                                              const double dx,
                                              const double ddx) {
  const auto a = ddx;
  const auto b = dx - 2.0 * a * p.x();
  const auto c = p.y() + p.x() * (a * p.x() - dx);
  return Quadratic(a, b, c);
}

boost::optional<std::tuple<Eigen::Vector2d, Eigen::Vector2d>>
Quadratic::tangentOfRayThroughPoint(const Quadratic& quadratic,
                                    const Eigen::Vector2d& p_r,
                                    const Eigen::Vector2d& p_q) {
  // Capture coefficients.
  double a, b, c;
  std::tie(a, b, c) = Quadratic::coefficients(quadratic);

  // Compute solving equation.
  const auto A = a;
  const auto B = 2.0 * a * p_r.x();
  const auto C = -c + b * p_r.x() + p_r.y();
  const auto q_r = Quadratic(A, B, C);

  // Solve.
  double r1, r2;
  std::tie(r1, r2) = Quadratic::roots(q_r);

  // Both or neither roots must be valid.
  if (std::isnan(r1)) {
    return boost::none;
  }

  // Done.
  Eigen::Vector2d p1(r1, quadratic(r1));
  Eigen::Vector2d p2(r2, quadratic(r2));
  return std::make_tuple(p1, p2);
}

double Quadratic::dx(const Quadratic& quadratic, const double x) {
  return quadratic.dx_coefficients_[0] * x + quadratic.dx_coefficients_[1];
}

double Quadratic::ddx(const Quadratic& quadratic) {
  return quadratic.coefficients_[0];
}

double Quadratic::a(const Quadratic& quadratic) {
  return quadratic.coefficients_[0];
}

double Quadratic::b(const Quadratic& quadratic) {
  return quadratic.coefficients_[1];
}

double Quadratic::c(const Quadratic& quadratic) {
  return quadratic.coefficients_[2];
}

double Quadratic::operator()(const double x) const {
  return x * (coefficients_[0] * x + coefficients_[1]) + coefficients_[2];
}

std::tuple<double, double, double> Quadratic::coefficients(
    const Quadratic& quadratic) {
  return std::make_tuple(Quadratic::a(quadratic), Quadratic::b(quadratic),
                         Quadratic::c(quadratic));
}

std::tuple<double, double> Quadratic::roots(const Quadratic& quadratic) {
  // Capture coefficients
  double a, b, c;
  std::tie(a, b, c) = Quadratic::coefficients(quadratic);

  // Not quadratic.
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

std::ostream& operator<<(std::ostream& os, const Quadratic& quadratic) {
  return os << "{a: " << Quadratic::a(quadratic)
            << ", b:" << Quadratic::b(quadratic)
            << ", c:" << Quadratic::c(quadratic) << "}";
}
}  // namespace maeve_automation_core
