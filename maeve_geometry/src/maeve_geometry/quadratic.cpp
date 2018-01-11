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

std::tuple<double, double> Quadratic::roots(const Quadratic& quadratic) {
  // Capture coefficientsl
  const auto a = Quadratic::a(quadratic);
  const auto b = Quadratic::b(quadratic);
  const auto c = Quadratic::c(quadratic);

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
