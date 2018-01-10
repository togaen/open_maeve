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

#include <limits>

namespace maeve_automation_core {
namespace {
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

Quadratic::Quadratic() : coefficients_({NaN, NaN, NaN}) {}

Quadratic::Quadratic(const double a, const double b, const double c)
    : coefficients_({a, b, c}), dt_coefficients_({2.0 * a, b}) {}

double Quadratic::dt(const Quadratic& quadratic, const double time) {
  return quadratic.dt_coefficients_[0] * time + quadratic.dt_coefficients_[1];
}

double Quadratic::ddt(const Quadratic& quadratic) {
  return quadratic.dt_coefficients_[0];
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

std::tuple<double, double> Quadratic::inverse(const Quadratic& quadratic,
                                              const double y) {
  return std::make_tuple(NaN, NaN);
}

std::ostream& operator<<(std::ostream& os, const Quadratic& quadratic) {
  return os << "{a: " << Quadratic::a(quadratic)
            << ", b:" << Quadratic::b(quadratic)
            << ", c:" << Quadratic::c(quadratic) << "}";
}
}  // namespace maeve_automation_core
