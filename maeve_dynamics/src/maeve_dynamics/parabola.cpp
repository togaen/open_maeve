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
#include "maeve_automation_core/maeve_dynamics/parabola.h"

#include <limits>

namespace maeve_automation_core {
namespace {
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

Parabola::Parabola() : coefficients_({NaN, NaN, NaN}) {}

Parabola::Parabola(const double a, const double b, const double c)
    : coefficients_({a, b, c}), dt_coefficients_({2.0 * a, b}) {}

double Parabola::dt(const Parabola& parabola, const double time) {
  return parabola.dt_coefficients_[0] * time + parabola.dt_coefficients_[1];
}

double Parabola::ddt(const Parabola& parabola) {
  return parabola.dt_coefficients_[0];
}

double Parabola::a(const Parabola& parabola) {
  return parabola.coefficients_[0];
}

double Parabola::b(const Parabola& parabola) {
  return parabola.coefficients_[1];
}

double Parabola::c(const Parabola& parabola) {
  return parabola.coefficients_[2];
}

double Parabola::operator()(const double x) const {
  return x * (coefficients_[0] * x + coefficients_[1]) + coefficients_[2];
}

std::ostream& operator<<(std::ostream& os, const Parabola& parabola) {
  return os << "{a: " << Parabola::a(parabola)
            << ", b:" << Parabola::b(parabola)
            << ", c:" << Parabola::c(parabola) << "}";
}
}  // namespace maeve_automation_core
