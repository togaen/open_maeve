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
#include "maeve_automation_core/isp_field/shape_parameters.h"

#include <cmath>
#include <limits>

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

ShapeParameters::ShapeParameters()
    : translation(NaN), range_min(NaN), range_max(NaN), alpha(NaN), beta(NaN) {}

ShapeParameters::ShapeParameters(const double t, const double r_min,
                                 const double r_max, const double a,
                                 const double b)
    : translation(t), range_min(r_min), range_max(r_max), alpha(a), beta(b) {}

double ShapeParameters::rangeMidPoint() const {
  return (range_min + range_max) / 2.0;
}

bool ShapeParameters::valid() const {
  const auto alpha_valid = (alpha >= 0.0) && (alpha <= 1.0);
  const auto beta_valid = (beta >= 0.0) && (beta <= 1.0);
  const auto range_valid = (range_min <= range_max);
  const auto translation_valid = !std::isnan(translation);
  return alpha_valid && beta_valid && range_valid && translation_valid;
}

std::ostream& operator<<(std::ostream& o, const ShapeParameters& sp) {
  return o << "{t: " << sp.translation << ", r_min: " << sp.range_min
           << ", r_mid: " << sp.rangeMidPoint() << ", r_max: " << sp.range_max
           << ", alpha: " << sp.alpha << ", beta: " << sp.beta << "}";
}

}  // namespace maeve_automation_core
