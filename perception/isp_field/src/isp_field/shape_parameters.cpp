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

#include <limits>

#include "maeve_automation_core/maeve_macros/checks.h"

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

ShapeParameters::ShapeParameters() : ShapeParameters(NaN, NaN, NaN, NaN, NaN) {}

ShapeParameters::ShapeParameters(const double t, const double r_min,
                                 const double r_max, const double a,
                                 const double b)
    : translation(t), range_min(r_min), range_max(r_max), alpha(a), beta(b) {}

double ShapeParameters::rangeMidPoint() const {
  return (range_min + range_max) / 2.0;
}

bool ShapeParameters::valid() const {
  // Perform checks.
  CHECK_STRICTLY_POSITIVE(alpha);
  CHECK_STRICTLY_POSITIVE(beta);
  CHECK_FINITE(alpha);
  CHECK_FINITE(beta);
  CHECK_LE(range_min, range_max);
  CHECK_FINITE(translation);

  // All good.
  return true;
}

std::ostream& operator<<(std::ostream& o, const ShapeParameters& sp) {
  return o << "[t: " << sp.translation << ", r_min: " << sp.range_min
           << ", r_mid: " << sp.rangeMidPoint() << ", r_max: " << sp.range_max
           << ", alpha: " << sp.alpha << ", beta: " << sp.beta << "]";
}

}  // namespace maeve_automation_core
