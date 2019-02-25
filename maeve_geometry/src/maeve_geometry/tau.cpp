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
#include "maeve_automation_core/maeve_geometry/tau.h"
#include "maeve_automation_core/maeve_geometry/comparisons.h"

#include <iostream>
#include <limits>

namespace maeve_automation_core {
namespace {
static constexpr auto NaN = std::numeric_limits<double>::quiet_NaN();
static constexpr auto INF = std::numeric_limits<double>::infinity();
}  // namespace

//------------------------------------------------------------------------------

double tauFromDiscreteScaleDt(const double s, const double s_dot,
                              const double t_delta) {
  return (s_dot == 0.0) ? INF : ((s / s_dot) - t_delta);
}

//------------------------------------------------------------------------------

speed_and_relative_distance compute_speed_and_relative_distance(
    const double tau_0, const double tau_t, const double t,
    const double delta_p1, const double p1_dot_0, const double p1_dot_t) {
  // TODO: handle v1 = v0

  const auto compute_speed = [&]() {
    if (tau_0 == INF) {
      return p1_dot_0;
    } else if (tau_t == INF) {
      return p1_dot_t;
    } else {
      const auto denominator = (tau_t - tau_0 + t);
      if (approxZero(denominator,
                     1e-3)) {  // TODO: figure out what to do in this case
        return NaN;
      }
      const auto numerator = (tau_t * p1_dot_t - tau_0 * p1_dot_0 + delta_p1);
      return (numerator / denominator);
    }
  };

  const auto speed = compute_speed();
  const auto D0 = (tau_0 * (p1_dot_0 - speed));
  const auto distance = (D0 + t * speed - delta_p1);

  return {speed, distance};
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
