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

#include <cmath>
#include <limits>

#include "maeve_automation_core/maeve_geometry/comparisons.h"

namespace maeve_automation_core {
namespace {
constexpr auto NaN = std::numeric_limits<double>::quiet_NaN();
constexpr auto INF = std::numeric_limits<double>::infinity();
}  // namespace

//------------------------------------------------------------------------------

double tau(const double range, const double relative_speed,
           const double epsilon) {
  return (approxZero(relative_speed, epsilon) ? INF : (range / relative_speed));
}

//------------------------------------------------------------------------------

double compute_relative_speed_for_tau(const double actor1_speed,
                                      const double actor2_speed) {
  return (actor1_speed - actor2_speed);
}

//------------------------------------------------------------------------------

double tau(const double range, const double actor1_speed,
           const double actor2_speed, const double epsilon) {
  const double relative_speed =
      compute_relative_speed_for_tau(actor1_speed, actor2_speed);
  return tau(range, relative_speed, epsilon);
}

//------------------------------------------------------------------------------

double tau_range(const double tau_0, const double relative_speed) {
  return (tau_0 * relative_speed);
}

//------------------------------------------------------------------------------

double tau_range(const double tau_0, const double actor1_speed,
                 const double actor2_speed) {
  const auto relative_speed =
      compute_relative_speed_for_tau(actor1_speed, actor2_speed);
  return tau_range(tau_0, relative_speed);
}

//------------------------------------------------------------------------------

double tau_range_at_t(const double range_0, const double t,
                      const double actor2_speed,
                      const double actor1_distance_delta) {
  return (range_0 + (t * actor2_speed) - actor1_distance_delta);
}

//------------------------------------------------------------------------------

double compute_actor2_speed_from_tau(const double tau_0, const double tau_t,
                                     const double t,
                                     const double actor1_distance_delta,
                                     const double actor1_speed_0,
                                     const double actor1_speed_t,
                                     const double epsilon) {
  // TODO(me): handle v0 = v1

  if (tau_0 == INF) {
    return actor1_speed_0;
  } else if (tau_t == INF) {
    return actor1_speed_t;
  }

  const auto denominator = (tau_t - tau_0 + t);
  if (approxZero(denominator, epsilon)) {
    // TODO(me): figure out what to do in this case
    return NaN;
  }
  const auto numerator =
      (tau_t * actor1_speed_t - tau_0 * actor1_speed_0 + actor1_distance_delta);
  return (numerator / denominator);
}

//------------------------------------------------------------------------------

double compute_range_from_actor2_speed_and_tau(
    const double t, const double tau_0, const double actor2_speed,
    const double actor1_speed_0, const double actor1_distance_delta) {
  const auto range_0 = tau_range(tau_0, actor1_speed_0, actor2_speed);
  return tau_range_at_t(range_0, t, actor2_speed, actor1_distance_delta);
}

//------------------------------------------------------------------------------

double tauFromDiscreteScaleDt(const double s, const double s_dot,
                              const double t_delta, const double epsilon) {
  return (tau(s, s_dot, epsilon) - t_delta);
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
