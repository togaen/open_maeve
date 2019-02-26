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
#include <sstream>
#include <stdexcept>

#include "maeve_automation_core/maeve_geometry/comparisons.h"
#include "maeve_automation_core/maeve_geometry/powers.h"

namespace maeve_automation_core {
namespace {
constexpr auto NaN = std::numeric_limits<double>::quiet_NaN();
constexpr auto INF = std::numeric_limits<double>::infinity();
}  // namespace

//------------------------------------------------------------------------------

double tau(const double range, const double relative_speed,
           const double epsilon) {
  if (approxZero(relative_speed, epsilon)) {
    return INF;
  }

  return (range / relative_speed);
}

//------------------------------------------------------------------------------

double compute_relative_dynamics_for_tau(const double actor1_component,
                                         const double actor2_component) {
  return (actor1_component - actor2_component);
}

//------------------------------------------------------------------------------

double tau(const double range, const double actor1_speed,
           const double actor2_speed, const double epsilon) {
  const double relative_speed =
      compute_relative_dynamics_for_tau(actor1_speed, actor2_speed);
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
      compute_relative_dynamics_for_tau(actor1_speed, actor2_speed);
  return tau_range(tau_0, relative_speed);
}

//------------------------------------------------------------------------------

double tau_range_at_t(const double range_0, const double actor1_distance_delta,
                      const double actor2_distance_delta) {
  return (range_0 + actor2_distance_delta - actor1_distance_delta);
}

//------------------------------------------------------------------------------

double tau_range_at_t(const double range_0, const double t,
                      const double actor2_speed,
                      const double actor1_distance_delta) {
  const double actor2_distance_delta = (t * actor2_speed);
  return tau_range_at_t(range_0, actor1_distance_delta, actor2_distance_delta);
}

//------------------------------------------------------------------------------

double compute_actor2_speed_from_tau(const double tau_0, const double tau_t,
                                     const double t,
                                     const double actor1_distance_delta,
                                     const double actor1_speed_0,
                                     const double actor1_speed_t,
                                     const double epsilon) {
  // Actor speed information is destroyed by the singularity at tau = 0
  if (approxZero(tau_0, epsilon) || approxZero(tau_t, epsilon)) {
    return NaN;
  }

  // Relative speed is zero, so actor2 speed is same as actor1 speed
  if (tau_0 == INF) {
    return actor1_speed_0;
  } else if (tau_t == INF) {
    return actor1_speed_t;
  }

  const auto denominator = (tau_t - tau_0 + t);

  // If relative speed does not change, there is not enough information to back
  // out actor2 speed
  if (approxZero(denominator, epsilon)) {
    // Verify assumptions
    const auto sum = (tau_t + t);
    if (approxZero(sum, epsilon)) {
      throw std::runtime_error("Actor collision detected after guard.");
    }

    return NaN;
  }

  const auto numerator =
      (tau_t * actor1_speed_t - tau_0 * actor1_speed_0 + actor1_distance_delta);
  return (numerator / denominator);
}

//------------------------------------------------------------------------------

double tau_range_at_t(const double t, const double tau_0,
                      const double actor2_speed, const double actor1_speed_0,
                      const double actor1_distance_delta) {
  const auto range_0 = tau_range(tau_0, actor1_speed_0, actor2_speed);
  const auto actor2_distance_delta = (t * actor2_speed);
  return tau_range_at_t(range_0, actor1_distance_delta, actor2_distance_delta);
}

//------------------------------------------------------------------------------

double tauFromDiscreteScaleDt(const double s, const double s_dot,
                              const double t_delta, const double epsilon) {
  return (tau(s, s_dot, epsilon) - t_delta);
}

//------------------------------------------------------------------------------

double tau_actor2_speed(const double tau, const double actor1_speed,
                        const double range, const double epsilon) {
  if (approxZero(tau, epsilon)) {
    return NaN;
  }

  return (actor1_speed - range / tau);
}

//------------------------------------------------------------------------------

double tau_at_t(const double range_0, const double t,
                const double actor1_speed_0, const double actor2_speed_0,
                const double actor1_accel, const double actor2_accel,
                const double epsilon) {
  const double t_relative_accel =
      (t * compute_relative_dynamics_for_tau(actor1_accel, actor2_accel));
  const double relative_speed =
      compute_relative_dynamics_for_tau(actor1_speed_0, actor2_speed_0);

  const double numerator = (range_0 + 0.5 * t * t_relative_accel);
  const double denominator = (relative_speed + t_relative_accel);

  if (approxZero(denominator, epsilon)) {
    return INF;
  }

  return ((numerator / denominator) - t);
}

//------------------------------------------------------------------------------

double compute_tau_desired_accel(const double t, const double range_0,
                                 const double actor1_speed_0,
                                 const double actor2_speed_0,
                                 const double actor2_accel,
                                 const double tau_desired,
                                 const double epsilon) {
  const auto singular_point = (-0.5 * t);
  if (approxEq(tau_desired, singular_point, epsilon)) {
    throw std::runtime_error("TODO(me): figure out what to do in this case.");
  }

  const auto delta_tau = (tau_desired + t);
  const auto relative_speed =
      compute_relative_dynamics_for_tau(actor1_speed_0, actor2_speed_0);

  const auto numerator = (delta_tau * relative_speed - range_0);
  const auto denominator = (0.5 * square(t) - (t * delta_tau));
  return ((numerator / denominator) + actor2_accel);
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
