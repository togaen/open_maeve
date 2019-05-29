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
#pragma once

#include <cmath>
#include <limits>
#include <stdexcept>

#include "maeve_automation_core/maeve_geometry/comparisons.h"
#include "maeve_automation_core/maeve_geometry/powers.h"

namespace maeve_automation_core {
/** @brief A tolerance to use when checking proximity to singularities. */
template <typename T>
struct tau_constants {
  static constexpr T EPS = static_cast<T>(1e-4);
  static constexpr T NaN = std::numeric_limits<T>::quiet_NaN();
  static constexpr T INF = std::numeric_limits<T>::infinity();
};  // struct

template <typename T>
constexpr T tau_constants<T>::EPS;
template <typename T>
constexpr T tau_constants<T>::NaN;
template <typename T>
constexpr T tau_constants<T>::INF;

/**
 * This function encodes the convention used by this library that relative
 * dynamics are computed as:
 *
 *     (actor1_component - actor2_component)
 *
 * For example, in a scene containing two actors where one is approaching from
 * behind, the convention would be that the rear, approaching actor is 'actor1'
 * and the other, leading actor is 'actor2'.
 *
 */
template <typename T>
T compute_relative_dynamics_for_tau(const T& actor1_component,
                                    const T& actor2_component) {
  return (actor1_component - actor2_component);
}

/**
 * @brief Given actor1 component, recover actor2 component from
 * compute_relative_dynamics_for_tau.
 */
template <typename T>
T get_actor2_speed_from_relative_dynamics(const T& relative_speed,
                                          const T& actor1_speed) {
  return (actor1_speed - relative_speed);
}

/**
 * @brief Compute time to contact (tau) between two objects separated by
 * straight line distance 'range' and approaching each other at
 * 'relative_speed'.
 *
 * @note Sign convention for tau is: positive as objects approach each other and
 * negative as they separate.
 *
 * @note Value convention for tau is: infinite when they are stationary w.r.t.
 * each other, zero when they are in contact, and otherwise finite.
 *
 * @note The objects are considered stationary w.r.t. each other when
 * the magnitude of 'relative_speed' is less than epsilon.
 *
 * @pre The sign of 'relative_speed' should be: negative if the objects are
 * moving apart and positive otherwise.
 */
template <typename T>
T tau(const T& range, const T& relative_speed, const T& epsilon) {
  if (approxZero(relative_speed, epsilon)) {
    return tau_constants<T>::INF;
  }

  return (range / relative_speed);
}

/**
 * @brief This is an overload for the above function that computes relative
 * speed internally given absolute speeds.
 */
template <typename T>
T tau(const T& range, const T& actor1_speed, const T& actor2_speed,
      const T& epsilon) {
  const auto relative_speed =
      compute_relative_dynamics_for_tau(actor1_speed, actor2_speed);
  return tau(range, relative_speed, epsilon);
}

/**
 * @brief Solve the tau function for relative speed.
 *
 * @note Relative speed is positive when actor1 is approaching actor2
 *
 * @note Relative speed is undefined (i.e., NaN) when tau is zero.
 */
template <typename T>
T tau_speed(const T& tau_0, const T& range, const T& eps) {
  return (approxZero(tau_0, eps) ? tau_constants<T>::NaN : (range / tau_0));
}

/**
 * @brief These solve the tau functions for range.
 * @{
 */
template <typename T>
T tau_range(const T& tau_0, const T& relative_speed) {
  return (tau_0 * relative_speed);
}
template <typename T>
T tau_range(const T& tau_0, const T& actor1_speed, const T& actor2_speed) {
  const auto relative_speed =
      compute_relative_dynamics_for_tau(actor1_speed, actor2_speed);
  return tau_range(tau_0, relative_speed);
}
/** @} */

/**
 * @brief Given two measurements of tau and two measurements of actor1 state
 * taken 't' seconds apart, compute a constant speed for actor2 over 't' that
 * explains both tau measurements.
 *
 * @note This function is extremely sensitive to the assumption that actor2
 * speed is constant. If that assumption is violated even slightly, the result
 * is almost certainly garbage.
 */
template <typename T>
T compute_actor2_speed_from_tau(const T& tau_0, const T& tau_t, const T& t,
                                const T& actor1_distance_delta,
                                const T& actor1_speed_0,
                                const T& actor1_speed_t, const T& epsilon) {
  // Actor speed information is destroyed by the singularity at tau = 0
  if (approxZero(tau_0, epsilon) || approxZero(tau_t, epsilon)) {
    return tau_constants<T>::NaN;
  }

  // Relative speed is zero, so actor2 speed is same as actor1 speed
  // TODO(me): guard against bad sign on infinities?
  if (std::isinf(tau_0)) {
    return actor1_speed_0;
  } else if (std::isinf(tau_t)) {
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

    return tau_constants<T>::NaN;
  }

  const auto numerator =
      (tau_t * actor1_speed_t - tau_0 * actor1_speed_0 + actor1_distance_delta);
  return (numerator / denominator);
}

/**
 * @brief Compute the range at some future time when actors 1 and 2 have moved
 * by the given deltas.
 */
template <typename T>
T tau_range_at_t(const T& range_0, const T& actor1_distance_delta,
                 const T& actor2_distance_delta) {
  return (range_0 + actor2_distance_delta - actor1_distance_delta);
}

/**
 * @brief Compute the range at time 't0 + t' given the following parameters:
 *
 * @param range_0                The range at t0 (i.e., the initial range)
 * @param t                      The time that has passed
 * @param actor2_speed           The speed of actor2
 * @param actor1_distance_delta  Change in distance of actor1 over time 't'
 *
 * @note 'actor2_speed' is assumed to be constant throughout time 't'.
 */
template <typename T>
T tau_range_at_t(const T& range_0, const T& t, const T& actor2_speed,
                 const T& actor1_distance_delta) {
  const double actor2_distance_delta = (t * actor2_speed);
  return tau_range_at_t(range_0, actor1_distance_delta, actor2_distance_delta);
}

/**
 * @brief Overload for the above function that computes range_0 using tau_0.
 */
template <typename T>
T tau_range_at_t(const T& t, const T& tau_0, const T& actor2_speed,
                 const T& actor1_speed_0, const T& actor1_distance_delta) {
  const auto range_0 = tau_range(tau_0, actor1_speed_0, actor2_speed);
  const auto actor2_distance_delta = (t * actor2_speed);
  return tau_range_at_t(range_0, actor1_distance_delta, actor2_distance_delta);
}

/**
 * @brief Given scale, scale time derivative, and timestemp, compute tau.
 *
 * When computing s_dot from finite differencing, the timestep needs to be
 * explicity considered when computing tau. That's why it is required as an
 * argument here.
 *
 * @param s The scale (maximum extent).
 * @param s_dot The derivative of scale with respect to time.
 * @param t_delta The timestamp used to compute s_dot
 *
 * @return Time to contact (tau).
 */
template <typename T>
T tauFromDiscreteScaleDt(const T& s, const T& s_dot, const T& t_delta,
                         const T& epsilon) {
  return (tau(s, s_dot, epsilon) - t_delta);
}

/** @brief Solve the tau function for actor2 speed */
template <typename T>
T tau_actor2_speed(const T& tau, const T& actor1_speed, const T& range,
                   const T& epsilon) {
  if (approxZero(tau, epsilon)) {
    return tau_constants<T>::NaN;
  }

  return (actor1_speed - range / tau);
}

/**
 * @brief For a given initial range and actor dynamics, compute tau at time t
 * under constant acceleration assumption.
 */
template <typename T>
T tau_at_t(const T& range_0, const T& t, const T& actor1_speed_0,
           const T& actor2_speed_0, const T& actor1_accel,
           const T& actor2_accel, const T& epsilon) {
  const auto t_relative_accel =
      (t * compute_relative_dynamics_for_tau(actor1_accel, actor2_accel));
  const auto relative_speed =
      compute_relative_dynamics_for_tau(actor1_speed_0, actor2_speed_0);

  const auto numerator = (range_0 + T(0.5) * t * t_relative_accel);
  const auto denominator = (relative_speed + t_relative_accel);

  if (approxZero(denominator, epsilon)) {
    return tau_constants<T>::INF;
  }

  return ((numerator / denominator) - t);
}

/**
 * @brief Compute the acceleration for actor1 that achieves 'tau_desired'
 * w.r.t. actor2 at time 't' given the problem information and assuming constant
 * acceleration motion for both actors.
 */
template <typename T>
T compute_actor1_accel_to_tau_desired(const T& t, const T& range_0,
                                      const T& actor1_speed_0,
                                      const T& actor2_speed_0,
                                      const T& actor2_accel,
                                      const T& tau_desired, const T& epsilon) {
  const auto singular_point = (T(-0.5) * t);
  if (approxEq(tau_desired, singular_point, epsilon)) {
    throw std::runtime_error("TODO(me): figure out what to do in this case.");
  }

  const auto delta_tau = (tau_desired + t);
  const auto relative_speed =
      compute_relative_dynamics_for_tau(actor1_speed_0, actor2_speed_0);

  const auto numerator = (delta_tau * relative_speed - range_0);
  const auto denominator = (T(0.5) * square(t) - (t * delta_tau));
  return ((numerator / denominator) + actor2_accel);
}

/** @brief Untested. */
template <typename T>
T partial_of_tau_wrt_actor1_accel(const T& t, const T& actor1_speed_0,
                                  const T& actor2_speed_0, const T& range_0,
                                  const T& actor1_accel, const T& actor2_accel,
                                  const T& epsilon) {
  const auto relative_accel =
      compute_relative_dynamics_for_tau(actor1_accel, actor2_accel);

  const auto relative_speed =
      compute_relative_dynamics_for_tau(actor1_speed_0, actor2_speed_0);

  const auto numerator = (t * (T(0.5) * t * relative_speed - range_0));
  const auto denominator = square(relative_speed + t * relative_accel);
  if (approxZero(denominator, epsilon)) {
    return tau_constants<T>::NaN;
  }

  return (numerator / denominator);
}

/** @brief Untested. */
template <typename T>
T partial_of_tau_wrt_actor2_accel(const T& t, const T& actor1_speed_0,
                                  const T& actor2_speed_0, const T& range_0,
                                  const T& actor1_accel, const T& actor2_accel,
                                  const T& epsilon) {
  return -partial_of_tau_wrt_actor1_accel(t, actor1_speed_0, actor2_speed_0,
                                          range_0, actor1_accel, actor2_accel,
                                          epsilon);
}

}  // namespace maeve_automation_core
