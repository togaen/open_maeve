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

namespace maeve_automation_core {
/** @brief A tolerance to use when checking proximity to singularities. */
struct tau_tolerance {
  static constexpr auto EPS = 1e-4;
};  // struct

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
double compute_relative_dynamics_for_tau(const double actor1_component,
                                         const double actor2_component);

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
double tau(const double range, const double relative_speed,
           const double epsilon);

/**
 * @brief This is an overload for the above function that computes relative
 * speed internally given absolute speeds.
 */
double tau(const double range, const double actor1_speed,
           const double actor2_speed, const double epsilon);

/**
 * @brief These solve the tau functions for range.
 * @{
 */
double tau_range(const double tau_0, const double relative_speed);
double tau_range(const double tau_0, const double actor1_speed,
                 const double actor2_speed);
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
double compute_actor2_speed_from_tau(const double tau_0, const double tau_t,
                                     const double t,
                                     const double actor1_distance_delta,
                                     const double actor1_speed_0,
                                     const double actor1_speed_t,
                                     const double epsilon);

/**
 * @brief Compute the range at some future time when actors 1 and 2 have moved
 * by the given deltas.
 */
double tau_range_at_t(const double range_0, const double actor1_distance_delta,
                      const double actor2_distance_delta);

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
double tau_range_at_t(const double range_0, const double t,
                      const double actor2_speed,
                      const double actor1_distance_delta);

/**
 * @brief Overload for the above function that computes range_0 using tau_0.
 */
double tau_range_at_t(const double t, const double tau_0,
                      const double actor2_speed, const double actor1_speed_0,
                      const double actor1_distance_delta);

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
double tauFromDiscreteScaleDt(const double s, const double s_dot,
                              const double t_delta, const double epsilon);

/** @brief Solve the tau function for actor2 speed */
double tau_actor2_speed(const double tau, const double actor1_speed,
                        const double range, const double epsilon);

/**
 * @brief For a given initial range and actor dynamics, compute tau at time t
 * under constant acceleration assumption.
 */
double tau_at_t(const double range_0, const double t,
                const double actor1_speed_0, const double actor2_speed_0,
                const double actor1_accel, const double actor2_accel,
                const double epsilon);

/**
 * @brief Compute the acceleration for actor1 that achieves 'tau_desired'
 * w.r.t. actor2 at time 't' given the problem information and assuming constant
 * acceleration motion for both actors.
 */
double compute_actor1_accel_to_tau_desired(const double t, const double range_0,
                                           const double actor1_speed_0,
                                           const double actor2_speed_0,
                                           const double actor2_accel,
                                           const double tau_desired,
                                           const double epsilon);
}  // namespace maeve_automation_core
