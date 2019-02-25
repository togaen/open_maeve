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
struct tau_tolerance {
  static constexpr auto EPS = 1e-4;
};  // struct

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

double tau(const double range, const double actor1_speed,
           const double actor2_speed, const double epsilon);

double tau_range_at_0(const double tau_0, const double actor1_speed,
                      const double actor2_speed);

double tau_range_at_t(const double range_0, const double t,
                      const double other_speed,
                      const double actor1_distance_delta);

double compute_other_speed_from_tau(const double tau_0, const double tau_t,
                                    const double t,
                                    const double actor1_distance_delta,
                                    const double actor1_speed_0,
                                    const double actor1_speed_t,
                                    const double epsilon);

double compute_range_from_other_speed_and_tau(
    const double t, const double tau_0, const double other_speed,
    const double actor1_speed_0, const double actor1_distance_delta);

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

}  // namespace maeve_automation_core
