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
                              const double t_delta);

// TODO: put little utility functions that codify ttc computation conventions

struct speed_and_relative_distance {
  const double speed;
  const double distance;
};

speed_and_relative_distance compute_speed_and_relative_distance(
    const double tau_0, const double tau_t, const double t,
    const double delta_p1, const double p1_dot_0, const double p1_dot_t);
}  // namespace maeve_automation_core
