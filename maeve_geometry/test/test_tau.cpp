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
#include <gtest/gtest.h>

#include <Eigen/Core>

#include <cmath>

#include "maeve_automation_core/maeve_geometry/tau.h"

namespace maeve_automation_core {
namespace {
const auto epsilon = 0.00001;
double extent(const double Z, const Eigen::Vector2d& p1,
              const Eigen::Vector2d& p2) {
  const auto d = (p1 - p2).norm();
  return d / Z;
}

speed_and_relative_distance simple_motion(const double t, const double v_0,
                                          const double a) {
  const auto speed = (v_0 + a * t);
  const auto distance = (v_0 * t + 0.5 * a * t * t);
  return {speed, distance};
}

}  // namespace

//------------------------------------------------------------------------------

TEST(Tau, verifyScaling) {
  const auto Z = 17.0;
  const auto Z_dot = -1.2;
  const Eigen::Vector2d P1(2.3, 3.13);
  const Eigen::Vector2d P2(5.67, -1.32);

  auto t_delta = 10.37;
  auto tau = -(Z + Z_dot * t_delta) / Z_dot;
  const auto e1 = extent(Z, P1, P2);
  const auto e2 = extent(Z + Z_dot * t_delta, P1, P2);
  const auto e_dot = (e2 - e1) / t_delta;
  const auto tau_estimated = tauFromDiscreteScaleDt(e2, e_dot, t_delta);
  EXPECT_NEAR(tau_estimated, tau, 0.0001)
      << "e1: " << e1 << ", e2: " << e2 << ", e_dot: " << e_dot;
}

//------------------------------------------------------------------------------

TEST(Tau, speed_and_relative_distance1) {
  constexpr auto a = 1.0;
  constexpr auto t = 1.0;
  constexpr auto v1_0 = 2.7;
  constexpr auto v2 = 3.0;
  constexpr auto D_0 = 10.0;

  const auto state1_at_t = simple_motion(t, v1_0, a);
  const auto state2_at_t = simple_motion(t, v2, 0.0);

  std::cout << "s1: " << state1_at_t.speed << ", " << state1_at_t.distance
            << std::endl;
  std::cout << "s2: " << state2_at_t.speed << ", " << state2_at_t.distance
            << std::endl;

  const auto VR_0 = (v1_0 - v2);
  const auto VR_t = (state1_at_t.speed - v2);
  const auto D_t = (D_0 + t * v2 - state1_at_t.distance);
  const auto tau_0 = (D_0 / VR_0);
  const auto tau_t = (D_t / VR_t);

  std::cout << "tau_0: " << tau_0 << ", tau_t: " << tau_t << std::endl;
  std::cout << "D: " << D_0 << ", D_t: " << D_t << std::endl;

  const auto sd = compute_speed_and_relative_distance(
      tau_0, tau_t, t, state1_at_t.distance, v1_0, state1_at_t.speed);

  EXPECT_NEAR(sd.speed, v2, epsilon);
  EXPECT_NEAR(sd.distance, D_t, epsilon);
}
#if 0
//------------------------------------------------------------------------------

TEST(Tau, speed_and_relative_distance2) {
  constexpr auto a = 1.27;
  constexpr auto t = 0.1;
  constexpr auto v1_0 = 3.3;
  constexpr auto v2 = 1.5;
  constexpr auto D_0 = 13.0;

  const auto state1_at_t = simple_motion(t, v1_0, a);
  const auto state2_at_t = simple_motion(t, v2, 0.0);

  const auto VR_0 = (v1_0 - v2);
  const auto VR_t = (state1_at_t.speed - v2);
  const auto D_t = (D_0 - state2_at_t.distance + state1_at_t.distance);
  const auto tau_0 = (D_0 / VR_0);
  const auto tau_t = (D_t / VR_t);

  const auto sd = compute_speed_and_relative_distance(
      tau_0, tau_t, t, state1_at_t.distance, v1_0, state1_at_t.speed);

  EXPECT_NEAR(sd.speed, v2, epsilon);
  EXPECT_NEAR(sd.distance, D_t, epsilon);
}

//------------------------------------------------------------------------------

TEST(Tau, speed_and_relative_distance3) {
  constexpr auto a = 1.27;
  constexpr auto t = 0.1;
  constexpr auto v1_0 = 30.0;
  constexpr auto v2 = 25.0;
  constexpr auto a2 = 0.0;
  constexpr auto D_0 = -10.0;

  const auto state1_at_t = simple_motion(t, v1_0, a);
  const auto state2_at_t = simple_motion(t, v2, a2);

  const auto VR_0 = (v1_0 - v2);
  const auto VR_t = (state1_at_t.speed - v2);
  const auto D_t = (D_0 - (t * v2) + state1_at_t.distance);
  const auto tau_0 = (D_0 / VR_0);
  const auto tau_t = (D_t / VR_t);

  std::cout << "tau_0: " << tau_0 << ", tau_t: " << tau_t << std::endl;
  std::cout << "D: " << D_0 << ", D_t: " << D_t << std::endl;

  const auto sd = compute_speed_and_relative_distance(
      tau_0, tau_t, t, state1_at_t.distance, v1_0, state1_at_t.speed);

  EXPECT_NEAR(sd.speed, v2, epsilon);
  EXPECT_NEAR(sd.distance, D_t, epsilon);
}

//------------------------------------------------------------------------------
#endif
}  // namespace maeve_automation_core
