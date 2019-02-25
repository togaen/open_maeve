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
#include <tuple>

#include "maeve_automation_core/maeve_geometry/tau.h"

namespace maeve_automation_core {
namespace {
constexpr auto NaN = std::numeric_limits<double>::quiet_NaN();
constexpr auto INF = std::numeric_limits<double>::infinity();

double extent(const double Z, const Eigen::Vector2d& p1,
              const Eigen::Vector2d& p2) {
  const auto d = (p1 - p2).norm();
  return d / Z;
}

std::tuple<double, double> simple_motion(const double t, const double v_0,
                                         const double a) {
  const auto speed = (v_0 + a * t);
  const auto distance = (v_0 * t + 0.5 * a * t * t);
  return std::make_tuple(distance, speed);
}

}  // namespace

//------------------------------------------------------------------------------

TEST(Tau, tau1) {
  {
    const auto range = 1.0;
    const auto relative_speed = 1.0;
    const auto expected_tau = 1.0;
    const auto computed_tau = tau(range, relative_speed, tau_tolerance::EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }

  {
    const auto range = 1.0;
    const auto relative_speed = -1.0;
    const auto expected_tau = -1.0;
    const auto computed_tau = tau(range, relative_speed, tau_tolerance::EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }

  {
    const auto range = 1.0;
    const auto relative_speed = 0.0;
    const auto expected_tau = INF;
    const auto computed_tau = tau(range, relative_speed, tau_tolerance::EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }

  {
    const auto range = 1.0;
    const auto relative_speed = (0.5 * tau_tolerance::EPS);
    const auto expected_tau = INF;
    const auto computed_tau = tau(range, relative_speed, tau_tolerance::EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }

  {
    const auto range = 1.0;
    const auto relative_speed = (2.0 * tau_tolerance::EPS);
    const auto computed_tau = tau(range, relative_speed, tau_tolerance::EPS);
    EXPECT_TRUE(std::isfinite(computed_tau));
  }
}

//------------------------------------------------------------------------------

TEST(Tau, tau2) {
  {
    const auto range = 1.0;
    const auto actor1_speed = 2.0;
    const auto actor2_speed = 1.0;
    const auto expected_tau = 1.0;
    const auto computed_tau =
        tau(range, actor1_speed, actor2_speed, tau_tolerance::EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }

  {
    const auto range = 1.0;
    const auto actor1_speed = 1.0;
    const auto actor2_speed = 2.0;
    const auto expected_tau = -1.0;
    const auto computed_tau =
        tau(range, actor1_speed, actor2_speed, tau_tolerance::EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }

  {
    const auto range = 1.0;
    const auto actor1_speed = 1.0;
    const auto actor2_speed = 1.0;
    const auto expected_tau = INF;
    const auto computed_tau =
        tau(range, actor1_speed, actor2_speed, tau_tolerance::EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }
}

//------------------------------------------------------------------------------

TEST(Tau, speed_and_relative_distance1) {
  constexpr auto a1 = 1.0;
  constexpr auto a2 = 0.0;
  constexpr auto t = 1.0;
  constexpr auto v1_0 = 2.7;
  constexpr auto v2 = 3.0;
  constexpr auto D_0 = 10.0;

  const auto state1_at_t = simple_motion(t, v1_0, a1);
  const auto state2_at_t = simple_motion(t, v2, a2);

  const auto D_t = tau_range_at_t(D_0, t, v2, std::get<0>(state1_at_t));
  const auto tau_0 = tau(D_0, v1_0, v2, tau_tolerance::EPS);
  const auto tau_t = tau(D_t, std::get<1>(state1_at_t), v2, tau_tolerance::EPS);

  std::cout << "tau_0: " << tau_0 << ", tau_t: " << tau_t << std::endl;
  std::cout << "D: " << D_0 << ", D_t: " << D_t << std::endl;

  const auto other_speed = compute_other_speed_from_tau(
      tau_0, tau_t, t, std::get<0>(state1_at_t), v1_0, std::get<1>(state1_at_t),
      tau_tolerance::EPS);

  const auto range_t = compute_range_from_other_speed_and_tau(
      t, tau_0, other_speed, v1_0, std::get<0>(state1_at_t));

  EXPECT_NEAR(other_speed, v2, tau_tolerance::EPS);
  EXPECT_NEAR(range_t, D_t, tau_tolerance::EPS);
}

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
  const auto tau_estimated =
      tauFromDiscreteScaleDt(e2, e_dot, t_delta, tau_tolerance::EPS);
  EXPECT_NEAR(tau_estimated, tau, 0.0001)
      << "e1: " << e1 << ", e2: " << e2 << ", e_dot: " << e_dot;
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
