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

//------------------------------------------------------------------------------

double extent(const double Z, const Eigen::Vector2d& p1,
              const Eigen::Vector2d& p2) {
  const auto d = (p1 - p2).norm();
  return d / Z;
}

//------------------------------------------------------------------------------

std::tuple<double, double> simple_motion(const double t, const double v_0,
                                         const double a) {
  const auto speed = (v_0 + a * t);
  const auto distance = (t * (v_0 + 0.5 * a * t));
  return std::make_tuple(distance, speed);
}

//------------------------------------------------------------------------------
struct range_and_speed_info {
  const double expected_range;
  const double expected_speed;
  const double computed_range;
  const double computed_speed;
};
range_and_speed_info computed_speed_and_range(const double actor1_accel,
                                              const double actor2_accel,
                                              const double t,
                                              const double actor1_speed_0,
                                              const double actor2_speed_0,
                                              const double range_0) {
  const auto actor1_state_at_t = simple_motion(t, actor1_speed_0, actor1_accel);
  const auto actor2_state_at_t = simple_motion(t, actor2_speed_0, actor2_accel);

  const auto range_t = tau_range_at_t(range_0, std::get<0>(actor1_state_at_t),
                                      std::get<0>(actor2_state_at_t));
  const auto tau_0 =
      tau(range_0, actor1_speed_0, actor2_speed_0, tau_tolerance::EPS);
  const auto tau_t = tau(range_t, std::get<1>(actor1_state_at_t),
                         std::get<1>(actor2_state_at_t), tau_tolerance::EPS);
#if 0
  std::cout << "v1_0: " << actor1_speed_0
            << ", v1_t: " << std::get<1>(actor1_state_at_t) << std::endl;
  std::cout << "t: " << t << ", tau_0: " << tau_0 << ", tau_t: " << tau_t
            << ", p1_delta: " << std::get<0>(actor1_state_at_t)
            << ", range_t: " << range_t << std::endl;
#endif
  const auto computed_actor2_speed = compute_actor2_speed_from_tau(
      tau_0, tau_t, t, std::get<0>(actor1_state_at_t), actor1_speed_0,
      std::get<1>(actor1_state_at_t), tau_tolerance::EPS);

  const auto computed_range_t = compute_range_from_actor2_speed_and_tau(
      t, tau_0, computed_actor2_speed, actor1_speed_0,
      std::get<0>(actor1_state_at_t));

  return {range_t, std::get<1>(actor2_state_at_t), computed_range_t,
          computed_actor2_speed};
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

TEST(Tau, compute_relative_speed_for_tau) {
  {
    const auto actor1_speed = 2.0;
    const auto actor2_speed = 1.0;
    const auto expected_relative_speed = 1.0;
    const auto computed_relative_speed =
        compute_relative_speed_for_tau(actor1_speed, actor2_speed);
    EXPECT_EQ(computed_relative_speed, expected_relative_speed);
  }

  {
    const auto actor1_speed = 1.0;
    const auto actor2_speed = 2.0;
    const auto expected_relative_speed = -1.0;
    const auto computed_relative_speed =
        compute_relative_speed_for_tau(actor1_speed, actor2_speed);
    EXPECT_EQ(computed_relative_speed, expected_relative_speed);
  }

  {
    const auto actor1_speed = 1.0;
    const auto actor2_speed = 1.0;
    const auto expected_relative_speed = 0.0;
    const auto computed_relative_speed =
        compute_relative_speed_for_tau(actor1_speed, actor2_speed);
    EXPECT_EQ(computed_relative_speed, expected_relative_speed);
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

TEST(Tau, tau_range1) {
  {
    const auto tau_0 = 1.0;
    const auto relative_speed = 1.0;
    const auto expected_range = 1.0;
    const auto computed_range = tau_range(tau_0, relative_speed);
    EXPECT_EQ(computed_range, expected_range);
  }

  {
    const auto tau_0 = 1.0;
    const auto relative_speed = 10.0;
    const auto expected_range = 10.0;
    const auto computed_range = tau_range(tau_0, relative_speed);
    EXPECT_EQ(computed_range, expected_range);
  }

  {
    const auto tau_0 = 0.5;
    const auto relative_speed = 10.0;
    const auto expected_range = 5.0;
    const auto computed_range = tau_range(tau_0, relative_speed);
    EXPECT_EQ(computed_range, expected_range);
  }

  {
    const auto tau_0 = -1.0;
    const auto relative_speed = -1.0;
    const auto expected_range = 1.0;
    const auto computed_range = tau_range(tau_0, relative_speed);
    EXPECT_EQ(computed_range, expected_range);
  }
}

//------------------------------------------------------------------------------

TEST(Tau, tau_range2) {
  {
    const auto tau_0 = 1.0;
    const auto actor1_speed = 2.0;
    const auto actor2_speed = 1.0;
    const auto expected_range = 1.0;
    const auto computed_range = tau_range(tau_0, actor1_speed, actor2_speed);
    EXPECT_EQ(computed_range, expected_range);
  }

  {
    const auto tau_0 = 1.0;
    const auto actor1_speed = 11.0;
    const auto actor2_speed = 1.0;
    const auto expected_range = 10.0;
    const auto computed_range = tau_range(tau_0, actor1_speed, actor2_speed);
    EXPECT_EQ(computed_range, expected_range);
  }

  {
    const auto tau_0 = 0.5;
    const auto actor1_speed = 11.0;
    const auto actor2_speed = 1.0;
    const auto expected_range = 5.0;
    const auto computed_range = tau_range(tau_0, actor1_speed, actor2_speed);
    EXPECT_EQ(computed_range, expected_range);
  }

  {
    const auto tau_0 = -1.0;
    const auto actor1_speed = 1.0;
    const auto actor2_speed = 2.0;
    const auto expected_range = 1.0;
    const auto computed_range = tau_range(tau_0, actor1_speed, actor2_speed);
    EXPECT_EQ(computed_range, expected_range);
  }
}

//------------------------------------------------------------------------------

TEST(Tau, tau_range_at_t) {
  {
    const auto range_0 = 10.0;
    const auto t = 1.0;
    const auto actor2_speed = 1.0;
    const auto actor1_distance_delta = 1.0;
    const auto expected_range_t = 10.0;
    const auto computed_range_t =
        tau_range_at_t(range_0, t, actor2_speed, actor1_distance_delta);
    EXPECT_EQ(computed_range_t, expected_range_t);
  }

  {
    const auto range_0 = 10.0;
    const auto t = 1.0;
    const auto actor2_speed = 1.0;
    const auto actor1_distance_delta = 2.0;
    const auto expected_range_t = 9.0;
    const auto computed_range_t =
        tau_range_at_t(range_0, t, actor2_speed, actor1_distance_delta);
    EXPECT_EQ(computed_range_t, expected_range_t);
  }
}

//------------------------------------------------------------------------------

TEST(Tau, range_and_actor2_speed_insufficient_information) {
  constexpr auto actor1_accel = 0.0;
  constexpr auto actor2_accel = 0.0;
  constexpr auto t = 1.0;
  constexpr auto actor1_speed_0 = 2.0;
  constexpr auto actor2_speed_0 = 1.0;
  constexpr auto range_0 = 10.0;

  const auto range_and_speed = computed_speed_and_range(
      actor1_accel, actor2_accel, t, actor1_speed_0, actor2_speed_0, range_0);

  EXPECT_TRUE(std::isnan(range_and_speed.computed_range));
  EXPECT_TRUE(std::isnan(range_and_speed.computed_speed));
}

//------------------------------------------------------------------------------

TEST(Tau, range_and_actor2_speed_exact) {
  constexpr auto actor1_accel = 1.0;
  constexpr auto actor2_accel = 0.0;
  constexpr auto t = 1.0;
  constexpr auto actor1_speed_0 = 2.0;
  constexpr auto actor2_speed_0 = 1.0;
  constexpr auto range_0 = 10.0;

  const auto range_and_speed = computed_speed_and_range(
      actor1_accel, actor2_accel, t, actor1_speed_0, actor2_speed_0, range_0);

  EXPECT_EQ(range_and_speed.computed_speed, range_and_speed.expected_speed);
  EXPECT_EQ(range_and_speed.computed_range, range_and_speed.expected_range);
}

//------------------------------------------------------------------------------

TEST(Tau, range_and_actor2_speed_approximate0) {
  constexpr auto actor1_accel = 1.0;
  constexpr auto actor2_accel = 0.0;
  constexpr auto t = 0.1;
  constexpr auto actor1_speed_0 = 25.0;
  constexpr auto actor2_speed_0 = 20.0;
  constexpr auto range_0 = 50.0;

  const auto range_and_speed = computed_speed_and_range(
      actor1_accel, actor2_accel, t, actor1_speed_0, actor2_speed_0, range_0);

  EXPECT_NEAR(range_and_speed.computed_speed, range_and_speed.expected_speed,
              tau_tolerance::EPS);
  EXPECT_NEAR(range_and_speed.computed_range, range_and_speed.expected_range,
              tau_tolerance::EPS);
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
