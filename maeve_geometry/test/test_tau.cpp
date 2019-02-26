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
struct info_info {
  const double range_t;
  const double computed_range_t;
  const double computed_actor2_speed_t;
  const double actor1_speed_t;
  const double actor2_speed_t;
  const double tau_0;
  const double tau_t;
};

#define P(v, x) #x << ": " << v.x

std::ostream& operator<<(std::ostream& os, const info_info& i) {
  os << "{" << P(i, range_t) << ", " << P(i, computed_range_t) << ", "
     << P(i, computed_actor2_speed_t) << ", " << P(i, actor1_speed_t) << ", "
     << P(i, actor2_speed_t) << ", " << P(i, tau_0) << ", " << P(i, tau_t)
     << "}";
  return os;
}

info_info compute_problem_info(const double actor1_accel,
                               const double actor2_accel, const double t,
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

  const auto computed_actor2_speed = compute_actor2_speed_from_tau(
      tau_0, tau_t, t, std::get<0>(actor1_state_at_t), actor1_speed_0,
      std::get<1>(actor1_state_at_t), tau_tolerance::EPS);

  const auto computed_range_t =
      tau_range_at_t(t, tau_0, computed_actor2_speed, actor1_speed_0,
                     std::get<0>(actor1_state_at_t));

  return {range_t,
          computed_range_t,
          computed_actor2_speed,
          std::get<1>(actor1_state_at_t),
          std::get<1>(actor2_state_at_t),
          tau_0,
          tau_t};
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

TEST(Tau, compute_relative_dynamics_for_tau) {
  {
    const auto actor1_speed = 2.0;
    const auto actor2_speed = 1.0;
    const auto expected_relative_speed = 1.0;
    const auto computed_relative_speed =
        compute_relative_dynamics_for_tau(actor1_speed, actor2_speed);
    EXPECT_EQ(computed_relative_speed, expected_relative_speed);
  }

  {
    const auto actor1_speed = 1.0;
    const auto actor2_speed = 2.0;
    const auto expected_relative_speed = -1.0;
    const auto computed_relative_speed =
        compute_relative_dynamics_for_tau(actor1_speed, actor2_speed);
    EXPECT_EQ(computed_relative_speed, expected_relative_speed);
  }

  {
    const auto actor1_speed = 1.0;
    const auto actor2_speed = 1.0;
    const auto expected_relative_speed = 0.0;
    const auto computed_relative_speed =
        compute_relative_dynamics_for_tau(actor1_speed, actor2_speed);
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
    const auto range_t = 1.0;
    const auto computed_range_t = tau_range(tau_0, relative_speed);
    EXPECT_EQ(computed_range_t, range_t);
  }

  {
    const auto tau_0 = 1.0;
    const auto relative_speed = 10.0;
    const auto range_t = 10.0;
    const auto computed_range_t = tau_range(tau_0, relative_speed);
    EXPECT_EQ(computed_range_t, range_t);
  }

  {
    const auto tau_0 = 0.5;
    const auto relative_speed = 10.0;
    const auto range_t = 5.0;
    const auto computed_range_t = tau_range(tau_0, relative_speed);
    EXPECT_EQ(computed_range_t, range_t);
  }

  {
    const auto tau_0 = -1.0;
    const auto relative_speed = -1.0;
    const auto range_t = 1.0;
    const auto computed_range_t = tau_range(tau_0, relative_speed);
    EXPECT_EQ(computed_range_t, range_t);
  }
}

//------------------------------------------------------------------------------

TEST(Tau, tau_range2) {
  {
    const auto tau_0 = 1.0;
    const auto actor1_speed = 2.0;
    const auto actor2_speed = 1.0;
    const auto range_t = 1.0;
    const auto computed_range_t = tau_range(tau_0, actor1_speed, actor2_speed);
    EXPECT_EQ(computed_range_t, range_t);
  }

  {
    const auto tau_0 = 1.0;
    const auto actor1_speed = 11.0;
    const auto actor2_speed = 1.0;
    const auto range_t = 10.0;
    const auto computed_range_t = tau_range(tau_0, actor1_speed, actor2_speed);
    EXPECT_EQ(computed_range_t, range_t);
  }

  {
    const auto tau_0 = 0.5;
    const auto actor1_speed = 11.0;
    const auto actor2_speed = 1.0;
    const auto range_t = 5.0;
    const auto computed_range_t = tau_range(tau_0, actor1_speed, actor2_speed);
    EXPECT_EQ(computed_range_t, range_t);
  }

  {
    const auto tau_0 = -1.0;
    const auto actor1_speed = 1.0;
    const auto actor2_speed = 2.0;
    const auto range_t = 1.0;
    const auto computed_range_t = tau_range(tau_0, actor1_speed, actor2_speed);
    EXPECT_EQ(computed_range_t, range_t);
  }
}

//------------------------------------------------------------------------------

TEST(Tau, tau_range_at_t) {
  {
    const auto range_0 = 10.0;
    const auto t = 1.0;
    const auto actor2_speed = 1.0;
    const auto actor1_distance_delta = 1.0;
    const auto range_t_t = 10.0;
    const auto computed_range_t_t =
        tau_range_at_t(range_0, t, actor2_speed, actor1_distance_delta);
    EXPECT_EQ(computed_range_t_t, range_t_t);
  }

  {
    const auto range_0 = 10.0;
    const auto t = 1.0;
    const auto actor2_speed = 1.0;
    const auto actor1_distance_delta = 2.0;
    const auto range_t_t = 9.0;
    const auto computed_range_t_t =
        tau_range_at_t(range_0, t, actor2_speed, actor1_distance_delta);
    EXPECT_EQ(computed_range_t_t, range_t_t);
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

  const auto info = compute_problem_info(
      actor1_accel, actor2_accel, t, actor1_speed_0, actor2_speed_0, range_0);

  EXPECT_TRUE(std::isnan(info.computed_range_t));
  EXPECT_TRUE(std::isnan(info.computed_actor2_speed_t));
}

//------------------------------------------------------------------------------

TEST(Tau, range_and_actor2_speed_exact) {
  constexpr auto actor1_accel = 1.0;
  constexpr auto actor2_accel = 0.0;
  constexpr auto t = 1.0;
  constexpr auto actor1_speed_0 = 2.0;
  constexpr auto actor2_speed_0 = 1.0;
  constexpr auto range_0 = 10.0;

  const auto info = compute_problem_info(
      actor1_accel, actor2_accel, t, actor1_speed_0, actor2_speed_0, range_0);

  EXPECT_EQ(info.computed_actor2_speed_t, info.actor2_speed_t);
  EXPECT_EQ(info.computed_range_t, info.range_t);
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

TEST(Tau, compute_actor2_speed_0_1) {
  constexpr auto actor1_accel = 1.0;
  constexpr auto actor2_accel = 0.0;
  constexpr auto t = 1.0;
  constexpr auto actor1_speed_0 = 2.0;
  constexpr auto actor2_speed_0 = 1.0;
  constexpr auto range_0 = 10.0;

  const auto info = compute_problem_info(
      actor1_accel, actor2_accel, t, actor1_speed_0, actor2_speed_0, range_0);

  const auto computed_actor2_speed_t_0 =
      tau_actor2_speed(info.tau_0, actor1_speed_0, range_0, tau_tolerance::EPS);
  EXPECT_EQ(computed_actor2_speed_t_0, actor2_speed_0);

  const auto computed_actor2_speed_t_t = tau_actor2_speed(
      info.tau_t, info.actor1_speed_t, info.range_t, tau_tolerance::EPS);
  EXPECT_EQ(computed_actor2_speed_t_t, info.actor2_speed_t);
}

//------------------------------------------------------------------------------

TEST(Tau, compute_actor2_speed_0_2) {
  constexpr auto actor1_accel = 3.1;
  constexpr auto actor2_accel = 1.37;
  constexpr auto t = 1.0;
  constexpr auto actor1_speed_0 = 2.0;
  constexpr auto actor2_speed_0 = 1.23;
  constexpr auto range_0 = 10.0;

  const auto info = compute_problem_info(
      actor1_accel, actor2_accel, t, actor1_speed_0, actor2_speed_0, range_0);

  const auto computed_actor2_speed_t_0 =
      tau_actor2_speed(info.tau_0, actor1_speed_0, range_0, tau_tolerance::EPS);
  EXPECT_EQ(computed_actor2_speed_t_0, actor2_speed_0);

  const auto computed_actor2_speed_t_t = tau_actor2_speed(
      info.tau_t, info.actor1_speed_t, info.range_t, tau_tolerance::EPS);
  EXPECT_EQ(computed_actor2_speed_t_t, info.actor2_speed_t);
}

//------------------------------------------------------------------------------

TEST(Tau, compute_actor2_speed_0_3) {
  constexpr auto actor1_accel = 3.1;
  constexpr auto actor2_accel = 1.37;
  constexpr auto t = 1.0;
  constexpr auto actor1_speed_0 = 2.0;
  constexpr auto actor2_speed_0 = 1.23;
  constexpr auto range_0 = 0.0;

  const auto info = compute_problem_info(
      actor1_accel, actor2_accel, t, actor1_speed_0, actor2_speed_0, range_0);

  const auto computed_actor2_speed_t_0 =
      tau_actor2_speed(info.tau_0, actor1_speed_0, range_0, tau_tolerance::EPS);

  EXPECT_TRUE(std::isnan(computed_actor2_speed_t_0));
}

//------------------------------------------------------------------------------

TEST(Tau, tau_at_t_0) {
  constexpr auto actor1_accel = 3.1;
  constexpr auto actor2_accel = 1.37;
  constexpr auto t = 1.0;
  constexpr auto actor1_speed_0 = 2.0;
  constexpr auto actor2_speed_0 = 1.23;
  constexpr auto range_0 = 10.0;

  const auto info = compute_problem_info(
      actor1_accel, actor2_accel, t, actor1_speed_0, actor2_speed_0, range_0);

  const double computed_tau_t =
      tau_at_t(range_0, t, actor1_speed_0, actor2_speed_0, actor1_accel,
               actor2_accel, tau_tolerance::EPS);

  EXPECT_EQ(computed_tau_t, info.tau_t);
}

//------------------------------------------------------------------------------

TEST(Tau, tau_at_t_1) {
  constexpr auto actor1_accel = 0.0;
  constexpr auto actor2_accel = 0.0;
  constexpr auto t = 1.0;
  constexpr auto actor1_speed_0 = 2.0;
  constexpr auto actor2_speed_0 = 1.0;
  constexpr auto range_0 = 10.0;

  const auto info = compute_problem_info(
      actor1_accel, actor2_accel, t, actor1_speed_0, actor2_speed_0, range_0);

  const double computed_tau_t =
      tau_at_t(range_0, t, actor1_speed_0, actor2_speed_0, actor1_accel,
               actor2_accel, tau_tolerance::EPS);

  EXPECT_EQ(computed_tau_t, info.tau_t);
}

//------------------------------------------------------------------------------

TEST(Tau, compute_tau_desired_accel) {
  constexpr auto actor1_accel = 0.0;
  constexpr auto actor2_accel = 0.0;
  constexpr auto t = 0.1;
  constexpr auto actor1_speed_0 = 25.0;
  constexpr auto actor2_speed_0 = 20.0;
  constexpr auto range_0 = 20.0;

  constexpr auto a_min = -4.0;
  constexpr auto tau_min = 3.0;

  const auto desired_control =
      compute_tau_desired_accel(t, range_0, actor1_speed_0, actor2_speed_0,
                                a_min, tau_min, tau_tolerance::EPS);

  const auto tau_t = tau_at_t(range_0, t, actor1_speed_0, actor2_speed_0,
                              desired_control, a_min, tau_tolerance::EPS);
  EXPECT_NEAR(tau_t, tau_min, tau_tolerance::EPS);
}

//------------------------------------------------------------------------------

TEST(Tau, compute_tau_desired_accel_singularity) {
  constexpr auto actor1_accel = 0.0;
  constexpr auto actor2_accel = 0.0;
  constexpr auto t = 0.1;
  constexpr auto actor1_speed_0 = 25.0;
  constexpr auto actor2_speed_0 = 20.0;
  constexpr auto range_0 = 20.0;

  constexpr auto a_min = -4.0;
  constexpr auto tau_min = (-0.5 * t);

  EXPECT_THROW(
      compute_tau_desired_accel(t, range_0, actor1_speed_0, actor2_speed_0,
                                a_min, tau_min, tau_tolerance::EPS),
      std::runtime_error);
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
