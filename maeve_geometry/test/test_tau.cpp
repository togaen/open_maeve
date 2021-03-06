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

#include "open_maeve/maeve_geometry/tau.h"

namespace open_maeve {
namespace {
constexpr auto EPS = 1e-4;
constexpr auto INF = std::numeric_limits<double>::infinity();

//------------------------------------------------------------------------------

double extent(const double Z, const Eigen::Vector2d& p1,
              const Eigen::Vector2d& p2) {
  const auto d = (p1 - p2).norm();
  return d / Z;
}

//------------------------------------------------------------------------------

}  // namespace

//------------------------------------------------------------------------------

TEST(Tau, tau1) {
  {
    const auto range = 0.0;
    const auto relative_speed = 0.0;
    const auto expected_tau = 0.0;
    const auto computed_tau = tau(range, relative_speed, EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }

  {
    const auto range = 1.0;
    const auto relative_speed = 1.0;
    const auto expected_tau = 1.0;
    const auto computed_tau = tau(range, relative_speed, EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }

  {
    const auto range = 1.0;
    const auto relative_speed = -1.0;
    const auto expected_tau = -1.0;
    const auto computed_tau = tau(range, relative_speed, EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }

  {
    const auto range = 1.0;
    const auto relative_speed = 0.0;
    const auto expected_tau = INF;
    const auto computed_tau = tau(range, relative_speed, EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }

  {
    const auto range = 1.0;
    const auto relative_speed = (0.5 * EPS);
    const auto expected_tau = INF;
    const auto computed_tau = tau(range, relative_speed, EPS);
    EXPECT_EQ(computed_tau, expected_tau);
  }

  {
    const auto range = 1.0;
    const auto relative_speed = (2.0 * EPS);
    const auto computed_tau = tau(range, relative_speed, EPS);
    EXPECT_TRUE(std::isfinite(computed_tau));
  }
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
  const auto tau_estimated = tauFromDiscreteScaleDt(e2, e_dot, t_delta, EPS);
  EXPECT_NEAR(tau_estimated, tau, 0.0001)
      << "e1: " << e1 << ", e2: " << e2 << ", e_dot: " << e_dot;
}

//------------------------------------------------------------------------------

}  // namespace open_maeve
