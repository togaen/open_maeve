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

#include "open_maeve/maeve_dynamics/pst_reachability.h"

namespace open_maeve {
TEST(Maeve_Dynamics_PST_Reachability, testTargetTerminalSpeed) {
  const Eigen::Vector2d p1(0.0, 0.0);
  const auto speed = 1.0;
  auto eps_bounds = Interval<double>(-1e-6, 1e-6);
  auto t_bounds = Interval<double>(0.0, 10.0);
  auto s_bounds = Interval<double>(0.0, 10.0);
  auto s_dot_bounds = Interval<double>(0.0, 50.0);
  auto s_ddot_bounds = Interval<double>(-2.0, 2.0);
  constexpr auto ORDER = 2;
  const auto constraints = IntervalConstraints<ORDER, double>(
      eps_bounds, t_bounds, {s_bounds, s_dot_bounds, s_ddot_bounds});

  {
    const Eigen::Vector2d p2(1.0, 0.2);

    EXPECT_NO_THROW({
      const auto connector = PST_Reachability::targetTerminalSpeed(
          Interval<double>(speed, speed), p1, p2,
          Interval<double>::max(s_bounds), constraints);
      ASSERT_FALSE(!connector);
      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(
          ss.str(),
          "{\"parabolic_segments\": [{\"a\": -2.00000, \"b\": 1.00000, "
          "\"c\": 0.00000, \"domain\": {\"min\": 0.00000, \"max\": 0.25000}}, "
          "{\"a\": 0.00000, \"b\": 0.00000, \"c\": 0.12500, \"domain\": "
          "{\"min\": "
          "0.25000, \"max\": 0.80635}}, {\"a\": 2.00000, \"b\": -3.22540, "
          "\"c\": 1.42540, \"domain\": {\"min\": 0.80635, \"max\": "
          "1.00000}}]}");
    });
  }

  {
    const Eigen::Vector2d p2(1.0, 0.5);

    EXPECT_NO_THROW({
      const auto connector = PST_Reachability::targetTerminalSpeed(
          Interval<double>(speed, speed), p1, p2,
          Interval<double>::max(s_bounds), constraints);
      const auto reachability = PST_Reachability::compute(
          Interval<double>(speed, speed), p1, p2, constraints);
      ASSERT_FALSE(!reachability);
      ASSERT_FALSE(!connector);
      std::stringstream ss;
      ss << *reachability;
      EXPECT_EQ(
          ss.str(),
          "{\"min_speed_connector\": {\"parabolic_segments\": [{\"a\": "
          "-2.00000, \"b\": 1.00000, \"c\": -0.00000, \"domain\": {\"min\": "
          "0.00000, \"max\": 0.12500}}, {\"a\": 0.00000, \"b\": 0.50000, "
          "\"c\": 0.03125, \"domain\": {\"min\": 0.12500, \"max\": 0.87500}}, "
          "{\"a\": -2.00000, \"b\": 4.00000, \"c\": -1.50000, \"domain\": "
          "{\"min\": 0.87500, \"max\": 1.00000}}]}, \"max_speed_connector\": "
          "{\"parabolic_segments\": [{\"a\": -2.00000, \"b\": 1.00000, "
          "\"c\": 0.00000, \"domain\": {\"min\": 0.00000, \"max\": 0.25000}}, "
          "{\"a\": 0.00000, \"b\": 0.00000, \"c\": 0.12500, \"domain\": "
          "{\"min\": "
          "0.25000, \"max\": 0.56699}}, {\"a\": 2.00000, \"b\": -2.26795, "
          "\"c\": 0.76795, \"domain\": {\"min\": 0.56699, \"max\": "
          "1.00000}}]}}");
    });
  }

  {
    const auto plot_speed = 5.0;
    const Eigen::Vector2d p2(1.0, 0.0);
    auto p = 3.0;  // p1.y();
    const auto path_inc = 0.005;
    while (p < Interval<double>::max(s_bounds)) {
      auto entered_feasible_region = false;
      auto exited_feasible_region = false;
      ASSERT_NO_THROW({
        const auto reachability = PST_Reachability::compute(
            Interval<double>(plot_speed, plot_speed), p1,
            Eigen::Vector2d(p2.x(), p), constraints);
        if (reachability) {
          ASSERT_FALSE(exited_feasible_region);
          entered_feasible_region = true;
#if 0
          std::cout << "{\"p\": " << p << ", \"interval\": "
                    << PST_Reachability::reachableInterval<double>(*reachability)
                    << "}," << std::endl;
#endif
        } else {
          if (entered_feasible_region) {
            exited_feasible_region = true;
          }
        }
      });

      p += path_inc;
    }
  }
}
}  // namespace open_maeve
