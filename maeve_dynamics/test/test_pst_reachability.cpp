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

#include "maeve_automation_core/maeve_dynamics/pst_reachability.h"

namespace maeve_automation_core {
TEST(Maeve_Dynamics_PST_Reachability, testTargetTerminalSpeed) {
  const Eigen::Vector2d p1(0.0, 0.0);
  const auto speed = 1.0;
  auto eps_bounds = Interval(-1e-6, 1e-6);
  auto t_bounds = Interval(0.0, 10.0);
  auto s_bounds = Interval(0.0, 10.0);
  auto s_dot_bounds = Interval(0.0, 50.0);
  auto s_ddot_bounds = Interval(-2.0, 2.0);
  const auto constraints = IntervalConstraints<2>(
      eps_bounds, t_bounds, {s_bounds, s_dot_bounds, s_ddot_bounds});

  {
    const Eigen::Vector2d p2(1.0, 0.2);

    EXPECT_NO_THROW({
      const auto connector = PST_Reachability::targetTerminalSpeed(
          Interval(speed, speed), p1, p2, Interval::max(s_bounds), constraints);
      ASSERT_FALSE(!connector);
      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(ss.str(),
                "{\"switching_times\": [0, 0.25, 0.806351, 1], "
                "\"parabola_coefficients\": [{\"a\": -2.00000, \"b\": 1.00000, "
                "\"c\": 0.00000}, {\"a\": 0.00000, \"b\": 0.00000, \"c\": "
                "0.12500}, {\"a\": 2.00000, \"b\": -3.22540, \"c\": "
                "1.42540}]}");
    });
  }

  {
    const Eigen::Vector2d p2(1.0, 0.5);

    EXPECT_NO_THROW({
      const auto connector = PST_Reachability::targetTerminalSpeed(
          Interval(speed, speed), p1, p2, Interval::max(s_bounds), constraints);
      const auto reachability = PST_Reachability::compute(
          Interval(speed, speed), p1, p2, constraints);
      ASSERT_FALSE(!reachability);
      ASSERT_FALSE(!connector);
      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(ss.str(),
                "{\"switching_times\": [0, 0.25, 0.566987, 1], "
                "\"parabola_coefficients\": [{\"a\": -2.00000, \"b\": 1.00000, "
                "\"c\": 0.00000}, {\"a\": 0.00000, \"b\": 0.00000, \"c\": "
                "0.12500}, {\"a\": 2.00000, \"b\": -2.26795, \"c\": "
                "0.76795}]}");
    });
  }

  {
    const auto plot_speed = 5.0;
    const Eigen::Vector2d p2(1.0, 0.0);
    auto p = 3.0;  // p1.y();
    const auto path_inc = 0.005;
    while (p < Interval::max(s_bounds)) {
      auto entered_feasible_region = false;
      auto exited_feasible_region = false;
      ASSERT_NO_THROW({
        const auto reachability =
            PST_Reachability::compute(Interval(plot_speed, plot_speed), p1,
                                      Eigen::Vector2d(p2.x(), p), constraints);
        if (reachability) {
          ASSERT_FALSE(exited_feasible_region);
          entered_feasible_region = true;
#if 0
          std::cout << "{\"p\": " << p << ", \"interval\": "
                    << PST_Reachability::reachableInterval(*reachability)
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
}  // namespace maeve_automation_core
