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
  {
    const Eigen::Vector2d p1(3.0, 2.0);
    const Eigen::Vector2d p2(7.0, 7.5);
    auto t_bounds = Interval(0.0, 10.0);
    auto s_bounds = Interval(0.0, 10.0);
    auto s_dot_bounds = Interval(0.0, 5.0);
    auto s_ddot_bounds = Interval(-1.0, 2.0);
    const auto constraints = IntervalConstraints<2>(
        t_bounds, {s_bounds, s_dot_bounds, s_ddot_bounds});

    const auto speed = Interval::min(s_dot_bounds);
    ASSERT_NO_THROW({
      const auto reachability = PST_Reachability::compute(
          Interval(speed, speed), p1, p2, constraints);
      if (reachability) {
        std::cout << PST_Reachability::reachableInterval(*reachability)
                  << std::endl;
        const auto& min_connector =
            PST_Reachability::minConnector(*reachability);
        const auto& max_connector =
            PST_Reachability::maxConnector(*reachability);
        std::cout << "min: " << min_connector << std::endl;
        std::cout << "max: " << max_connector << std::endl;
      } else {
        std::cout << "Not reachable for initial speed: " << speed << std::endl;
      }
    });
  }

#if 0
compute(
      const Interval& I_i, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints)

  IntervalConstraints(Interval&& t_bounds,
                      std::array<Interval, Order + 1>&& s_bounds);

#endif
  const Eigen::Vector2d p1(3.0, 2.0);
  const Eigen::Vector2d p2(4.0, 7.5);
  auto t_bounds = Interval(0.0, 10.0);
  auto s_bounds = Interval(0.0, 10.0);
  auto s_dot_bounds = Interval(0.0, 5.0);
  auto s_ddot_bounds = Interval(-1.0, 2.0);
  const auto constraints =
      IntervalConstraints<2>(t_bounds, {s_bounds, s_dot_bounds, s_ddot_bounds});

  const auto start_speed = Interval::min(s_dot_bounds);
  const auto speed_inc = 0.01;
  auto speed = start_speed;
  const auto end_speed = Interval::max(s_dot_bounds);
  while (speed <= end_speed) {
    ASSERT_NO_THROW({
      const auto reachability = PST_Reachability::compute(
          Interval(speed, speed), p1, p2, constraints);
      if (reachability) {
        std::cout << PST_Reachability::reachableInterval(*reachability)
                  << std::endl;
      } else {
        std::cout << "Not reachable for initial speed: " << speed << std::endl;
      }
    });

    speed += speed_inc;
  }
}
}  // namespace maeve_automation_core
