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
namespace pst {
TEST(Maeve_Dynamics_PST_Reachability, testReachability) {
  const auto p1 = Eigen::Vector2d(0, 0);
  const auto p2 = Eigen::Vector2d(5, 5);
  const auto v_i = Interval(1, 1);
  const auto c = IntervalConstraints<2>(Interval(-1, -1),
                                        {Interval(0, 1), Interval(-4, 4)});

  {
    const auto r = reachability<Type::I>(p1, v_i, p2, c);
    EXPECT_FALSE(!r);
  }
  {
    const auto r = reachability<Type::II>(p1, v_i, p2, c);
    EXPECT_FALSE(!r);
  }
  {
    const auto r = reachability<Type::III>(p1, v_i, p2, c);
    EXPECT_FALSE(!r);
  }
  {
    const auto r = reachability<Type::IV>(p1, v_i, p2, c);
    EXPECT_FALSE(!r);
  }
  {
    const auto r = reachability<Type::V>(p1, v_i, p2, c);
    EXPECT_FALSE(!r);
  }
  {
    const auto r = reachability<Type::VI>(p1, v_i, p2, c);
    EXPECT_FALSE(!r);
  }
  {
    const auto r = reachability<Type::VII>(p1, v_i, p2, c);
    EXPECT_FALSE(!r);
  }
  {
    const auto r = reachability<Type::VIII>(p1, v_i, p2, c);
    EXPECT_FALSE(!r);
  }
}
}  // namespace pst
}  // namespace maeve_automation_core
