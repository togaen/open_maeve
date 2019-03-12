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

#include "maeve_automation_core/maeve_dynamics/interval_constraints.h"

namespace maeve_automation_core {
namespace {
const auto eps_bounds = Interval<double>(0.0, 0.0);
}  // namespace

TEST(Maeve_Dynamics_Interval_Constraints, testAccessors) {
  {
    const auto c = IntervalConstraints<2, double>(
        eps_bounds, Interval<double>(0, 1),
        {Interval<double>(2, 3), Interval<double>(4, 5)});

    const auto& t = IntervalConstraints<2, double>::boundsT(c);
    EXPECT_EQ(t, Interval<double>(0, 1));

    const auto& s0 = IntervalConstraints<2, double>::boundsS<0>(c);
    EXPECT_EQ(s0, Interval<double>(2, 3));

    const auto& s1 = IntervalConstraints<2, double>::boundsS<1>(c);
    EXPECT_EQ(s1, Interval<double>(4, 5));
  }

  {
    const auto c = IntervalConstraints<2, double>(
        eps_bounds, Interval<double>(0, 1),
        {Interval<double>(2, 3), Interval<double>(4, 5)});

    // This should not compile.
    // const auto& s = IntervalConstraints<2, double>::boundsS<3>(c);
    // EXPECT_TRUE(false);
  }
}

TEST(Maeve_Dynamics_Interval_Constraints, testIntersect) {
  {
    const auto c1 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(0, 1),
        {Interval<double>(0, 1), Interval<double>(2, 3)});
    const auto c2 = c1;
    const auto c = IntervalConstraints<1, double>::intersect(c1, c2);
    EXPECT_EQ(c, c1);
  }

  {
    const auto c1 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(0, 1),
        {Interval<double>(0, 1), Interval<double>(2, 3)});
    const auto c2 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(),
        {Interval<double>(0, 1), Interval<double>(2, 3)});
    const auto c = IntervalConstraints<1, double>::intersect(c1, c2);
    EXPECT_NE(c, c1);

    const auto c3 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(),
        {Interval<double>(0, 1), Interval<double>(2, 3)});
    EXPECT_EQ(c, c3);
  }

  {
    const auto c1 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(0, 1),
        {Interval<double>(), Interval<double>(2, 3)});
    const auto c2 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(),
        {Interval<double>(0, 1), Interval<double>(2, 3)});
    const auto c = IntervalConstraints<1, double>::intersect(c1, c2);
    EXPECT_NE(c, c1);

    const auto c3 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(),
        {Interval<double>(), Interval<double>(2, 3)});
    EXPECT_EQ(c, c3);
  }

  {
    const auto c1 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(0, 1),
        {Interval<double>(0, 1), Interval<double>(2, 3)});
    const auto c2 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(-2, 5),
        {Interval<double>(0.5, 0.75), Interval<double>(2.3, 3.7)});
    const auto c = IntervalConstraints<1, double>::intersect(c1, c2);
    EXPECT_NE(c, c1);

    const auto c3 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(0, 1),
        {Interval<double>(0.5, 0.75), Interval<double>(2.3, 3)});
    EXPECT_EQ(c, c3);
  }
}

TEST(Maeve_Dynamics_Interval_Constraints, testSatisfiable) {
  {
    const auto c = IntervalConstraints<0, double>(
        eps_bounds, Interval<double>(0, 1), {Interval<double>()});
    EXPECT_FALSE((IntervalConstraints<0, double>::satisfiable(c)));
  }

  {
    const auto c = IntervalConstraints<0, double>(
        eps_bounds, Interval<double>(), {Interval<double>(0, 1)});
    EXPECT_FALSE((IntervalConstraints<0, double>::satisfiable(c)));
  }

  {
    const auto c = IntervalConstraints<0, double>(
        eps_bounds, Interval<double>(0, 1), {Interval<double>(0, 1)});
    EXPECT_TRUE((IntervalConstraints<0, double>::satisfiable(c)));
  }
}

TEST(Maeve_Dynamics_Interval_Constraints, testValid) {
  {
    const auto c = IntervalConstraints<0, double>(
        eps_bounds, Interval<double>(0, 1), {Interval<double>(0, 1)});
    EXPECT_TRUE((IntervalConstraints<0, double>::satisfiable(c)));
  }
}

TEST(Maeve_Dynamics_Interval_Constraints, testComparisons) {
  {
    const auto c1 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(0, 1),
        {Interval<double>(0, 1), Interval<double>(2, 3)});
    const auto c2 = c1;
    EXPECT_EQ(c1, c2);
    EXPECT_FALSE((c1 != c2));
  }

  {
    const auto c1 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(0, 1),
        {Interval<double>(0, 1), Interval<double>(2, 3)});
    const auto c2 = IntervalConstraints<1, double>(
        eps_bounds, Interval<double>(1, 2),
        {Interval<double>(0, 1), Interval<double>(2, 3)});
    EXPECT_NE(c1, c2);
    EXPECT_FALSE((c1 == c2));
  }
}

TEST(Maeve_Dynamics_Interval_Constraints, testConstruction) {
  {
    const auto c = IntervalConstraints<0, double>(
        eps_bounds, Interval<double>(0, 1), {Interval<double>(0, 1)});
    EXPECT_TRUE(true);
  }

  {
    // This should not compile.
    // const auto c = IntervalConstraints<0, double>(eps_bounds,
    // Interval<double>(0, 1),
    //                                      {Interval<double>(0, 1),
    //                                      Interval<double>(2, 3)});
    // EXPECT_TRUE(false);
  }
}
}  // namespace maeve_automation_core
