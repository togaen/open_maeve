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

#include <cmath>
#include <limits>

#include "maeve_automation_core/maeve_geometry/interval.h"

namespace maeve_automation_core {
namespace {
const auto Inf = std::numeric_limits<double>::infinity();
const auto Min = std::numeric_limits<double>::lowest();
const auto Max = std::numeric_limits<double>::max();
const auto NaN = std::numeric_limits<double>::quiet_NaN();
const auto epsilon = 0.00001;
}  // namespace

TEST(Maeve_Geometry_Interval, testIsSubsetEq) {
  EXPECT_TRUE(Interval::isSubsetEq(Interval(-0.5, 0.5), Interval(-1.0, 1.0)));
  EXPECT_TRUE(Interval::isSubsetEq(Interval(3.2, 4.2), Interval(3.2, 4.2)));
  EXPECT_FALSE(Interval::isSubsetEq(Interval(-3.0, -1.0), Interval(1.0, 3.0)));
  EXPECT_FALSE(Interval::isSubsetEq(Interval(1.0, 3.0), Interval(2.0, 4.0)));
  EXPECT_FALSE(Interval::isSubsetEq(Interval(), Interval()));
}

TEST(Maeve_Geometry_Interval, testProjectToInterval) {
  const auto i = Interval(-1, 1);
  {
    const auto val = 0.0;
    const auto p = Interval::projectToInterval(i, val);
    EXPECT_EQ(p, val);
  }

  {
    const auto val = -3.4;
    const auto p = Interval::projectToInterval(i, val);
    EXPECT_EQ(p, Interval::min(i));
  }

  {
    const auto val = 2.7;
    const auto p = Interval::projectToInterval(i, val);
    EXPECT_EQ(p, Interval::max(i));
  }
}

TEST(Maeve_Geometry_Interval, scale_to_interval) {
  {
    const auto i = Interval();
    EXPECT_TRUE(std::isnan(Interval::scale_to_interval(i, 0.0)));
    EXPECT_TRUE(std::isnan(Interval::scale_to_interval(i, NaN)));
  }

  {
    const auto i = Interval(0.0, 1.0);
    EXPECT_FALSE(std::isnan(Interval::scale_to_interval(i, 0.0)));
    EXPECT_TRUE(std::isnan(Interval::scale_to_interval(i, NaN)));
  }

  {
    const auto i = Interval(0.0, 3.0);
    EXPECT_EQ(Interval::scale_to_interval(i, 0.0), 0.0);
    EXPECT_EQ(Interval::scale_to_interval(i, 1.0), (1.0 / 3.0));
    EXPECT_EQ(Interval::scale_to_interval(i, 1.5), 0.5);
    EXPECT_EQ(Interval::scale_to_interval(i, -1.0), (-1.0 / 3.0));
    EXPECT_EQ(Interval::scale_to_interval(i, 4.0), (4.0 / 3.0));
  }
}

TEST(Maeve_Geometry_Interval, testFactories) {
  EXPECT_EQ(Interval::affinelyExtendedReals(), Interval(-Inf, Inf));
  EXPECT_EQ(Interval::maxRepresentableReals(), Interval(Min, Max));
}

TEST(Maeve_Geometry_Interval, testMerge) {
  {
    const auto i1 = Interval();
    const auto i2 = Interval(0, 1);
    const auto i_opt = Interval::merge(i1, i2);
    ASSERT_FALSE(!i_opt);
    const auto i = *i_opt;
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_EQ(Interval::min(i), 0.0);
    EXPECT_EQ(Interval::max(i), 1.0);
  }

  {
    const auto i_opt = Interval::merge(Interval(), Interval());
    ASSERT_FALSE(!i_opt);
    const auto i = *i_opt;
    EXPECT_TRUE(Interval::empty(i));
  }

  {
    const auto i1 = Interval(2, 3);
    const auto i2 = Interval(0, 1);
    const auto i = Interval::merge(i1, i2);
    EXPECT_TRUE(!i);
  }

  {
    const auto i1 = Interval(0.25, 0.75);
    const auto i2 = Interval(0, 1);
    const auto i_opt = Interval::merge(i1, i2);
    ASSERT_FALSE(!i_opt);
    const auto i = *i_opt;
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_EQ(Interval::min(i), 0.0);
    EXPECT_EQ(Interval::max(i), 1.0);
  }

  {
    const auto i1 = Interval(0.25, 1);
    const auto i2 = Interval(0, 1);
    const auto i_opt = Interval::merge(i1, i2);
    ASSERT_FALSE(!i_opt);
    const auto i = *i_opt;
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_EQ(Interval::min(i), 0.0);
    EXPECT_EQ(Interval::max(i), 1.0);
  }
}

TEST(Maeve_Geometry_Interval, testComparisons) {
  {
    const auto i1 = Interval(0.25, 1);
    const auto i2 = Interval(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval(-0.25, 0.5);
    const auto i2 = Interval(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval(0, 0.5);
    const auto i2 = Interval(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval(0, 1);
    const auto i2 = Interval(0, 1);
    EXPECT_TRUE((i1 == i2));
    EXPECT_FALSE((i1 != i2));
  }

  {
    const auto i1 = Interval(0, 1);
    const auto i2 = Interval();
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval();
    const auto i2 = Interval();
    EXPECT_TRUE((i1 == i2));
    EXPECT_FALSE((i1 != i2));
  }
}

TEST(Maeve_Geometry_Interval, testContains) {
  {
    const auto i = Interval();
    EXPECT_FALSE(Interval::contains(i, 0.0));
  }

  {
    const auto i = Interval(-1.0, 1.0);
    EXPECT_TRUE(Interval::contains(i, 0.0));
    EXPECT_FALSE(Interval::contains(i, 2.0));
  }
}

TEST(Maeve_Geometry_Interval, testEmpty) {
  {
    const auto i1 = Interval();
    const auto i2 = Interval();
    const auto i3 = Interval::intersect(i1, i2);
    EXPECT_TRUE(Interval::empty(i3));
    const auto i4 = Interval::convexHull(i1, i2);
    EXPECT_TRUE(Interval::empty(i4));
  }

  {
    const auto i1 = Interval();
    const auto i2 = Interval(0.0, 1.0);
    const auto i3 = Interval::intersect(i1, i2);
    EXPECT_TRUE(Interval::empty(i3));
    const auto i4 = Interval::convexHull(i1, i2);
    EXPECT_FALSE(Interval::empty(i4));
    EXPECT_EQ(i4, i2);
  }
}

TEST(Maeve_Geometry_Interval, testZeroLength) {
  {
    const auto i = Interval(0, 1);
    EXPECT_FALSE(Interval::zeroLength(i));
  }

  {
    const auto i = Interval();
    EXPECT_FALSE(Interval::zeroLength(i));
  }

  {
    const auto i = Interval(2, 2);
    EXPECT_TRUE(Interval::zeroLength(i));
  }
}

TEST(Maeve_Geometry_Interval, testConvexHull) {
  {
    const auto i1 = Interval(-1.0, 1.0);
    const auto i2 = Interval(-2.0, 2.0);
    const auto i = Interval::convexHull(i1, i2);
    EXPECT_EQ(Interval::min(i), -2.0);
    EXPECT_EQ(Interval::max(i), 2.0);
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_NEAR(Interval::length(i), 4.0, epsilon);
  }

  {
    const auto i1 = Interval(-1.0, 1.0);
    const auto i2 = Interval(-2.0, 0.0);
    const auto i = Interval::convexHull(i1, i2);
    EXPECT_EQ(Interval::min(i), -2.0);
    EXPECT_EQ(Interval::max(i), 1.0);
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_NEAR(Interval::length(i), 3.0, epsilon);
  }

  {
    const auto i1 = Interval(5.0, 10.0);
    const auto i2 = Interval(-2.0, 0.0);
    const auto i = Interval::convexHull(i1, i2);
    EXPECT_EQ(Interval::min(i), -2.0);
    EXPECT_EQ(Interval::max(i), 10.0);
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_NEAR(Interval::length(i), 12.0, epsilon);
  }
}

TEST(Maeve_Geometry_Interval, testIntersection) {
  {
    const auto i1 = Interval(-1.0, 1.0);
    const auto i2 = Interval(-0.5, 1.5);
    const auto i = Interval::intersect(i1, i2);
    EXPECT_EQ(Interval::min(i), -0.5);
    EXPECT_EQ(Interval::max(i), 1.0);
    EXPECT_NEAR(Interval::length(i), 1.5, epsilon);
  }

  {
    const auto i1 = Interval(-2.0, -1.0);
    const auto i2 = Interval(1.0, 2.0);
    const auto i = Interval::intersect(i1, i2);
    EXPECT_TRUE(Interval::empty(i));
    EXPECT_TRUE(std::isnan(Interval::min(i)));
    EXPECT_TRUE(std::isnan(Interval::max(i)));
    EXPECT_TRUE(std::isnan(Interval::length(i)));
  }
}

TEST(Maeve_Geometry_Interval, testConstruction) {
  {
    const auto i = Interval();
    EXPECT_TRUE(std::isnan(Interval::min(i)));
    EXPECT_TRUE(std::isnan(Interval::max(i)));
    EXPECT_TRUE(std::isnan(Interval::length(i)));
  }

  {
    const auto i = Interval(-1.0, 1.0);
    EXPECT_EQ(Interval::min(i), -1.0);
    EXPECT_EQ(Interval::max(i), 1.0);
    EXPECT_NEAR(Interval::length(i), 2.0, epsilon);
  }

  {
    const auto i = Interval(0.0, 0.0);
    EXPECT_EQ(Interval::min(i), 0.0);
    EXPECT_EQ(Interval::max(i), 0.0);
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_EQ(Interval::length(i), 0.0);
  }

  {
    EXPECT_THROW({ Interval(1.0, -1.0); }, std::runtime_error);
  }
}

TEST(Maeve_Geometry_Interval, addition) {
  const auto i = Interval(-1.0, 1.0);
  const auto j = Interval(-2.0, 2.0);

  EXPECT_EQ((i + j), Interval(-3.0, 3.0));
  EXPECT_EQ((i + j), (j + i));
  EXPECT_EQ((i - 1.0), Interval(-2.0, 0.0));
  EXPECT_EQ((i + 1.0), Interval(0.0, 2.0));
}

}  // namespace maeve_automation_core
