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

#include "maeve_automation_core/maeve_geometry/interval.h"

namespace maeve_automation_core {
namespace {
const auto epsilon = 0.00001;
}  // namespace

TEST(Maeve_Geometry_Interval, testComparisons) {
  {
    const auto i1 = Interval(0.25, 1);
    const auto i2 = Interval(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
    EXPECT_TRUE((i1 > i2));
    EXPECT_TRUE((i1 >= i2));
    EXPECT_FALSE((i1 < i2));
    EXPECT_FALSE((i1 <= i2));
  }

  {
    const auto i1 = Interval(-0.25, 0.5);
    const auto i2 = Interval(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
    EXPECT_FALSE((i1 > i2));
    EXPECT_FALSE((i1 >= i2));
    EXPECT_TRUE((i1 < i2));
    EXPECT_TRUE((i1 <= i2));
  }

  {
    const auto i1 = Interval(0, 0.5);
    const auto i2 = Interval(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
    EXPECT_FALSE((i1 > i2));
    EXPECT_FALSE((i1 >= i2));
    EXPECT_TRUE((i1 < i2));
    EXPECT_TRUE((i1 <= i2));
  }

  {
    const auto i1 = Interval(1, 0);
    const auto i2 = Interval(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_FALSE((i1 != i2));
    EXPECT_FALSE((i1 > i2));
    EXPECT_FALSE((i1 >= i2));
    EXPECT_FALSE((i1 < i2));
    EXPECT_FALSE((i1 <= i2));
  }

  {
    const auto i1 = Interval(0, 1);
    const auto i2 = Interval(0, 1);
    EXPECT_TRUE((i1 == i2));
    EXPECT_FALSE((i1 != i2));
    EXPECT_FALSE((i1 > i2));
    EXPECT_TRUE((i1 >= i2));
    EXPECT_FALSE((i1 < i2));
    EXPECT_TRUE((i1 <= i2));
  }

  {
    const auto i1 = Interval(1, 0);
    const auto i2 = Interval(1, 0);
    EXPECT_FALSE((i1 == i2));
    EXPECT_FALSE((i1 != i2));
    EXPECT_FALSE((i1 > i2));
    EXPECT_FALSE((i1 >= i2));
    EXPECT_FALSE((i1 < i2));
    EXPECT_FALSE((i1 <= i2));
  }

  {
    const auto i1 = Interval(1, 0);
    const auto i2 = Interval();
    EXPECT_FALSE((i1 == i2));
    EXPECT_FALSE((i1 != i2));
    EXPECT_FALSE((i1 > i2));
    EXPECT_FALSE((i1 >= i2));
    EXPECT_FALSE((i1 < i2));
    EXPECT_FALSE((i1 <= i2));
  }

  {
    const auto i1 = Interval(0, 1);
    const auto i2 = Interval();
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
    EXPECT_FALSE((i1 > i2));
    EXPECT_FALSE((i1 >= i2));
    EXPECT_FALSE((i1 < i2));
    EXPECT_FALSE((i1 <= i2));
  }

  {
    const auto i1 = Interval();
    const auto i2 = Interval();
    EXPECT_TRUE((i1 == i2));
    EXPECT_FALSE((i1 != i2));
    EXPECT_FALSE((i1 > i2));
    EXPECT_FALSE((i1 >= i2));
    EXPECT_FALSE((i1 < i2));
    EXPECT_FALSE((i1 <= i2));
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

  {
    const auto i = Interval(1.0, -1.0);
    EXPECT_FALSE(Interval::contains(i, 0.0));
    EXPECT_FALSE(Interval::contains(i, 2.0));
  }
}

TEST(Maeve_Geometry_Interval, testEmpty) {
  {
    const auto i1 = Interval();
    const auto i2 = Interval();
    const auto i3 = Interval::intersection(i1, i2);
    EXPECT_TRUE(Interval::valid(i3));
    EXPECT_TRUE(Interval::empty(i3));
    const auto i4 = Interval::convexHull(i1, i2);
    EXPECT_TRUE(Interval::valid(i4));
    EXPECT_TRUE(Interval::empty(i4));
  }

  {
    const auto i1 = Interval();
    const auto i2 = Interval(0.0, 1.0);
    const auto i3 = Interval::intersection(i1, i2);
    EXPECT_TRUE(Interval::valid(i3));
    EXPECT_TRUE(Interval::empty(i3));
    const auto i4 = Interval::convexHull(i1, i2);
    EXPECT_FALSE(Interval::valid(i4));
    EXPECT_FALSE(Interval::empty(i4));
  }
}

TEST(Maeve_Geometry_Interval, testConvexHull) {
  {
    const auto i1 = Interval(-1.0, 1.0);
    const auto i2 = Interval(-2.0, 2.0);
    const auto i = Interval::convexHull(i1, i2);
    EXPECT_TRUE(Interval::valid(i));
    EXPECT_EQ(Interval::min(i), -2.0);
    EXPECT_EQ(Interval::max(i), 2.0);
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_NEAR(Interval::length(i), 4.0, epsilon);
  }

  {
    const auto i1 = Interval(-1.0, 1.0);
    const auto i2 = Interval(-2.0, 0.0);
    const auto i = Interval::convexHull(i1, i2);
    EXPECT_TRUE(Interval::valid(i));
    EXPECT_EQ(Interval::min(i), -2.0);
    EXPECT_EQ(Interval::max(i), 1.0);
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_NEAR(Interval::length(i), 3.0, epsilon);
  }

  {
    const auto i1 = Interval(5.0, 10.0);
    const auto i2 = Interval(-2.0, 0.0);
    const auto i = Interval::convexHull(i1, i2);
    EXPECT_TRUE(Interval::valid(i));
    EXPECT_EQ(Interval::min(i), -2.0);
    EXPECT_EQ(Interval::max(i), 10.0);
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_NEAR(Interval::length(i), 12.0, epsilon);
  }

  {
    const auto i1 = Interval(10.0, 5.0);
    const auto i2 = Interval(-2.0, 0.0);
    const auto i = Interval::convexHull(i1, i2);
    EXPECT_FALSE(Interval::valid(i));
    EXPECT_TRUE(std::isnan(Interval::min(i)));
    EXPECT_TRUE(std::isnan(Interval::max(i)));
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_TRUE(std::isnan(Interval::length(i)));
  }
}

TEST(Maeve_Geometry_Interval, testIntersection) {
  {
    const auto i1 = Interval(-1.0, 1.0);
    const auto i2 = Interval(-0.5, 1.5);
    const auto i = Interval::intersection(i1, i2);
    EXPECT_TRUE(Interval::valid(i));
    EXPECT_EQ(Interval::min(i), -0.5);
    EXPECT_EQ(Interval::max(i), 1.0);
    EXPECT_NEAR(Interval::length(i), 1.5, epsilon);
  }

  {
    const auto i1 = Interval(1.0, -1.0);
    const auto i2 = Interval(-0.5, 1.5);
    const auto i = Interval::intersection(i1, i2);
    EXPECT_FALSE(Interval::valid(i));
    EXPECT_TRUE(std::isnan(Interval::min(i)));
    EXPECT_TRUE(std::isnan(Interval::max(i)));
    EXPECT_TRUE(std::isnan(Interval::length(i)));
  }

  {
    const auto i1 = Interval(1.0, -1.0);
    const auto i2 = Interval(0.5, -1.5);
    const auto i = Interval::intersection(i1, i2);
    EXPECT_FALSE(Interval::valid(i));
    EXPECT_TRUE(std::isnan(Interval::min(i)));
    EXPECT_TRUE(std::isnan(Interval::max(i)));
    EXPECT_TRUE(std::isnan(Interval::length(i)));
  }

  {
    const auto i1 = Interval(1.0, -1.0);
    const auto i2 = Interval(0.5, -0.5);
    const auto i = Interval::intersection(i1, i2);
    EXPECT_FALSE(Interval::valid(i));
    EXPECT_TRUE(std::isnan(Interval::min(i)));
    EXPECT_TRUE(std::isnan(Interval::max(i)));
  }

  {
    const auto i1 = Interval(-2.0, -1.0);
    const auto i2 = Interval(1.0, 2.0);
    const auto i = Interval::intersection(i1, i2);
    EXPECT_TRUE(Interval::valid(i));
    EXPECT_TRUE(Interval::empty(i));
    EXPECT_TRUE(std::isnan(Interval::min(i)));
    EXPECT_TRUE(std::isnan(Interval::max(i)));
    EXPECT_TRUE(std::isnan(Interval::length(i)));
  }
}

TEST(Maeve_Geometry_Interval, testConstruction) {
  {
    const auto i = Interval();
    EXPECT_TRUE(Interval::valid(i));
    EXPECT_TRUE(std::isnan(Interval::min(i)));
    EXPECT_TRUE(std::isnan(Interval::max(i)));
    EXPECT_TRUE(std::isnan(Interval::length(i)));
  }

  {
    const auto i = Interval(-1.0, 1.0);
    EXPECT_TRUE(Interval::valid(i));
    EXPECT_EQ(Interval::min(i), -1.0);
    EXPECT_EQ(Interval::max(i), 1.0);
    EXPECT_NEAR(Interval::length(i), 2.0, epsilon);
  }

  {
    const auto i = Interval(0.0, 0.0);
    EXPECT_TRUE(Interval::valid(i));
    EXPECT_EQ(Interval::min(i), 0.0);
    EXPECT_EQ(Interval::max(i), 0.0);
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_EQ(Interval::length(i), 0.0);
  }

  {
    const auto i = Interval(1.0, -1.0);
    EXPECT_FALSE(Interval::valid(i));
    EXPECT_TRUE(std::isnan(Interval::min(i)));
    EXPECT_TRUE(std::isnan(Interval::max(i)));
    EXPECT_FALSE(Interval::empty(i));
    EXPECT_TRUE(std::isnan(Interval::length(i)));
  }
}

}  // namespace maeve_automation_core
