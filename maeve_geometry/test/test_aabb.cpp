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

#include "maeve_automation_core/maeve_geometry/aabb.h"

namespace maeve_automation_core {
namespace {
const auto epsilon = 0.00001;
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

TEST(Maeve_Geometry_AABB, testOperations) {
  {
    const auto a = AABB<1>();
    const auto b = AABB<1>();
    const auto c = AABB<1>::intersection(a, b);
    EXPECT_TRUE(AABB<1>::valid(c));
    EXPECT_TRUE(AABB<1>::empty(c));
    EXPECT_TRUE(std::isnan(AABB<1>::volume(c)));
  }

  {
    const auto a = AABB<2>({Interval(0, 1), Interval(0, 1)});
    const auto b = AABB<2>({Interval(0.5, 1.5), Interval(0.25, 0.75)});
    const auto c = AABB<2>::intersection(a, b);
    EXPECT_TRUE(AABB<2>::valid(c));
    EXPECT_FALSE(AABB<2>::empty(c));
    EXPECT_NEAR(AABB<2>::min(c, 0), 0.5, epsilon);
    EXPECT_NEAR(AABB<2>::max(c, 0), 1.0, epsilon);
    EXPECT_NEAR(AABB<2>::min(c, 1), 0.25, epsilon);
    EXPECT_NEAR(AABB<2>::max(c, 1), 0.75, epsilon);
    EXPECT_NEAR(AABB<2>::volume(c), 0.25, epsilon);
  }

  {
    const auto a = AABB<2>({Interval(0, 1), Interval(0, 1)});
    const auto b = AABB<2>({Interval(0.5, 1.5), Interval(0.25, 0.75)});
    const auto c = AABB<2>::convexHull(a, b);
    EXPECT_TRUE(AABB<2>::valid(c));
    EXPECT_FALSE(AABB<2>::empty(c));
    EXPECT_NEAR(AABB<2>::min(c, 0), 0.0, epsilon);
    EXPECT_NEAR(AABB<2>::max(c, 0), 1.5, epsilon);
    EXPECT_NEAR(AABB<2>::min(c, 1), 0.0, epsilon);
    EXPECT_NEAR(AABB<2>::max(c, 1), 1.0, epsilon);
    EXPECT_NEAR(AABB<2>::volume(c), 1.5, epsilon);
  }
}

TEST(Maeve_Geometry_AABB, testProperties) {
  {
    const auto a = AABB<1>();
    EXPECT_TRUE(AABB<1>::valid(a));
    EXPECT_TRUE(AABB<1>::empty(a));
    EXPECT_TRUE(std::isnan(AABB<1>::volume(a)));
  }

  {
    const auto a = AABB<3>({Interval(0, 1), Interval(1, 2), Interval(2, 3)});
    EXPECT_TRUE(AABB<3>::valid(a));
    EXPECT_FALSE(AABB<3>::empty(a));
    EXPECT_NEAR(AABB<3>::volume(a), 1.0, epsilon);
  }

  {
    const auto a = AABB<3>({Interval(0, 2), Interval(1, 4), Interval(2, 6)});
    EXPECT_TRUE(AABB<3>::valid(a));
    EXPECT_FALSE(AABB<3>::empty(a));
    EXPECT_NEAR(AABB<3>::volume(a), 24.0, epsilon);
  }

  {
    const auto a = AABB<2>({Interval(0, 1), Interval(0, 1)});
    EXPECT_TRUE(AABB<2>::valid(a));
    EXPECT_FALSE(AABB<2>::empty(a));
    EXPECT_TRUE(AABB<2>::contains(a, Eigen::Vector2d(0.5, 0.75)));
    EXPECT_FALSE(AABB<2>::contains(a, Eigen::Vector2d(1.5, 0.75)));
  }
}

TEST(Maeve_Geometry_AABB, testCompileTimeChecks) {
  // None of these tests should compile.
  {
      // const auto a = AABB<1>();
  }

  {
      // const auto a = AABB<1>({{1, 2}});
  }

  {
      // const auto a = AABB<0>();
  }

  {
    // const auto a = AABB<-2>();
  }
}

TEST(Maeve_Geometry_AABB, testConstruction) {
  {
    const auto a = AABB<1>();
    EXPECT_TRUE(AABB<1>::valid(a));
  }

  {
    const auto a = AABB<2>({Interval(1, 2), Interval(1.5, 2.5)});
    EXPECT_TRUE(AABB<2>::valid(a));
  }
}
}  // namespace maeve_automation_core
