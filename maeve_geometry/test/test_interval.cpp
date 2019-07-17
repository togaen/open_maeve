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
const auto epsilon = 1e-5;
}  // namespace

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, from_unordered_bounds) {
  ASSERT_NO_THROW(
      { const auto i = Interval_d::from_unordered_bounds(1.0, 0.0); });
  EXPECT_EQ(Interval_d(0.0, 1.0), Interval_d::from_unordered_bounds(1.0, 0.0));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, are_adjacent_ordered) {
  const auto i1 = Interval_d(-1.0, 1.0);
  const auto i2 = Interval_d(1.0, 2.0);
  const auto i_empty = Interval_d();
  const auto i4 = Interval_d(3.0, 4.0);
  const auto i_reals = Interval_d::max_representable_reals();
  const auto i5 = Interval_d(1.70711, 1.70711);

  EXPECT_TRUE(Interval_d::are_adjacent_ordered(i1, i2));
  EXPECT_FALSE(Interval_d::are_adjacent_ordered(i2, i1));

  EXPECT_FALSE(Interval_d::are_adjacent_ordered(i_empty, i_empty));

  EXPECT_FALSE(Interval_d::are_adjacent_ordered(i1, i_empty));
  EXPECT_FALSE(Interval_d::are_adjacent_ordered(i_empty, i1));

  EXPECT_FALSE(Interval_d::are_adjacent_ordered(i2, i4));
  EXPECT_FALSE(Interval_d::are_adjacent_ordered(i4, i2));

  EXPECT_FALSE(Interval_d::are_adjacent_ordered(i5, i_reals));
  EXPECT_FALSE(Interval_d::are_adjacent_ordered(i_reals, i5));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, approx_eq) {
  const auto i1 = Interval_d(-1.0, 1.0);
  const auto i2 = Interval_d(-1.0 + 0.5 * epsilon, 1.0 + 0.5 * epsilon);
  const auto i3 = (i1 + (2.0 * epsilon));
  const auto i_empty = Interval_d();

  EXPECT_TRUE(Interval_d::approx_eq(i1, i1, epsilon));
  EXPECT_TRUE(Interval_d::approx_eq(i1, i2, epsilon));
  EXPECT_TRUE(Interval_d::approx_eq(i2, i1, epsilon));
  EXPECT_FALSE(Interval_d::approx_eq(i1, i3, epsilon));
  EXPECT_FALSE(Interval_d::approx_eq(i3, i1, epsilon));
  EXPECT_FALSE(Interval_d::approx_eq(i1, i_empty, epsilon));
  EXPECT_FALSE(Interval_d::approx_eq(i_empty, i1, epsilon));
  EXPECT_TRUE(Interval_d::approx_eq(i_empty, i_empty, epsilon));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, origin_centered_symmetric) {
  {
    const auto length = 2.0;
    EXPECT_EQ(Interval_d::origin_centered_symmetric(length),
              Interval_d(-1.0, 1.0));
  }

  {
    const auto length = 1.0;
    EXPECT_EQ(Interval_d::origin_centered_symmetric(length),
              Interval_d(-0.5, 0.5));
  }

  {
    const auto length = NaN;
    EXPECT_TRUE(
        Interval_d::empty(Interval_d::origin_centered_symmetric(length)));
  }

  {
    const auto length = 0.0;
    EXPECT_EQ(Interval_d::origin_centered_symmetric(length),
              Interval_d(0.0, 0.0));
  }

  {
    EXPECT_THROW(
        {
          const auto length = -1.0;
          EXPECT_EQ(Interval_d::origin_centered_symmetric(length),
                    Interval_d(-0.5, 0.5));
        },
        std::runtime_error);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, nearest_value) {
  const auto i = Interval_d(-1.0, 1.0);
  EXPECT_EQ(Interval_d::nearest_value(i, 0.0), 0.0);
  EXPECT_EQ(Interval_d::nearest_value(i, 0.25), 0.25);
  EXPECT_EQ(Interval_d::nearest_value(i, -0.33), -0.33);
  EXPECT_EQ(Interval_d::nearest_value(i, -10.0), -1.0);
  EXPECT_EQ(Interval_d::nearest_value(i, 10.0), 1.0);
  EXPECT_TRUE(std::isnan(Interval_d::nearest_value(i, NaN)));
  EXPECT_EQ(Interval_d::nearest_value(i, Inf), 1.0);
  EXPECT_EQ(Interval_d::nearest_value(i, -Inf), -1.0);
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, center) {
  EXPECT_EQ(Interval_d::center(Interval_d(-1.0, 1.0)), 0.0);
  EXPECT_EQ(Interval_d::center(Interval_d(-3.0, -2.0)), -2.5);
  EXPECT_TRUE(std::isnan(Interval_d::center(Interval_d())));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, testIsSubsetEq) {
  EXPECT_TRUE(Interval<double>::is_subset_eq(Interval<double>(-0.5, 0.5),
                                             Interval<double>(-1.0, 1.0)));
  EXPECT_TRUE(Interval<double>::is_subset_eq(Interval<double>(3.2, 4.2),
                                             Interval<double>(3.2, 4.2)));
  EXPECT_FALSE(Interval<double>::is_subset_eq(Interval<double>(-3.0, -1.0),
                                              Interval<double>(1.0, 3.0)));
  EXPECT_FALSE(Interval<double>::is_subset_eq(Interval<double>(1.0, 3.0),
                                              Interval<double>(2.0, 4.0)));
  EXPECT_FALSE(
      Interval<double>::is_subset_eq(Interval<double>(), Interval<double>()));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, project_to_range) {
  const auto i1 = Interval_d(-3.0, 1.0);
  const auto i2 = Interval_d(5.0, 6.0);

  EXPECT_EQ(Interval_d::project_to_range(-1.0, i1, i2), 5.5);
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, testProjectToInterval) {
  const auto i = Interval<double>(-1.0, 1.0);
  {
    const auto val = 0.0;
    const auto p = Interval<double>::project_to_interval(i, val);
    EXPECT_EQ(p, val);
  }

  {
    const auto val = -3.4;
    const auto p = Interval<double>::project_to_interval(i, val);
    EXPECT_EQ(p, Interval<double>::min(i));
  }

  {
    const auto val = 2.7;
    const auto p = Interval<double>::project_to_interval(i, val);
    EXPECT_EQ(p, Interval<double>::max(i));
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, scale_to_interval) {
  {
    const auto i = Interval<double>();
    EXPECT_TRUE(std::isnan(Interval<double>::scale_to_interval(i, 0.0)));
    EXPECT_TRUE(std::isnan(Interval<double>::scale_to_interval(i, NaN)));
  }

  {
    const auto i = Interval<double>(0.0, 1.0);
    EXPECT_FALSE(std::isnan(Interval<double>::scale_to_interval(i, 0.0)));
    EXPECT_TRUE(std::isnan(Interval<double>::scale_to_interval(i, NaN)));
  }

  {
    const auto i = Interval<double>(0.0, 3.0);
    EXPECT_EQ(Interval<double>::scale_to_interval(i, 0.0), 0.0);
    EXPECT_EQ(Interval<double>::scale_to_interval(i, 1.0), (1.0 / 3.0));
    EXPECT_EQ(Interval<double>::scale_to_interval(i, 1.5), 0.5);
    EXPECT_EQ(Interval<double>::scale_to_interval(i, -1.0), (-1.0 / 3.0));
    EXPECT_EQ(Interval<double>::scale_to_interval(i, 4.0), (4.0 / 3.0));
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, testFactories) {
  EXPECT_EQ(Interval<double>::affinely_extended_reals(),
            Interval<double>(-Inf, Inf));
  EXPECT_EQ(Interval<double>::max_representable_reals(),
            Interval<double>(Min, Max));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, testMerge) {
  {
    const auto i1 = Interval<double>();
    const auto i2 = Interval<double>(0, 1);
    const auto i_opt = Interval<double>::merge(i1, i2);
    ASSERT_FALSE(!i_opt);
    const auto i = *i_opt;
    EXPECT_FALSE(Interval<double>::empty(i));
    EXPECT_EQ(Interval<double>::min(i), 0.0);
    EXPECT_EQ(Interval<double>::max(i), 1.0);
  }

  {
    const auto i_opt =
        Interval<double>::merge(Interval<double>(), Interval<double>());
    ASSERT_FALSE(!i_opt);
    const auto i = *i_opt;
    EXPECT_TRUE(Interval<double>::empty(i));
  }

  {
    const auto i1 = Interval<double>(2, 3);
    const auto i2 = Interval<double>(0, 1);
    const auto i = Interval<double>::merge(i1, i2);
    EXPECT_TRUE(!i);
  }

  {
    const auto i1 = Interval<double>(0.25, 0.75);
    const auto i2 = Interval<double>(0, 1);
    const auto i_opt = Interval<double>::merge(i1, i2);
    ASSERT_FALSE(!i_opt);
    const auto i = *i_opt;
    EXPECT_FALSE(Interval<double>::empty(i));
    EXPECT_EQ(Interval<double>::min(i), 0.0);
    EXPECT_EQ(Interval<double>::max(i), 1.0);
  }

  {
    const auto i1 = Interval<double>(0.25, 1);
    const auto i2 = Interval<double>(0, 1);
    const auto i_opt = Interval<double>::merge(i1, i2);
    ASSERT_FALSE(!i_opt);
    const auto i = *i_opt;
    EXPECT_FALSE(Interval<double>::empty(i));
    EXPECT_EQ(Interval<double>::min(i), 0.0);
    EXPECT_EQ(Interval<double>::max(i), 1.0);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, grow) {
  const auto i = Interval_d(-1.0, 1.0);
  EXPECT_THROW({ const auto i2 = Interval_d::grow(i, -2.0); },
               std::runtime_error);

  EXPECT_EQ(Interval_d(-2.0, 2.0), Interval_d::grow(i, 1.0));
  EXPECT_EQ(Interval_d(0.0, 0.0), Interval_d::grow(i, -1.0));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, testComparisons) {
  {
    const auto i1 = Interval<double>(0.25, 1);
    const auto i2 = Interval<double>(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval<double>(-0.25, 0.5);
    const auto i2 = Interval<double>(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval<double>(0, 0.5);
    const auto i2 = Interval<double>(0, 1);
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval<double>(0, 1);
    const auto i2 = Interval<double>(0, 1);
    EXPECT_TRUE((i1 == i2));
    EXPECT_FALSE((i1 != i2));
  }

  {
    const auto i1 = Interval<double>(0, 1);
    const auto i2 = Interval<double>();
    EXPECT_FALSE((i1 == i2));
    EXPECT_TRUE((i1 != i2));
  }

  {
    const auto i1 = Interval<double>();
    const auto i2 = Interval<double>();
    EXPECT_TRUE((i1 == i2));
    EXPECT_FALSE((i1 != i2));
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, testContains) {
  {
    const auto i = Interval<double>();
    EXPECT_FALSE(Interval<double>::contains(i, 0.0));
  }

  {
    const auto i = Interval<double>(-1.0, 1.0);
    EXPECT_TRUE(Interval<double>::contains(i, 0.0));
    EXPECT_FALSE(Interval<double>::contains(i, 2.0));
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, testEmpty) {
  {
    const auto i1 = Interval<double>();
    const auto i2 = Interval<double>();
    const auto i3 = Interval<double>::intersect(i1, i2);
    EXPECT_TRUE(Interval<double>::empty(i3));
    const auto i4 = Interval<double>::convex_hull(i1, i2);
    EXPECT_TRUE(Interval<double>::empty(i4));
  }

  {
    const auto i1 = Interval<double>();
    const auto i2 = Interval<double>(0.0, 1.0);
    const auto i3 = Interval<double>::intersect(i1, i2);
    EXPECT_TRUE(Interval<double>::empty(i3));
    const auto i4 = Interval<double>::convex_hull(i1, i2);
    EXPECT_FALSE(Interval<double>::empty(i4));
    EXPECT_EQ(i4, i2);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, testZeroLength) {
  {
    const auto i = Interval<double>(0, 1);
    EXPECT_FALSE(Interval<double>::zero_length(i));
  }

  {
    const auto i = Interval<double>();
    EXPECT_FALSE(Interval<double>::zero_length(i));
  }

  {
    const auto i = Interval<double>(2, 2);
    EXPECT_TRUE(Interval<double>::zero_length(i));
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, testConvexHull) {
  {
    const auto i1 = Interval<double>(-1.0, 1.0);
    const auto i2 = Interval<double>(-2.0, 2.0);
    const auto i = Interval<double>::convex_hull(i1, i2);
    EXPECT_EQ(Interval<double>::min(i), -2.0);
    EXPECT_EQ(Interval<double>::max(i), 2.0);
    EXPECT_FALSE(Interval<double>::empty(i));
    EXPECT_NEAR(Interval<double>::length(i), 4.0, epsilon);
  }

  {
    const auto i1 = Interval<double>(-1.0, 1.0);
    const auto i2 = Interval<double>(-2.0, 0.0);
    const auto i = Interval<double>::convex_hull(i1, i2);
    EXPECT_EQ(Interval<double>::min(i), -2.0);
    EXPECT_EQ(Interval<double>::max(i), 1.0);
    EXPECT_FALSE(Interval<double>::empty(i));
    EXPECT_NEAR(Interval<double>::length(i), 3.0, epsilon);
  }

  {
    const auto i1 = Interval<double>(5.0, 10.0);
    const auto i2 = Interval<double>(-2.0, 0.0);
    const auto i = Interval<double>::convex_hull(i1, i2);
    EXPECT_EQ(Interval<double>::min(i), -2.0);
    EXPECT_EQ(Interval<double>::max(i), 10.0);
    EXPECT_FALSE(Interval<double>::empty(i));
    EXPECT_NEAR(Interval<double>::length(i), 12.0, epsilon);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, testIntersection) {
  {
    const auto i1 = Interval<double>(-1.0, 1.0);
    const auto i2 = Interval<double>(-0.5, 1.5);
    const auto i = Interval<double>::intersect(i1, i2);
    EXPECT_EQ(Interval<double>::min(i), -0.5);
    EXPECT_EQ(Interval<double>::max(i), 1.0);
    EXPECT_NEAR(Interval<double>::length(i), 1.5, epsilon);
  }

  {
    const auto i1 = Interval<double>(-2.0, -1.0);
    const auto i2 = Interval<double>(1.0, 2.0);
    const auto i = Interval<double>::intersect(i1, i2);
    EXPECT_TRUE(Interval<double>::empty(i));
    EXPECT_TRUE(std::isnan(Interval<double>::min(i)));
    EXPECT_TRUE(std::isnan(Interval<double>::max(i)));
    EXPECT_TRUE(std::isnan(Interval<double>::length(i)));
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, testConstruction) {
  {
    const auto i = Interval<double>();
    EXPECT_TRUE(std::isnan(Interval<double>::min(i)));
    EXPECT_TRUE(std::isnan(Interval<double>::max(i)));
    EXPECT_TRUE(std::isnan(Interval<double>::length(i)));
  }

  {
    const auto i = Interval<double>(-1.0, 1.0);
    EXPECT_EQ(Interval<double>::min(i), -1.0);
    EXPECT_EQ(Interval<double>::max(i), 1.0);
    EXPECT_NEAR(Interval<double>::length(i), 2.0, epsilon);
  }

  {
    const auto i = Interval<double>(0.0, 0.0);
    EXPECT_EQ(Interval<double>::min(i), 0.0);
    EXPECT_EQ(Interval<double>::max(i), 0.0);
    EXPECT_FALSE(Interval<double>::empty(i));
    EXPECT_EQ(Interval<double>::length(i), 0.0);
  }

  {
    EXPECT_THROW({ Interval<double>(1.0, -1.0); }, std::runtime_error);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Interval, addition) {
  const auto i = Interval<double>(-1.0, 1.0);
  const auto j = Interval<double>(-2.0, 2.0);
  const auto k = Interval<double>();

  EXPECT_EQ((i + j), Interval<double>(-3.0, 3.0));
  EXPECT_EQ((i + j), (j + i));
  EXPECT_EQ((i - 1.0), Interval<double>(-2.0, 0.0));
  EXPECT_EQ((i + 1.0), Interval<double>(0.0, 2.0));
  EXPECT_EQ((i + k), i);
  EXPECT_EQ((k + i), i);
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
