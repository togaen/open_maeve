/*
 * Copyright 2019 Maeve Automation
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

#include "maeve_automation_core/maeve_geometry/bounds_ordered_interval.h"

namespace maeve_automation_core {

//------------------------------------------------------------------------------

TEST(BoundsOrderedInterval, ordering) {
  {
    const auto i1 = BoundsOrderedInterval(Interval(0.25, 1));
    const auto i2 = BoundsOrderedInterval(Interval(0, 1));
    EXPECT_TRUE((i1 > i2));
    EXPECT_TRUE((i1 >= i2));
    EXPECT_FALSE((i1 < i2));
    EXPECT_FALSE((i1 <= i2));
  }

  {
    const auto i1 = BoundsOrderedInterval(Interval(-0.25, 0.5));
    const auto i2 = BoundsOrderedInterval(Interval(0, 1));
    EXPECT_FALSE((i1 > i2));
    EXPECT_FALSE((i1 >= i2));
    EXPECT_TRUE((i1 < i2));
    EXPECT_TRUE((i1 <= i2));
  }

  {
    const auto i1 = BoundsOrderedInterval(Interval(0, 0.5));
    const auto i2 = BoundsOrderedInterval(Interval(0, 1));
    EXPECT_FALSE((i1 > i2));
    EXPECT_FALSE((i1 >= i2));
    EXPECT_TRUE((i1 < i2));
    EXPECT_TRUE((i1 <= i2));
  }

  {
    const auto i1 = BoundsOrderedInterval(Interval(0, 1));
    const auto i2 = BoundsOrderedInterval(Interval(0, 1));
    EXPECT_FALSE((i1 > i2));
    EXPECT_TRUE((i1 >= i2));
    EXPECT_FALSE((i1 < i2));
    EXPECT_TRUE((i1 <= i2));
  }

  {
    const auto i1 = BoundsOrderedInterval(Interval(0, 1));
    const auto i2 = BoundsOrderedInterval(Interval());
    EXPECT_FALSE((i1 > i2));
    EXPECT_FALSE((i1 >= i2));
    EXPECT_FALSE((i1 < i2));
    EXPECT_FALSE((i1 <= i2));
  }

  {
    const auto i1 = BoundsOrderedInterval(Interval());
    const auto i2 = BoundsOrderedInterval(Interval());
    EXPECT_FALSE((i1 > i2));
    EXPECT_FALSE((i1 >= i2));
    EXPECT_FALSE((i1 < i2));
    EXPECT_FALSE((i1 <= i2));
  }
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
