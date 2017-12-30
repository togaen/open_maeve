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

#include "maeve_automation_core/maeve_geometry/disjoint_interval.h"

namespace maeve_automation_core {
namespace {
const auto epsilon = 0.00001;
}  // namespace

TEST(Maeve_Geometry_Disjoint_Interval, testInsert) {
  {
    auto di = DisjointInterval();
    {
      const auto i = Interval(0, 1);
      const auto p = DisjointInterval::insert(di, i);
      auto it_insert = DisjointInterval::begin(di);
      EXPECT_EQ(p, it_insert);
      EXPECT_EQ(DisjointInterval::size(di), 1);

      auto it_begin = DisjointInterval::begin(di);
      EXPECT_EQ(*it_begin, i);
    }
    {
      const auto i = Interval(3, 4);
      const auto p = DisjointInterval::insert(di, i);
      auto it_insert = DisjointInterval::begin(di);
      EXPECT_EQ(++it_insert, p);
      EXPECT_EQ(DisjointInterval::size(di), 2);

      auto it_begin = DisjointInterval::begin(di);
      EXPECT_EQ(*it_begin, Interval(0, 1));
      EXPECT_EQ(*(++it_begin), i);
    }
    {
      const auto i = Interval(0.5, 2);
      const auto p = DisjointInterval::insert(di, i);
      auto it_insert = DisjointInterval::begin(di);
      EXPECT_EQ(p, it_insert);
      EXPECT_EQ(DisjointInterval::size(di), 2);

      auto it_begin = DisjointInterval::begin(di);
      EXPECT_EQ(*it_begin, Interval(0, 2));
      EXPECT_EQ(*(++it_begin), Interval(3, 4));
    }
    {
      const auto i = Interval(4, 5);
      const auto p = DisjointInterval::insert(di, i);
      auto it_insert = DisjointInterval::begin(di);
      EXPECT_EQ(++it_insert, p);
      EXPECT_EQ(DisjointInterval::size(di), 2);

      auto it_begin = DisjointInterval::begin(di);
      EXPECT_EQ(*it_begin, Interval(0, 2));
      EXPECT_EQ(*(++it_begin), Interval(3, 5));
    }
    {
      const auto i = Interval(5.0 + epsilon, 6);
      const auto p = DisjointInterval::insert(di, i);
      auto it_insert = ++(++DisjointInterval::begin(di));
      EXPECT_EQ(it_insert, p);
      EXPECT_EQ(DisjointInterval::size(di), 3);

      auto it_begin = DisjointInterval::begin(di);
      EXPECT_EQ(*it_begin, Interval(0, 2));
      EXPECT_EQ(*(++it_begin), Interval(3, 4));
      EXPECT_EQ(*(++it_begin), i);
    }
    {
      const auto i = Interval(-10, 10);
      const auto p = DisjointInterval::insert(di, i);
      auto it_insert = DisjointInterval::begin(di);
      EXPECT_EQ(it_insert, p);
      EXPECT_EQ(DisjointInterval::size(di), 3);

      auto it_begin = DisjointInterval::begin(di);
      EXPECT_EQ(*it_begin, Interval(0, 2));
      EXPECT_EQ(*(++it_begin), Interval(3, 4));
      EXPECT_EQ(*(++it_begin), i);
    }
  }

  {
    auto di = DisjointInterval();
    {
      const auto i = Interval(0, 1);
      const auto p = DisjointInterval::insert(di, i);
      EXPECT_EQ(p, DisjointInterval::begin(di));
      EXPECT_EQ(DisjointInterval::size(di), 1);
    }
    {
      const auto i = Interval(3, 4);
      const auto p = DisjointInterval::insert(di, i);
      EXPECT_EQ(++DisjointInterval::begin(di), p);
      EXPECT_EQ(DisjointInterval::size(di), 2);
    }
    {
      const auto i = Interval(0.5, 2);
      const auto p = DisjointInterval::insert(di, i);
      EXPECT_EQ(p, DisjointInterval::begin(di));
      EXPECT_EQ(DisjointInterval::size(di), 2);
    }
  }

  {
    auto di = DisjointInterval();
    {
      const auto i = Interval(0, 1);
      const auto p = DisjointInterval::insert(di, i);
      EXPECT_EQ(p, DisjointInterval::begin(di));
      EXPECT_EQ(DisjointInterval::size(di), 1);
    }
    {
      const auto i = Interval(0.5, 2);
      const auto p = DisjointInterval::insert(di, i);
      EXPECT_EQ(p, DisjointInterval::begin(di));
      EXPECT_EQ(DisjointInterval::size(di), 1);
    }
    {
      const auto i = Interval(3, 4);
      const auto p = DisjointInterval::insert(di, i);
      EXPECT_EQ(++DisjointInterval::begin(di), p);
      EXPECT_EQ(DisjointInterval::size(di), 2);
    }
  }

  {
    auto di = DisjointInterval();
    const auto p = DisjointInterval::insert(di, Interval());
    EXPECT_EQ(DisjointInterval::end(di), p);
  }
}

}  // namespace maeve_automation_core
