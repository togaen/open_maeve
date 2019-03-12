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
DisjointInterval<double> makeTestDI() {
  return DisjointInterval<double>(
      {Interval<double>(0.0, 1.0), Interval<double>(2.0, 3.0),
       Interval<double>(4.0, 5.0), Interval<double>(6.0, 7.0),
       Interval<double>(8.0, 9.0)});
}
}  // namespace

TEST(Maeve_Geometry_Disjoint_Interval, testIntersection) {
  {
    const auto di1 = makeTestDI();
    const auto di2 = DisjointInterval<double>({Interval<double>(0.5, 0.75),
                                               Interval<double>(2.5, 2.8),
                                               Interval<double>(6.3, 6.6)});
    const auto di = DisjointInterval<double>::intersect(di1, di2);
    EXPECT_EQ(di, di2);
  }

  {
    const auto di1 = makeTestDI();
    const auto di2 = DisjointInterval<double>();
    const auto di = DisjointInterval<double>::intersect(di1, di2);
    EXPECT_EQ(di, di2);
  }

  {
    const auto di1 = makeTestDI();
    const auto di2 = DisjointInterval<double>(
        {Interval<double>(-88, -78), Interval<double>(45, 46)});
    const auto di =
        DisjointInterval<double>::intersect(di1, DisjointInterval<double>());
  }

  {
    const auto di1 = makeTestDI();
    const auto di2 = DisjointInterval<double>({Interval<double>(0.0, 10.0)});
    const auto di = DisjointInterval<double>::intersect(di1, di2);
    EXPECT_EQ(di, di1);
  }

  {
    const auto di1 = makeTestDI();
    const auto di2 = DisjointInterval<double>({Interval<double>(0.5, 2.5)});
    const auto di = DisjointInterval<double>::intersect(di1, di2);
    EXPECT_EQ(di, DisjointInterval<double>({Interval<double>(0.5, 1.0),
                                            Interval<double>(2.0, 2.5)}));
  }
}

TEST(Maeve_Geometry_Disjoint_Interval, testContains) {
  const auto di = makeTestDI();
  EXPECT_TRUE(DisjointInterval<double>::contains(di, 0.5));
  EXPECT_TRUE(DisjointInterval<double>::contains(di, 6.01));
  EXPECT_FALSE(DisjointInterval<double>::contains(di, 3.01));
  EXPECT_FALSE(DisjointInterval<double>::contains(di, 11.0));
}

TEST(Maeve_Geometry_Disjoint_Interval, testComparison) {
  {
    EXPECT_EQ(makeTestDI(), makeTestDI());
    EXPECT_FALSE((makeTestDI() != makeTestDI()));
  }

  {
    auto di1 = makeTestDI();
    auto di2 = makeTestDI();
    DisjointInterval<double>::insert(di2, Interval<double>(0.5, 1.5));
    EXPECT_EQ(DisjointInterval<double>::size(di2),
              DisjointInterval<double>::size(di1));
    EXPECT_NE(di1, di2);
    EXPECT_FALSE((di1 == di2));
  }

  {
    auto di1 = makeTestDI();
    auto di2 = makeTestDI();
    DisjointInterval<double>::insert(di2, Interval<double>(0.5, 2.5));
    EXPECT_EQ(DisjointInterval<double>::size(di2), 4);
    EXPECT_EQ(DisjointInterval<double>::size(di1), 5);
    EXPECT_NE(di1, di2);
    EXPECT_FALSE((di1 == di2));
  }
}

TEST(Maeve_Geometry_Disjoint_Interval, testInsert) {
  {
    auto di = makeTestDI();
    const auto p =
        DisjointInterval<double>::insert(di, Interval<double>(0.5, 2.5));
    EXPECT_EQ(di,
              DisjointInterval<double>(
                  {Interval<double>(0.0, 3.0), Interval<double>(4.0, 5.0),
                   Interval<double>(6.0, 7.0), Interval<double>(8.0, 9.0)}));
  }

  {
    auto di = makeTestDI();
    const auto p =
        DisjointInterval<double>::insert(di, Interval<double>(-1.0, -0.5));
    EXPECT_EQ(di,
              DisjointInterval<double>(
                  {Interval<double>(-1.0, -0.5), Interval<double>(0.0, 1.0),
                   Interval<double>(2.0, 3.0), Interval<double>(4.0, 5.0),
                   Interval<double>(6.0, 7.0), Interval<double>(8.0, 9.0)}));
  }

  {
    auto di = makeTestDI();
    const auto p =
        DisjointInterval<double>::insert(di, Interval<double>(10.0, 11.0));
    EXPECT_EQ(di,
              DisjointInterval<double>(
                  {Interval<double>(0.0, 1.0), Interval<double>(2.0, 3.0),
                   Interval<double>(4.0, 5.0), Interval<double>(6.0, 7.0),
                   Interval<double>(8.0, 9.0), Interval<double>(10.0, 11.0)}));
  }

  {
    auto di = makeTestDI();
    const auto p =
        DisjointInterval<double>::insert(di, Interval<double>(8.5, 9.5));
    EXPECT_EQ(di, DisjointInterval<double>(
                      {Interval<double>(0.0, 1.0), Interval<double>(2.0, 3.0),
                       Interval<double>(4.0, 5.0), Interval<double>(6.0, 7.0),
                       Interval<double>(8.0, 9.5)}));
  }

  {
    auto di = makeTestDI();
    const auto p =
        DisjointInterval<double>::insert(di, Interval<double>(0.5, 1.5));
    EXPECT_EQ(di, DisjointInterval<double>(
                      {Interval<double>(0.0, 1.5), Interval<double>(2.0, 3.0),
                       Interval<double>(4.0, 5.0), Interval<double>(6.0, 7.0),
                       Interval<double>(8.0, 9.0)}));
  }

  {
    auto di = makeTestDI();
    const auto p =
        DisjointInterval<double>::insert(di, Interval<double>(1.5, 7.5));
    EXPECT_EQ(di, DisjointInterval<double>({Interval<double>(0.0, 1.0),
                                            Interval<double>(1.5, 7.5),
                                            Interval<double>(8.0, 9.0)}));
  }

  {
    auto di = DisjointInterval<double>();
    {
      const auto i = Interval<double>(0, 1);
      const auto p = DisjointInterval<double>::insert(di, i);
      auto it_insert = DisjointInterval<double>::begin(di);
      EXPECT_EQ(p, it_insert);
      EXPECT_EQ(DisjointInterval<double>::size(di), 1);

      auto it_begin = DisjointInterval<double>::begin(di);
      EXPECT_EQ(*it_begin, i);
    }
    {
      const auto i = Interval<double>(3, 4);
      const auto p = DisjointInterval<double>::insert(di, i);
      auto it_insert = DisjointInterval<double>::begin(di);
      EXPECT_EQ(++it_insert, p);
      EXPECT_EQ(DisjointInterval<double>::size(di), 2);

      auto it_begin = DisjointInterval<double>::begin(di);
      EXPECT_EQ(*it_begin, Interval<double>(0, 1));
      EXPECT_EQ(*(++it_begin), i);
    }
    {
      const auto i = Interval<double>(0.5, 2);
      const auto p = DisjointInterval<double>::insert(di, i);
      auto it_insert = DisjointInterval<double>::begin(di);
      EXPECT_EQ(p, it_insert);
      EXPECT_EQ(DisjointInterval<double>::size(di), 2);

      auto it_begin = DisjointInterval<double>::begin(di);
      EXPECT_EQ(*it_begin, Interval<double>(0, 2));
      EXPECT_EQ(*(++it_begin), Interval<double>(3, 4));
    }
    {
      const auto i = Interval<double>(4, 5);
      const auto p = DisjointInterval<double>::insert(di, i);
      auto it_insert = DisjointInterval<double>::begin(di);
      EXPECT_EQ(++it_insert, p);
      EXPECT_EQ(DisjointInterval<double>::size(di), 2);

      auto it_begin = DisjointInterval<double>::begin(di);
      EXPECT_EQ(*it_begin, Interval<double>(0, 2));
      EXPECT_EQ(*(++it_begin), Interval<double>(3, 5));
    }
    {
      const auto i = Interval<double>(5.0 + epsilon, 6);
      const auto p = DisjointInterval<double>::insert(di, i);
      auto it_insert = ++(++DisjointInterval<double>::begin(di));
      EXPECT_EQ(it_insert, p);
      EXPECT_EQ(DisjointInterval<double>::size(di), 3);

      auto it_begin = DisjointInterval<double>::begin(di);
      EXPECT_EQ(*it_begin, Interval<double>(0, 2));
      EXPECT_EQ(*(++it_begin), Interval<double>(3, 5))
          << "Disjoint Interval: " << di;
      EXPECT_EQ(*(++it_begin), i);
    }
    {
      const auto i = Interval<double>(-10, 10);
      const auto p = DisjointInterval<double>::insert(di, i);
      auto it_insert = DisjointInterval<double>::begin(di);
      EXPECT_EQ(it_insert, p);
      EXPECT_EQ(DisjointInterval<double>::size(di), 1);

      auto it_begin = DisjointInterval<double>::begin(di);
      EXPECT_EQ(*it_begin, Interval<double>(-10.0, 10.0));
    }
  }

  {
    auto di = DisjointInterval<double>();
    {
      const auto i = Interval<double>(0, 1);
      const auto p = DisjointInterval<double>::insert(di, i);
      EXPECT_EQ(p, DisjointInterval<double>::begin(di));
      EXPECT_EQ(DisjointInterval<double>::size(di), 1);
    }
    {
      const auto i = Interval<double>(3, 4);
      const auto p = DisjointInterval<double>::insert(di, i);
      EXPECT_EQ(++DisjointInterval<double>::begin(di), p);
      EXPECT_EQ(DisjointInterval<double>::size(di), 2);
    }
    {
      const auto i = Interval<double>(0.5, 2);
      const auto p = DisjointInterval<double>::insert(di, i);
      EXPECT_EQ(p, DisjointInterval<double>::begin(di));
      EXPECT_EQ(DisjointInterval<double>::size(di), 2);
    }
  }

  {
    auto di = DisjointInterval<double>();
    {
      const auto i = Interval<double>(0, 1);
      const auto p = DisjointInterval<double>::insert(di, i);
      EXPECT_EQ(p, DisjointInterval<double>::begin(di));
      EXPECT_EQ(DisjointInterval<double>::size(di), 1);
    }
    {
      const auto i = Interval<double>(0.5, 2);
      const auto p = DisjointInterval<double>::insert(di, i);
      EXPECT_EQ(p, DisjointInterval<double>::begin(di));
      EXPECT_EQ(DisjointInterval<double>::size(di), 1);
    }
    {
      const auto i = Interval<double>(3, 4);
      const auto p = DisjointInterval<double>::insert(di, i);
      EXPECT_EQ(++DisjointInterval<double>::begin(di), p);
      EXPECT_EQ(DisjointInterval<double>::size(di), 2);
    }
  }

  {
    auto di = DisjointInterval<double>();
    const auto p = DisjointInterval<double>::insert(di, Interval<double>());
    EXPECT_EQ(DisjointInterval<double>::end(di), p);
  }
}

}  // namespace maeve_automation_core
