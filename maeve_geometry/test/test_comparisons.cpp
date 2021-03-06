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

#include "boost/optional.hpp"

#include "open_maeve/maeve_geometry/comparisons.h"

namespace {
const auto a = 1.317;
const auto b = 2.0;
const auto c = -5.2747;
const auto d = 0.0;
const auto e = -5.2747177;
const auto f = -5.2749;
const auto epsilon = 0.0001;
}  // namespace

namespace open_maeve {

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Comparisons, testExclusiveOr) {
  EXPECT_TRUE(exclusiveOr(0, 1));
  EXPECT_TRUE(exclusiveOr(1, 0));
  EXPECT_FALSE(exclusiveOr(0, 0));
  EXPECT_FALSE(exclusiveOr(1, 1));

  EXPECT_TRUE(exclusiveOr(false, true));
  EXPECT_TRUE(exclusiveOr(true, false));
  EXPECT_FALSE(exclusiveOr(false, false));
  EXPECT_FALSE(exclusiveOr(true, true));

  const boost::optional<std::string> a_true = std::string("Hello, world!");
  const boost::optional<std::string> a_false = boost::none;

  EXPECT_TRUE(exclusiveOr(a_false, a_true));
  EXPECT_TRUE(exclusiveOr(a_true, a_false));
  EXPECT_FALSE(exclusiveOr(a_false, a_false));
  EXPECT_FALSE(exclusiveOr(a_true, a_true));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Comparisons, testClampToZero) {
  EXPECT_EQ(clampToZero(0.05, 0.1), 0.0);
  EXPECT_EQ(clampToZero(-0.05, 0.1), 0.0);
  EXPECT_EQ(clampToZero(1.0, 0.04), 1.0);
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Comparisons, testZero) {
  EXPECT_TRUE(approxZero(d, epsilon));
  EXPECT_TRUE(approxZero(d + epsilon * epsilon, epsilon));
  EXPECT_FALSE(approxZero(d + 2.0 * epsilon, epsilon));
  EXPECT_FALSE(approxZero(1.0, epsilon));
  EXPECT_TRUE(approxZero(0.0, 0.0));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Comparisons, testRelEquality) {
  EXPECT_TRUE(approxRelEq(c, e, epsilon, 0.0));
  EXPECT_TRUE(approxRelEq(e, c, epsilon, 0.0));
  EXPECT_TRUE(approxRelEq(a, a, epsilon, 0.0));
  EXPECT_TRUE(approxRelEq(a, a, 0.0, 0.0));

  EXPECT_FALSE(approxRelEq(c, e, 0.0, 0.0));
  EXPECT_FALSE(approxRelEq(e, c, 0.0, 0.0));
  EXPECT_FALSE(approxRelEq(a, b, epsilon, 0.0));
  EXPECT_FALSE(approxRelEq(b, a, epsilon, 0.0));

  EXPECT_TRUE(approxRelEq(c, e, 0.0, 1.0));
  EXPECT_TRUE(approxRelEq(e, c, 0.0, 1.0));
  EXPECT_TRUE(approxRelEq(a, b, epsilon, 1.0));
  EXPECT_TRUE(approxRelEq(b, a, epsilon, 1.0));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Comparisons, testEquality) {
  EXPECT_TRUE(approxEq(c, e, epsilon));
  EXPECT_TRUE(approxEq(e, c, epsilon));
  EXPECT_FALSE(approxEq(c, e, 0.0));
  EXPECT_FALSE(approxEq(e, c, 0.0));
  EXPECT_FALSE(approxEq(a, b, epsilon));
  EXPECT_FALSE(approxEq(b, a, epsilon));
  EXPECT_TRUE(approxEq(a, a, epsilon));
  EXPECT_TRUE(approxEq(a, a, 0.0));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Comparisons, testInequality) {
  EXPECT_FALSE(approxNe(c, e, epsilon));
  EXPECT_FALSE(approxNe(e, c, epsilon));
  EXPECT_TRUE(approxNe(c, e, 0.0));
  EXPECT_TRUE(approxNe(e, c, 0.0));
  EXPECT_TRUE(approxNe(a, b, epsilon));
  EXPECT_TRUE(approxNe(b, a, epsilon));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Comparisons, testLessThan) {
  EXPECT_TRUE(approxLt(f, c, 0.0));
  EXPECT_TRUE(approxLt(f, c, epsilon));
  EXPECT_FALSE(approxLt(c, f, epsilon));
  EXPECT_FALSE(approxLt(d, d, epsilon));
  EXPECT_FALSE(approxLt(d, d, 0.0));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Comparisons, testLessThanEqual) {
  EXPECT_TRUE(approxLe(c, e, epsilon));
  EXPECT_TRUE(approxLe(e, c, epsilon));
  EXPECT_FALSE(approxLe(c, e, 0.0));
  EXPECT_TRUE(approxLe(e, c, 0.0));
  EXPECT_TRUE(approxLe(c, e, epsilon));
  EXPECT_TRUE(approxLe(e, c, epsilon));
  EXPECT_TRUE(approxLe(a, b, epsilon));
  EXPECT_FALSE(approxLe(b, a, epsilon));
  EXPECT_TRUE(approxLe(d, d, epsilon));
  EXPECT_TRUE(approxLe(d, d, 0.0));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Comparisons, testGreaterThan) {
  EXPECT_TRUE(approxGt(c, e, 0.0));
  EXPECT_FALSE(approxGt(c, e, epsilon));
  EXPECT_FALSE(approxGt(f, c, epsilon));
  EXPECT_TRUE(approxGt(c, f, epsilon));
  EXPECT_FALSE(approxGt(d, d, epsilon));
  EXPECT_FALSE(approxGt(d, d, 0.0));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Comparisons, testGreaterThanEqual) {
  EXPECT_TRUE(approxGe(c, e, 0.0));
  EXPECT_FALSE(approxGe(e, c, 0.0));
  EXPECT_TRUE(approxGe(c, e, epsilon));
  EXPECT_TRUE(approxGe(e, c, epsilon));
  EXPECT_FALSE(approxGe(f, c, epsilon));
  EXPECT_TRUE(approxGe(c, f, epsilon));
  EXPECT_TRUE(approxGe(d, d, epsilon));
  EXPECT_TRUE(approxGe(d, d, 0.0));
}

//------------------------------------------------------------------------------

}  // namespace open_maeve
