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

#include "maeve_automation_core/maeve_geometry/comparisons.h"

namespace {
const auto a = 1.317;
const auto b = 2.0;
const auto c = -5.2747;
const auto d = 0.0;
const auto e = -5.2747177;
const auto f = -5.2749;
const auto epsilon = 0.0001;
}  // namespace

namespace maeve_automation_core {

TEST(Maeve_Geometry, testEquality) {
  EXPECT_TRUE(approxEq(c, e, epsilon));
  EXPECT_TRUE(approxEq(e, c, epsilon));
  EXPECT_FALSE(approxEq(c, e, 0.0));
  EXPECT_FALSE(approxEq(e, c, 0.0));
  EXPECT_FALSE(approxEq(a, b, epsilon));
  EXPECT_FALSE(approxEq(b, a, epsilon));
}

TEST(Maeve_Geometry, testInequality) {
  EXPECT_FALSE(approxNe(c, e, epsilon));
  EXPECT_FALSE(approxNe(e, c, epsilon));
  EXPECT_TRUE(approxNe(c, e, 0.0));
  EXPECT_TRUE(approxNe(e, c, 0.0));
  EXPECT_TRUE(approxNe(a, b, epsilon));
  EXPECT_TRUE(approxNe(b, a, epsilon));
}

TEST(Maeve_Geometry, testLessThan) {
  EXPECT_TRUE(approxLt(f, c, 0.0));
  EXPECT_TRUE(approxLt(f, c, epsilon));
  EXPECT_FALSE(approxLt(c, f, epsilon));
  EXPECT_FALSE(approxLt(d, d, epsilon));
  EXPECT_FALSE(approxLt(d, d, 0.0));
}

TEST(Maeve_Geometry, testLessThanEqual) {
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

TEST(Maeve_Geometry, testGreaterThan) {
  EXPECT_TRUE(approxGt(c, e, 0.0));
  EXPECT_FALSE(approxGt(c, e, epsilon));
  EXPECT_FALSE(approxGt(f, c, epsilon));
  EXPECT_TRUE(approxGt(c, f, epsilon));
  EXPECT_FALSE(approxGt(d, d, epsilon));
  EXPECT_FALSE(approxGt(d, d, 0.0));
}

TEST(Maeve_Geometry, testGreaterThanEqual) {
  EXPECT_TRUE(approxGe(c, e, 0.0));
  EXPECT_FALSE(approxGe(e, c, 0.0));
  EXPECT_TRUE(approxGe(c, e, epsilon));
  EXPECT_TRUE(approxGe(e, c, epsilon));
  EXPECT_FALSE(approxGe(f, c, epsilon));
  EXPECT_TRUE(approxGe(c, f, epsilon));
  EXPECT_TRUE(approxGe(d, d, epsilon));
  EXPECT_TRUE(approxGe(d, d, 0.0));
}

}  // namespace maeve_automation_core
