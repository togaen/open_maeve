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

#include "open_maeve/isp_field/shape_parameters.h"

namespace open_maeve {
namespace {
constexpr auto epsilon = 0.00001;
}  // namespace

TEST(ShapeParams, testMidPoint) {
  const auto t = 0.0;
  const auto r_min = 3.5;
  const auto r_max = 5.0;
  const auto a = 1.0;
  const auto b = 1.0;
  ShapeParameters sp(t, r_min, r_max, a, b);
  EXPECT_NEAR(sp.rangeMidPoint(), (r_min + r_max) / 2.0, epsilon);
  EXPECT_TRUE(std::isnan(ShapeParameters().rangeMidPoint()));
}
}  // namespace open_maeve
