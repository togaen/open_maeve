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

#include "maeve_automation_core/isp_field/potential_transforms.h"

namespace maeve_automation_core {
namespace {
constexpr auto INF = std::numeric_limits<double>::infinity();
constexpr auto EPS = 1e-4;
constexpr auto t = 0.0;
constexpr auto r_min = 3.5;
constexpr auto r_max = 5.0;
constexpr auto a = 1.0;
constexpr auto b = 1.0;
const ShapeParameters sp(t, r_min, r_max, a, b);
}  // namespace

//------------------------------------------------------------------------------

TEST(PotentialTransform, testHard) {
  PotentialTransform<ConstraintType::HARD> h(sp);
  std::cout << "inf: " << h(cv::Point2d(INF, 0.0)) << std::endl;

  EXPECT_TRUE(false) << "TODO(me): This failing test is only a placeholder; "
                        "put a real test here.";
}

//------------------------------------------------------------------------------

TEST(PotentialTransform, testSoft) {
  PotentialTransform<ConstraintType::SOFT> h(sp);
  std::cout << "inf: " << h(cv::Point2d(INF, 0.0)) << std::endl;

  EXPECT_TRUE(false) << "TODO(me): This failing test is only a placeholder; "
                        "put a real test here.";
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
