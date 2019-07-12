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

#include "maeve_automation_core/maeve_dynamics/simple.h"

namespace maeve_automation_core {
namespace {
constexpr auto EPS = 1e-4;
}  // namespace

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_simple, simple_motion) {
  {
    constexpr auto t = 1.0;
    constexpr auto v = 0.0;
    constexpr auto a = 1.0;
    const auto sd = simple_motion(t, v, a);

    const SimpleDisplacement<double> expected_sd = {0.5, 1.0};
    EXPECT_EQ(sd, expected_sd);
  }

  {
    constexpr auto t = 1.0;
    constexpr auto v = 1.0;
    constexpr auto a = 1.0;
    const auto sd = simple_motion(t, v, a);

    const SimpleDisplacement<double> expected_sd = {1.5, 2.0};
    EXPECT_EQ(sd, expected_sd);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_simple, time_to_zero_relative_speed_already_equal) {
  constexpr auto v1 = 1.0;
  constexpr auto v2 = 1.0;
  constexpr auto a1 = 0.0;
  constexpr auto a2 = 0.0;
  EXPECT_EQ(time_to_zero_relative_speed(v1, v2, a1, a2, EPS), 0.0);
}

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_simple, time_to_zero_relative_speed_never_equal) {
  constexpr auto v1 = 1.0;
  constexpr auto v2 = 2.0;
  constexpr auto a1 = 0.0;
  constexpr auto a2 = 0.0;
  EXPECT_TRUE(std::isnan(time_to_zero_relative_speed(v1, v2, a1, a2, EPS)));
}

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_simple, time_to_zero_relative_speed_in_future) {
  constexpr auto v1 = 1.0;
  constexpr auto v2 = 2.0;
  constexpr auto a1 = 1.0;
  constexpr auto a2 = 0.0;
  EXPECT_EQ(time_to_zero_relative_speed(v1, v2, a1, a2, EPS), 1.0);
}

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_simple, time_to_zero_relative_speed_in_past) {
  constexpr auto v1 = 1.0;
  constexpr auto v2 = 2.0;
  constexpr auto a1 = 0.0;
  constexpr auto a2 = 1.0;
  EXPECT_EQ(time_to_zero_relative_speed(v1, v2, a1, a2, EPS), -1.0);
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
