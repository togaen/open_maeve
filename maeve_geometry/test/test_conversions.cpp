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

#include "maeve_core/maeve_geometry/conversions.h"

namespace {
const auto a = 1.317;
const auto b = 2.0;
const auto c = -5.2747;
const auto f = -5.2749;
}  // namespace

namespace maeve_core {

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Conversions, sqrt_magnitude) {
  EXPECT_EQ(std::sqrt(a), sqrt_magnitude(a));
  EXPECT_EQ(std::sqrt(b), sqrt_magnitude(b));
  EXPECT_EQ(-std::sqrt(std::abs(c)), sqrt_magnitude(c));
  EXPECT_EQ(-std::sqrt(std::abs(f)), sqrt_magnitude(f));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Conversions, degree_rad) {
  EXPECT_EQ(a, deg_2_rad(rad_2_deg(a)));
  EXPECT_EQ(M_PI, deg_2_rad(180.0));
  EXPECT_EQ(180.0, rad_2_deg(M_PI));
  EXPECT_EQ((0.5 * M_PI), deg_2_rad(90.0));
  EXPECT_EQ(90.0, rad_2_deg((0.5 * M_PI)));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Conversions, inverse) {
  constexpr auto VAL = 3.0;
  EXPECT_EQ(inverse(VAL), (1.0 / 3.0));
  EXPECT_EQ(inverse(inverse(VAL)), VAL);
  EXPECT_TRUE(std::isnan(inverse(0.0)));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Conversions, signed_value) {
  constexpr auto NEG = true;
  constexpr auto POS = false;
  constexpr auto VAL = 1.0;

  EXPECT_EQ(signed_value(VAL, false), VAL);
  EXPECT_EQ(signed_value(-VAL, false), VAL);
  EXPECT_EQ(signed_value(VAL, true), -VAL);
  EXPECT_EQ(signed_value(-VAL, true), -VAL);
}

//------------------------------------------------------------------------------

}  // namespace maeve_core
