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

#include "maeve_automation_core/maeve_geometry/optical_flow.h"

namespace maeve_automation_core {

//------------------------------------------------------------------------------

TEST(OpticalFlow, flow_component) {
  constexpr auto focal_length = 1.0;
  constexpr auto center = 10.0;
  constexpr auto pixel_coordinate = 5.0;
  constexpr auto translation_speed_parallel = 1.0;
  constexpr auto translation_speed_perpendicular = 1.0;

  {
    constexpr auto depth = 0.0;
    const auto component = flow_component(
        focal_length, center, depth, pixel_coordinate,
        translation_speed_parallel, translation_speed_perpendicular);
    EXPECT_TRUE(std::isnan(component))
        << "Computed component: " << component << ", but expected NaN";
  }
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
