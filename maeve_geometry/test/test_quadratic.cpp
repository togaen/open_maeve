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

#include "maeve_automation_core/maeve_geometry/quadratic.h"

namespace maeve_automation_core {

TEST(Maeve_Dynamics_Quadratic, testDerivatives) {
  {
    const auto p = Quadratic(2, 3, 4);
    EXPECT_EQ(Quadratic::dt(p, 0.5), 4.0 * 0.5 + 3.0);
    EXPECT_EQ(Quadratic::ddt(p), 4.0);
  }
}

TEST(Maeve_Dynamics_Quadratic, testEval) {
  {
    const auto p = Quadratic(1, 1, 1);
    EXPECT_EQ(p(1.0), 3.0);
  }

  {
    const auto p = Quadratic(0, 0, 3);
    EXPECT_EQ(p(1.0), 3.0);
  }

  {
    const auto p = Quadratic(3.37, 2, 3);
    EXPECT_EQ(p(1.5), 3.37 * 1.5 * 1.5 + 2.0 * 1.5 + 3.0);
  }
}

TEST(Maeve_Dynamics_Quadratic, testConstruction) {
  {
    const auto p = Quadratic(1, 1, 1);
    EXPECT_TRUE(true);
  }

  {
    const auto p = Quadratic();
    EXPECT_TRUE(std::isnan(Quadratic::a(p)));
    EXPECT_TRUE(std::isnan(Quadratic::b(p)));
    EXPECT_TRUE(std::isnan(Quadratic::c(p)));
  }
}
}  // namespace maeve_automation_core
