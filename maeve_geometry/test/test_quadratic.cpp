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
namespace {
const auto epsilon = 0.00001;
}  // namespace

TEST(Maeve_Dynamics_Quadratic, testPointWithDerivates) {
  {
    const auto q = Quadratic(9, 17, 7);
    const auto x = 3.37;
    const auto p = Eigen::Vector2d(x, q(x));
    const auto dx = Quadratic::dx(q, p.x());
    const auto ddx = Quadratic::ddx(q);

    const auto q1 = Quadratic::fromPointWithDerivatives(p, dx, ddx);
    EXPECT_NEAR(Quadratic::a(q1), Quadratic::a(q), epsilon)
        << "Quadratic: " << q1;
    EXPECT_NEAR(Quadratic::b(q1), Quadratic::b(q), epsilon)
        << "Quadratic: " << q1;
    EXPECT_NEAR(Quadratic::c(q1), Quadratic::c(q), epsilon)
        << "Quadratic: " << q1;
  }
}

TEST(Maeve_Dynamics_Quadratic, testRootFinder) {
  {
    const auto q = Quadratic(1, 1, 1);
    double r1, r2;
    std::tie(r1, r2) = Quadratic::roots(q);
    EXPECT_TRUE(std::isnan(r1));
    EXPECT_TRUE(std::isnan(r2));
  }

  {
    const auto q = Quadratic(1, 2, 1);
    double r1, r2;
    std::tie(r1, r2) = Quadratic::roots(q);
    EXPECT_EQ(r1, r2);
    EXPECT_NEAR(r2, -1.0, epsilon);
  }

  {
    const auto q = Quadratic(0, 2, 1);
    double r1, r2;
    std::tie(r1, r2) = Quadratic::roots(q);
    EXPECT_EQ(r1, r2);
    EXPECT_NEAR(r2, -0.5, epsilon);
  }

  {
    const auto q = Quadratic(1, 2, 0);
    double r1, r2;
    std::tie(r1, r2) = Quadratic::roots(q);
    EXPECT_EQ(r1, r2);
    EXPECT_NEAR(r2, -2.0, epsilon);
  }

  {
    const auto q = Quadratic(3, 5, 2);
    double r1, r2;
    std::tie(r1, r2) = Quadratic::roots(q);
    EXPECT_NEAR(r1, -1.0, epsilon);
    EXPECT_NEAR(r2, -0.66666, epsilon);
  }
}

TEST(Maeve_Dynamics_Quadratic, testDerivatives) {
  {
    const auto p = Quadratic(2, 3, 4);
    EXPECT_EQ(Quadratic::dx(p, 0.5), 4.0 * 0.5 + 3.0);
    EXPECT_EQ(Quadratic::ddx(p), 2.0);
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
