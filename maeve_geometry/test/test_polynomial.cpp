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

#include "maeve_automation_core/maeve_geometry/polynomial.h"

namespace maeve_automation_core {
namespace {
const auto epsilon = 0.00001;
}  // namespace

TEST(Maeve_Dynamics_Polynomial, testTangentRay) {
  {
    const auto q = Polynomial(3, -7, -2);
    const auto p_r = Eigen::Vector2d(3, -8);
    const auto p_q = Eigen::Vector2d(0, q(0));
    const auto p_t = Polynomial::tangentOfRayThroughPoint(q, p_r, p_q);
    ASSERT_FALSE(!p_t);
    const auto p1 = std::get<0>(*p_t);
    const auto del1 = (p1 - p_r);
    const auto m1 = del1.y() / del1.x();
    const auto p2 = std::get<1>(*p_t);
    const auto del2 = (p2 - p_r);
    const auto m2 = del2.y() / del2.x();
    std::cout << "p1: " << p1.transpose() << " : " << m1 << " : " << (p_r.y() - m1 * p_r.x()) << ", p2: " << p2.transpose() << " : " << m2 << " : " << (p_r.y() - m2 * p_r.x()) << std::endl;
  }

  {
    const auto q = Polynomial(1, 0, -1);
    const auto p_r = Eigen::Vector2d(0, 0);
    const auto p_q = Eigen::Vector2d(0, 1);
    const auto p_t = Polynomial::tangentOfRayThroughPoint(q, p_r, p_q);
    ASSERT_TRUE(!p_t);
  }

  {
    const auto q = Polynomial(1, 0, 1);
    const auto p_r = Eigen::Vector2d(0, 0);
    const auto p_q = Eigen::Vector2d(0, 1);
    const auto p_t = Polynomial::tangentOfRayThroughPoint(q, p_r, p_q);
    ASSERT_FALSE(!p_t);
    const auto p1 = std::get<0>(*p_t);
    const auto p2 = std::get<1>(*p_t);
    EXPECT_NEAR(p1.x(), -1.0, epsilon);
    EXPECT_NEAR(p1.y(), 2.0, epsilon);
    EXPECT_NEAR(p2.x(), 1.0, epsilon);
    EXPECT_NEAR(p2.y(), 2.0, epsilon);
  }
}

TEST(Maeve_Dynamics_Polynomial, testPointWithDerivates) {
  {
    const auto q = Polynomial(9, 17, 7);
    const auto x = 3.37;
    const auto p = Eigen::Vector2d(x, q(x));
    const auto dx = Polynomial::dx(q, p.x());
    const auto ddx = Polynomial::ddx(q);

    const auto q1 = Polynomial::fromPointWithDerivatives(p, dx, ddx);
    EXPECT_NEAR(Polynomial::a(q1), Polynomial::a(q), epsilon)
        << "Polynomial: " << q1;
    EXPECT_NEAR(Polynomial::b(q1), Polynomial::b(q), epsilon)
        << "Polynomial: " << q1;
    EXPECT_NEAR(Polynomial::c(q1), Polynomial::c(q), epsilon)
        << "Polynomial: " << q1;
  }
}

TEST(Maeve_Dynamics_Polynomial, testRootFinder) {
  {
    const auto q = Polynomial(1, 1, 1);
    double r1, r2;
    std::tie(r1, r2) = Polynomial::roots(q);
    EXPECT_TRUE(std::isnan(r1));
    EXPECT_TRUE(std::isnan(r2));
  }

  {
    const auto q = Polynomial(1, 2, 1);
    double r1, r2;
    std::tie(r1, r2) = Polynomial::roots(q);
    EXPECT_EQ(r1, r2);
    EXPECT_NEAR(r2, -1.0, epsilon);
  }

  {
    const auto q = Polynomial(0, 2, 1);
    double r1, r2;
    std::tie(r1, r2) = Polynomial::roots(q);
    EXPECT_EQ(r1, r2);
    EXPECT_NEAR(r2, -0.5, epsilon);
  }

  {
    const auto q = Polynomial(1, 2, 0);
    double r1, r2;
    std::tie(r1, r2) = Polynomial::roots(q);
    EXPECT_EQ(r1, r2);
    EXPECT_NEAR(r2, -2.0, epsilon);
  }

  {
    const auto q = Polynomial(3, 5, 2);
    double r1, r2;
    std::tie(r1, r2) = Polynomial::roots(q);
    EXPECT_NEAR(r1, -1.0, epsilon);
    EXPECT_NEAR(r2, -0.66666, epsilon);
  }
}

TEST(Maeve_Dynamics_Polynomial, testDerivatives) {
  {
    const auto p = Polynomial(2, 3, 4);
    EXPECT_EQ(Polynomial::dx(p, 0.5), 4.0 * 0.5 + 3.0);
    EXPECT_EQ(Polynomial::ddx(p), 2.0);
  }
}

TEST(Maeve_Dynamics_Polynomial, testEval) {
  {
    const auto p = Polynomial(1, 1, 1);
    EXPECT_EQ(p(1.0), 3.0);
  }

  {
    const auto p = Polynomial(0, 0, 3);
    EXPECT_EQ(p(1.0), 3.0);
  }

  {
    const auto p = Polynomial(3.37, 2, 3);
    EXPECT_EQ(p(1.5), 3.37 * 1.5 * 1.5 + 2.0 * 1.5 + 3.0);
  }
}

TEST(Maeve_Dynamics_Polynomial, testConstruction) {
  {
    const auto p = Polynomial(1, 1, 1);
    EXPECT_TRUE(true);
  }

  {
    const auto p = Polynomial();
    EXPECT_TRUE(std::isnan(Polynomial::a(p)));
    EXPECT_TRUE(std::isnan(Polynomial::b(p)));
    EXPECT_TRUE(std::isnan(Polynomial::c(p)));
  }
}
}  // namespace maeve_automation_core
