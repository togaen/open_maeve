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

#include "open_maeve/maeve_geometry/comparisons.h"
#include "open_maeve/maeve_geometry/polynomial.h"

namespace open_maeve {
namespace {
const auto NaN = std::numeric_limits<double>::quiet_NaN();
const auto Inf = std::numeric_limits<double>::infinity();
const auto epsilon = 5e-4;
}  // namespace

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, comparators) {
  const auto P1 = Polynomial(0.0, 0.0, 0.0);
  const auto P2 = Polynomial(1.0, 0.0, 0.0);

  EXPECT_EQ(P1, P1);
  EXPECT_NE(P1, P2);
  EXPECT_NE(P1, Polynomial(P1, Interval_d(-1.0, 1.0)));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, domain) {
  EXPECT_NO_THROW({ const auto P = Polynomial(1.0, 1.0, 1.0); });
  EXPECT_THROW(
      {
        const auto P = Polynomial(1.0, 1.0, 1.0,
                                  Interval<double>::affinely_extended_reals());
      },
      std::domain_error);
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, is_constant) {
  EXPECT_TRUE(Polynomial::is_constant(Polynomial(0.0, 0.0, 1.0)));
  EXPECT_FALSE(Polynomial::is_constant(Polynomial(0.0, 1.0, 1.0)));
  EXPECT_FALSE(Polynomial::is_constant(Polynomial(1.0, 1.0, 1.0)));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, testQuadraticPointAtDerivative) {
  {
    const auto x = 0.0;
    const auto P = Polynomial(1, 1, 1);
    const auto dx = Polynomial::dx(P, x);
    const auto p2 = Polynomial::quadraticPointAtDerivative(P, dx);

    const Eigen::Vector2d p1(x, P(x));
    EXPECT_NEAR(p1.x(), p2.x(), epsilon);
    EXPECT_NEAR(p1.y(), p2.y(), epsilon);
  }

  {
    const auto x = 1.3;
    const auto P = Polynomial(1, 1, 1);
    const auto dx = Polynomial::dx(P, x);
    const auto p2 = Polynomial::quadraticPointAtDerivative(P, dx);

    const Eigen::Vector2d p1(x, P(x));
    EXPECT_NEAR(p1.x(), p2.x(), epsilon);
    EXPECT_NEAR(p1.y(), p2.y(), epsilon);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, testFromPointAndCriticalLine) {
  {
    const auto p = Eigen::Vector2d(1.0, 3.0);
    const auto y_critical = 0.0;
    const auto ddx = -2.0;

    const auto points =
        Polynomial::findConstrainedCriticalPoints(p, y_critical, ddx, epsilon);
    EXPECT_TRUE(!points);
  }

  {
    const auto p = Eigen::Vector2d(2.0, 3.0);
    const auto y_critical = 1.0;
    const auto ddx = 2.0;

    const auto points =
        Polynomial::findConstrainedCriticalPoints(p, y_critical, ddx, epsilon);
    ASSERT_FALSE(!points);

    Eigen::Vector2d pt1, pt2;
    std::tie(pt1, pt2) = *points;

    const auto p1 = Polynomial::from_point_with_derivatives(pt1, 0.0, ddx);
    const auto p2 = Polynomial::from_point_with_derivatives(pt2, 0.0, ddx);

    EXPECT_EQ(p1, Polynomial(2.0, -4.0, 3.0));
    EXPECT_EQ(p2, Polynomial(2.0, -12.0, 19.0));
  }

  {
    const auto p = Eigen::Vector2d(1.0, 3.0);
    const auto y_critical = 0.0;
    const auto ddx = 2.0;

    const auto points =
        Polynomial::findConstrainedCriticalPoints(p, y_critical, ddx, epsilon);
    ASSERT_FALSE(!points);

    Eigen::Vector2d pt1, pt2;
    std::tie(pt1, pt2) = *points;

    const auto p1 = Polynomial::from_point_with_derivatives(pt1, 0.0, ddx);
    const auto p2 = Polynomial::from_point_with_derivatives(pt2, 0.0, ddx);

    EXPECT_TRUE(
        Polynomial::approx_eq(p1, Polynomial(2.0, 0.89898, 0.10102), epsilon));
    EXPECT_TRUE(
        Polynomial::approx_eq(p2, Polynomial(2.0, -8.89898, 9.89898), epsilon));
  }

  {
    const auto p = Eigen::Vector2d(1.0, 3.0);
    const auto y_critical = 0.0;
    const auto ddx = 1.0;

    const auto points =
        Polynomial::findConstrainedCriticalPoints(p, y_critical, ddx, epsilon);
    ASSERT_FALSE(!points);

    Eigen::Vector2d pt1, pt2;
    std::tie(pt1, pt2) = *points;

    const auto p1 = Polynomial::from_point_with_derivatives(pt1, 0.0, ddx);
    const auto p2 = Polynomial::from_point_with_derivatives(pt2, 0.0, ddx);

    EXPECT_TRUE(
        Polynomial::approx_eq(p1, Polynomial(1.0, 1.4641, 0.5359), epsilon));
    EXPECT_TRUE(
        Polynomial::approx_eq(p2, Polynomial(1.0, -5.4641, 7.4641), epsilon));
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, testUniqueCriticalPoint) {
  {
    const auto p = Polynomial(0, 1, 2);
    const auto c = Polynomial::uniqueCriticalPoint(p);
    EXPECT_TRUE(!c);
  }

  {
    const auto p = Polynomial(0, 0, 1);
    const auto c = Polynomial::uniqueCriticalPoint(p);
    EXPECT_TRUE(!c);
  }

  {
    const auto p = Polynomial(3, 9, 5);
    const auto c = Polynomial::uniqueCriticalPoint(p);
    ASSERT_FALSE(!c);
    EXPECT_NEAR(c->x(), (-9.0 / 6.0), epsilon);
    EXPECT_NEAR(c->y(), p(c->x()), epsilon);
    EXPECT_NEAR(Polynomial::dx(p, c->x()), 0.0, epsilon);
  }

  {
    const auto P = Polynomial(1.0, 0.0, 0.0);
    const auto c = Polynomial::uniqueCriticalPoint(P);
    ASSERT_FALSE(!c);
    EXPECT_EQ(c->x(), 0.0);
    EXPECT_EQ(c->y(), 0.0);
  }

  {  // Critical point exists, but is outside the domain
    const auto P = Polynomial(1.0, 0.0, 0.0, Interval_d(10.0, 20.0));
    const auto c = Polynomial::uniqueCriticalPoint(P);
    ASSERT_TRUE(!c);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, testTangentRay) {
  {
    const auto q = Polynomial(3, 7, 2);
    const auto p_r = Eigen::Vector2d(3, -8);
    const auto p_t = Polynomial::tangentRaysThroughPoint(q, p_r);
    ASSERT_FALSE(!p_t);
    const auto p1 = Polynomial(p_r, std::get<0>(*p_t));
    const auto p2 = Polynomial(p_r, std::get<1>(*p_t));
    EXPECT_NEAR(Polynomial::dx(p1, p_r.x()), -1.38181, epsilon);
    EXPECT_NEAR(Polynomial::c(p1), -3.85456, epsilon);
    EXPECT_NEAR(Polynomial::dx(p2, p_r.x()), 51.3818, epsilon);
    EXPECT_NEAR(Polynomial::c(p2), -162.145, epsilon);
  }

  {
    const auto q = Polynomial(3, -7, -2);
    const auto p_r = Eigen::Vector2d(3, -8);
    const auto p_t = Polynomial::tangentRaysThroughPoint(q, p_r);
    ASSERT_FALSE(!p_t);
  }

  {
    const auto q = Polynomial(1, 0, -1);
    const auto p_r = Eigen::Vector2d(0, 0);
    const auto p_t = Polynomial::tangentRaysThroughPoint(q, p_r);
    ASSERT_TRUE(!p_t);
  }

  {
    const auto q = Polynomial(1, 0, 1);
    const auto p_r = Eigen::Vector2d(0, 0);
    const auto p_t = Polynomial::tangentRaysThroughPoint(q, p_r);
    ASSERT_FALSE(!p_t);
    const auto p1 = Polynomial(p_r, std::get<0>(*p_t));
    const auto p2 = Polynomial(p_r, std::get<1>(*p_t));
    EXPECT_NEAR(Polynomial::dx(p1, p_r.x()), -2.0, epsilon);
    EXPECT_NEAR(Polynomial::c(p1), 0.0, epsilon);
    EXPECT_NEAR(Polynomial::dx(p2, p_r.x()), 2.0, epsilon);
    EXPECT_NEAR(Polynomial::c(p2), 0.0, epsilon);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, testPointWithDerivates) {
  {
    const auto q = Polynomial(9, 17, 7);
    const auto x = 3.37;
    const auto p = Eigen::Vector2d(x, q(x));
    const auto dx = Polynomial::dx(q, p.x());
    const auto ddx = Polynomial::ddx(q);

    const auto q1 = Polynomial::from_point_with_derivatives(p, dx, ddx);
    EXPECT_NEAR(Polynomial::a(q1), Polynomial::a(q), epsilon)
        << "Polynomial: " << q1;
    EXPECT_NEAR(Polynomial::b(q1), Polynomial::b(q), epsilon)
        << "Polynomial: " << q1;
    EXPECT_NEAR(Polynomial::c(q1), Polynomial::c(q), epsilon)
        << "Polynomial: " << q1;
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, testRootFinder) {
  {
    const auto a = 393.7183923846020374;
    const auto c = 901.030976000000002836;
    const auto b = (2.0 * std::sqrt(a * c));
    const auto p = Polynomial(a, b, c);
    const auto roots = Polynomial::roots(a, b, c, 1e-6);
    EXPECT_FALSE(!roots);
  }

  {
    const auto q = Polynomial(1, 1, 1);
    const auto roots = Polynomial::roots(q, epsilon);
    EXPECT_TRUE(!roots);
  }

  {
    const auto q = Polynomial(1, 2, 1);
    const auto roots = Polynomial::roots(q, epsilon);
    ASSERT_FALSE(!roots);

    double r1, r2;
    std::tie(r1, r2) = *roots;
    EXPECT_EQ(r1, r2);
    EXPECT_NEAR(r2, -1.0, epsilon);
  }

  {
    const auto q = Polynomial(0, 2, 1);
    const auto roots = Polynomial::roots(q, epsilon);
    ASSERT_FALSE(!roots);

    double r1, r2;
    std::tie(r1, r2) = *roots;
    EXPECT_EQ(r1, r2);
    EXPECT_NEAR(r2, -0.5, epsilon);
  }

  {
    const auto q = Polynomial(1, -2, 0);
    const auto roots = Polynomial::roots(q, epsilon);
    ASSERT_FALSE(!roots);

    double r1, r2;
    std::tie(r1, r2) = *roots;
    EXPECT_EQ(r1, 0.0);
    EXPECT_NEAR(r2, 2.0, epsilon);
  }

  {
    const auto q = Polynomial(1, 2, 0);
    const auto roots = Polynomial::roots(q, epsilon);
    ASSERT_FALSE(!roots);

    double r1, r2;
    std::tie(r1, r2) = *roots;
    EXPECT_NEAR(r1, -2.0, epsilon);
    EXPECT_EQ(r2, 0.0);
  }

  {
    const auto q = Polynomial(3, 5, 2);
    const auto roots = Polynomial::roots(q, epsilon);
    ASSERT_FALSE(!roots);

    double r1, r2;
    std::tie(r1, r2) = *roots;
    EXPECT_NEAR(r1, -1.0, epsilon);
    EXPECT_NEAR(r2, -0.66666, epsilon);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, testDerivatives) {
  {
    const auto p = Polynomial(2, 3, 4);
    EXPECT_EQ(Polynomial::dx(p, 0.5), 4.0 * 0.5 + 3.0);
    EXPECT_EQ(Polynomial::ddx(p), 2.0);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, testEval) {
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

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Polynomial, testConstruction) {
  {
    EXPECT_THROW({ const auto p = Polynomial(Inf, Inf, Inf); },
                 std::domain_error);
  }

  {
    const auto p = Polynomial(1, 1, 1);
    EXPECT_TRUE(true);
  }

  {
    const auto p = Polynomial(Eigen::Vector2d(0, 0), Eigen::Vector2d(1, 0));
    double a, b, c;
    std::tie(a, b, c) = Polynomial::coefficients(p);
    EXPECT_EQ(a, 0.0);
    EXPECT_NEAR(b, 0, epsilon);
    EXPECT_NEAR(c, 0, epsilon);
  }

  {
    const auto p = Polynomial(Eigen::Vector2d(0, 0), Eigen::Vector2d(0, 1));
    double a, b, c;
    std::tie(a, b, c) = Polynomial::coefficients(p);
    EXPECT_EQ(a, 0.0);
    EXPECT_EQ(b, Inf);
    EXPECT_TRUE(std::isnan(c));
  }

  {
    const auto p = Polynomial(Eigen::Vector2d(0, 1), Eigen::Vector2d(0, 0));
    double a, b, c;
    std::tie(a, b, c) = Polynomial::coefficients(p);
    EXPECT_EQ(a, 0.0);
    EXPECT_EQ(b, -Inf);
    EXPECT_TRUE(std::isnan(c));
  }

  {
    const Eigen::Vector2d p1(1.0, 2.0);
    const Eigen::Vector2d p2(3.0, 7.0);
    const Eigen::Vector2d d = (p2 - p1);
    const auto p = Polynomial(p1, p2);
    double a, b, c;
    std::tie(a, b, c) = Polynomial::coefficients(p);
    EXPECT_EQ(a, 0.0);
    EXPECT_NEAR(b, (d.y() / d.x()), epsilon);
    EXPECT_NEAR(c, (p2.y() - (d.y() / d.x()) * p2.x()), epsilon);
  }

  {
    const Eigen::Vector2d p2(1.0, 2.0);
    const Eigen::Vector2d p1(3.0, 7.0);
    const Eigen::Vector2d d = (p2 - p1);
    const auto p = Polynomial(p1, p2);
    double a, b, c;
    std::tie(a, b, c) = Polynomial::coefficients(p);
    EXPECT_EQ(a, 0.0);
    EXPECT_NEAR(b, (d.y() / d.x()), epsilon);
    EXPECT_NEAR(c, (p2.y() - (d.y() / d.x()) * p2.x()), epsilon);
  }
}

//------------------------------------------------------------------------------

}  // namespace open_maeve
