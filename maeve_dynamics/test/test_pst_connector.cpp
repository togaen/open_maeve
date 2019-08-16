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

#include <sstream>

#include <gtest/gtest.h>

#include "open_maeve/maeve_dynamics/pst_connector.h"

namespace open_maeve {
namespace {
constexpr auto EPS = 1e-4;
}  // namespace

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_PST_Connector, taxonomy) {
  {
    const Eigen::Vector2d p1(3, 2);
    const auto p1_dt = 3.0;
    const auto p1_ddt = -3.0;
    const Eigen::Vector2d p2(5, 7);
    const auto p2_dt = 0.0;
    const auto p2_ddt = -3.0;

    const auto connector =
        PST_Connector::computePLP(p1, p1_dt, p1_ddt, p2, p2_dt, p2_ddt, EPS);
    ASSERT_FALSE(!connector);

    EXPECT_FALSE(PST_Connector::is_Pminus(*connector));
    EXPECT_FALSE(PST_Connector::is_PminusL_0(*connector));
  }

  {
    const auto p_line_up = Polynomial(0.0, 2.0, -2.0);
    const auto p_line_down = Polynomial(0.0, 2.0, 2.0);
    const auto p_plus = Polynomial(1.0, 0.0, -1.0);
    const auto p_minus = Polynomial(-1.0, 0.0, 1.0);
    const auto pst_plus =
        PST_Connector({0.0, 1.0, 1.0, 1.0}, {p_plus, p_line_up, p_line_up});
    const auto pst_minus = PST_Connector({-2.0, -1.0, -1.0, -1.0},
                                         {p_minus, p_line_down, p_line_down});
    EXPECT_TRUE(PST_Connector::is_Pminus(pst_minus));
    EXPECT_FALSE(PST_Connector::is_Pminus(pst_plus));
    EXPECT_FALSE(PST_Connector::is_PminusL_0(pst_minus));
    EXPECT_FALSE(PST_Connector::is_PminusL_0(pst_plus));
  }

  {
    const auto p_x_high = Polynomial(0.0, 0.0, 1.0);
    const auto p_x_low = Polynomial(0.0, 0.0, -1.0);
    const auto p_plus = Polynomial(1.0, 0.0, -1.0);
    const auto p_minus = Polynomial(-1.0, 0.0, 1.0);
    const auto pst_plus =
        PST_Connector({-1.0, 0.0, 1.0, 1.0}, {p_plus, p_x_low, p_x_low},
                      PST_Connector::SpeedConstraint::NONE);
    const auto pst_minus =
        PST_Connector({-1.0, 0.0, 1.0, 1.0}, {p_minus, p_x_high, p_x_high},
                      PST_Connector::SpeedConstraint::NONE);
    EXPECT_FALSE(PST_Connector::is_Pminus(pst_minus));
    EXPECT_FALSE(PST_Connector::is_Pminus(pst_plus));
    EXPECT_TRUE(PST_Connector::is_PminusL_0(pst_minus));
    EXPECT_FALSE(PST_Connector::is_PminusL_0(pst_plus));
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_PST_Connector, testComputePLP) {
  {
    const Eigen::Vector2d p1(3, 2);
    const auto p1_dt = -3.0;
    const auto p1_ddt = 0.0;
    const Eigen::Vector2d p2(5, 7);
    const auto p2_dt = 0.0;
    const auto p2_ddt = 3.0;

    EXPECT_NO_THROW({
      const auto connector =
          PST_Connector::computePLP(p1, p1_dt, p1_ddt, p2, p2_dt, p2_ddt, EPS);
      ASSERT_TRUE(!connector);
    });
  }

  {
    const Eigen::Vector2d p1(3, 2);
    const auto p1_dt = -3.0;
    const auto p1_ddt = -3.0;
    const Eigen::Vector2d p2(5, 7);
    const auto p2_dt = 0.0;
    const auto p2_ddt = -3.0;

    EXPECT_NO_THROW({
      const auto connector =
          PST_Connector::computePLP(p1, p1_dt, p1_ddt, p2, p2_dt, p2_ddt, EPS);
      ASSERT_TRUE(!connector);
    });
  }

  {
    const Eigen::Vector2d p1(3, 2);
    const auto p1_dt = 3.0;
    const auto p1_ddt = -2.0;
    const Eigen::Vector2d p2(5, 7);
    const auto p2_dt = 0.0;
    const auto p2_ddt = -2.0;

    EXPECT_NO_THROW({
      const auto connector =
          PST_Connector::computePLP(p1, p1_dt, p1_ddt, p2, p2_dt, p2_ddt, EPS);
      ASSERT_TRUE(!connector);
    });
  }

  {
    const Eigen::Vector2d p1(3, 2);
    const auto p1_dt = 3.0;
    const auto p1_ddt = -3.0;
    const Eigen::Vector2d p2(5, 7);
    const auto p2_dt = 0.0;
    const auto p2_ddt = -3.0;

    EXPECT_NO_THROW({
      const auto connector =
          PST_Connector::computePLP(p1, p1_dt, p1_ddt, p2, p2_dt, p2_ddt, EPS);
      ASSERT_FALSE(!connector);

      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(
          ss.str(),
          "{\"parabolic_segments\": [{\"a\": -3.00000, \"b\": 21.00000, "
          "\"c\": -34.00000, \"domain\": {\"min\": 3.00000, \"max\": "
          "3.02778}}, "
          "{\"a\": 0.00000, \"b\": 2.83333, \"c\": -6.49769, \"domain\": "
          "{\"min\": "
          "3.02778, \"max\": 4.52778}}, {\"a\": -3.00000, \"b\": 30.00000, "
          "\"c\": -68.00000, \"domain\": {\"min\": 4.52778, \"max\": "
          "5.00000}}]}");
    });
  }

  {
    const Eigen::Vector2d p1(3, 2);
    const auto p1_dt = 1.0;
    const auto p1_ddt = 2.0;
    const Eigen::Vector2d p2(5, 7);
    const auto p2_dt = 4.0;
    const auto p2_ddt = 2.0;

    EXPECT_NO_THROW({
      const auto connector =
          PST_Connector::computePLP(p1, p1_dt, p1_ddt, p2, p2_dt, p2_ddt, EPS);
      ASSERT_FALSE(!connector);

      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(
          ss.str(),
          "{\"parabolic_segments\": [{\"a\": 2.00000, \"b\": -11.00000, "
          "\"c\": 17.00000, \"domain\": {\"min\": 3.00000, \"max\": 3.37500}}, "
          "{\"a\": 0.00000, \"b\": 2.50000, \"c\": -5.78125, \"domain\": "
          "{\"min\": "
          "3.37500, \"max\": 4.62500}}, {\"a\": 2.00000, \"b\": -16.00000, "
          "\"c\": 37.00000, \"domain\": {\"min\": 4.62500, \"max\": "
          "5.00000}}]}");
    });
  }

  {
    const Eigen::Vector2d p1(3, 2);
    const auto p1_dt = 4.0;
    const auto p1_ddt = -4.0;
    const Eigen::Vector2d p2(5, 7);
    const auto p2_dt = 4.0;
    const auto p2_ddt = 3.0;

    EXPECT_NO_THROW({
      const auto connector =
          PST_Connector::computePLP(p1, p1_dt, p1_ddt, p2, p2_dt, p2_ddt, EPS);
      ASSERT_FALSE(!connector);

      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(
          ss.str(),
          "{\"parabolic_segments\": [{\"a\": -4.00000, \"b\": 28.00000, "
          "\"c\": -46.00000, \"domain\": {\"min\": 3.00000, \"max\": "
          "3.21429}}, "
          "{\"a\": 0.00000, \"b\": 2.28571, \"c\": -4.67347, \"domain\": "
          "{\"min\": "
          "3.21429, \"max\": 4.71429}}, {\"a\": 3.00000, \"b\": -26.00000, "
          "\"c\": 62.00000, \"domain\": {\"min\": 4.71429, \"max\": "
          "5.00000}}]}");
    });
  }

  {
    const Eigen::Vector2d p1(3, 2);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 4.0;
    const Eigen::Vector2d p2(5, 7);
    const auto p2_dt = 0.0;
    const auto p2_ddt = -3.0;

    EXPECT_NO_THROW({
      const auto connector =
          PST_Connector::computePLP(p1, p1_dt, p1_ddt, p2, p2_dt, p2_ddt, EPS);
      ASSERT_FALSE(!connector);

      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(
          ss.str(),
          "{\"parabolic_segments\": [{\"a\": 4.00000, \"b\": -24.00000, "
          "\"c\": 38.00000, \"domain\": {\"min\": 3.00000, \"max\": 3.41107}}, "
          "{\"a\": 0.00000, \"b\": 3.28857, \"c\": -8.54164, \"domain\": "
          "{\"min\": "
          "3.41107, \"max\": 4.45190}}, {\"a\": -3.00000, \"b\": 30.00000, "
          "\"c\": -68.00000, \"domain\": {\"min\": 4.45190, \"max\": "
          "5.00000}}]}");
    });
  }

  {
    const Eigen::Vector2d p1(0, 0);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 4.0;
    const Eigen::Vector2d p2(5, 5);
    const auto p2_dt = 0.0;
    const auto p2_ddt = -3.0;

    EXPECT_NO_THROW({
      const auto connector =
          PST_Connector::computePLP(p1, p1_dt, p1_ddt, p2, p2_dt, p2_ddt, EPS);
      ASSERT_FALSE(!connector);

      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(
          ss.str(),
          "{\"parabolic_segments\": [{\"a\": 4.00000, \"b\": 0.00000, \"c\": "
          "0.00000, \"domain\": {\"min\": 0.00000, \"max\": 0.12888}}, {\"a\": "
          "0.00000, \"b\": 1.03100, \"c\": -0.06644, \"domain\": {\"min\": "
          "0.12888, \"max\": 4.82817}}, {\"a\": -3.00000, \"b\": 30.00000, "
          "\"c\": -70.00000, \"domain\": {\"min\": 4.82817, \"max\": "
          "5.00000}}]}");
    });
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_PST_Connector, testComputePP) {
  {
    const Eigen::Vector2d p1(0.0, 0.0);
    const auto p1_dt = 1.0;
    const auto p1_ddt = 2.0;
    const auto p2_ddt = -p1_ddt;
    const auto p2 = Eigen::Vector2d(1.0, 1.3);

    ASSERT_NO_THROW({
      const auto connector =
          PST_Connector::computePP(p1, p1_dt, p1_ddt, p2, p2_ddt, EPS);
      ASSERT_TRUE(!connector);
    });
  }

  {
    const Eigen::Vector2d p1(3, 2);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 1.0;
    const auto p2_ddt = -p1_ddt;
    const auto P1 = Polynomial::from_point_with_derivatives(p1, p1_dt, p1_ddt);
    const auto P2 = Polynomial::from_point_with_derivatives(
        Eigen::Vector2d(4.0, 3.0), 2.0, p2_ddt);
    const auto p2 = Polynomial::uniqueCriticalPoint(P2);
    ASSERT_FALSE(!p2);

    ASSERT_NO_THROW({
      const auto connector =
          PST_Connector::computePP(p1, p1_dt, p1_ddt, *p2, p2_ddt, EPS);
      ASSERT_FALSE(!connector);
      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(
          ss.str(),
          "{\"parabolic_segments\": [{\"a\": 1.00000, \"b\": -6.00000, "
          "\"c\": 11.00000, \"domain\": {\"min\": 3.00000, \"max\": 4.00000}}, "
          "{\"a\": 0.00000, \"b\": 2.00000, \"c\": -5.00000, \"domain\": "
          "{\"min\": "
          "4.00000, \"max\": 4.00000}}, {\"a\": -1.00000, \"b\": 10.00000, "
          "\"c\": -21.00000, \"domain\": {\"min\": 4.00000, \"max\": "
          "5.00000}}]}");
    });
  }

  {
    const Eigen::Vector2d p1(0.0, 0.0);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 1.0;
    const auto p2_ddt = -p1_ddt;
    const auto p2 = Eigen::Vector2d(0.5, 0.5);

    ASSERT_NO_THROW({
      const auto connector =
          PST_Connector::computePP(p1, p1_dt, p1_ddt, p2, p2_ddt, EPS);
      EXPECT_TRUE(!connector);
    });
  }

  {
    const Eigen::Vector2d p1(0.0, 0.0);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 1.0;
    const auto p2_ddt = -p1_ddt;
    const auto P1 = Polynomial::from_point_with_derivatives(p1, p1_dt, p1_ddt);
    const auto P2 = Polynomial::from_point_with_derivatives(
        Eigen::Vector2d(1.0, 1.0), 2.0, p2_ddt);
    const auto p2 = Polynomial::uniqueCriticalPoint(P2);
    ASSERT_FALSE(!p2);

    ASSERT_NO_THROW({
      const auto connector =
          PST_Connector::computePP(p1, p1_dt, p1_ddt, *p2, p2_ddt, EPS);
      ASSERT_FALSE(!connector);
      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(
          ss.str(),
          "{\"parabolic_segments\": [{\"a\": 1.00000, \"b\": 0.00000, \"c\": "
          "0.00000, \"domain\": {\"min\": 0.00000, \"max\": 1.00000}}, {\"a\": "
          "0.00000, \"b\": 2.00000, \"c\": -1.00000, \"domain\": {\"min\": "
          "1.00000, \"max\": 1.00000}}, {\"a\": -1.00000, \"b\": 4.00000, "
          "\"c\": -2.00000, \"domain\": {\"min\": 1.00000, \"max\": "
          "2.00000}}]}");
    });
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_PST_Connector, testNoExceptConstruction) {
  const auto P = Polynomial(0, 0, 0);

  EXPECT_THROW(
      {
        const auto c = PST_Connector({3, 2, 1, 0}, {P, P, P});
      },
      std::exception);

  EXPECT_NO_THROW({
    const auto c =
        PST_Connector::noExceptionConstructor({3, 2, 1, 0}, {P, P, P});
  });
}

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_PST_Connector, testComputePL_0P) {
  {
    const Eigen::Vector2d p1(1.0, 3.0);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 4.0;
    const Eigen::Vector2d p2(p1.x() + 3.0,
                             p1.y() + Polynomial(p1_ddt, p1_dt, 0.0)(3.0));
    const auto p2_ddt = -4.0;
    const auto connector =
        PST_Connector::computePL_0P(p1, p1_dt, p1_ddt, p2, p2_ddt, EPS);
    ASSERT_TRUE(!connector);
  }

  {
    const Eigen::Vector2d p1(1.0, 3.0);
    const auto p1_dt = 0.0;
    const auto p1_ddt = -4.0;
    const Eigen::Vector2d p2(p1.x() + 3.0,
                             p1.y() + Polynomial(p1_ddt, p1_dt, 0.0)(3.0));
    const auto p2_ddt = -4.0;
    const auto connector =
        PST_Connector::computePL_0P(p1, p1_dt, p1_ddt, p2, p2_ddt, EPS);
    ASSERT_TRUE(!connector);
  }

  {
    const Eigen::Vector2d p1(1.0, 3.0);
    const auto p1_dt = 0.0;
    const auto p1_ddt = -4.0;
    const Eigen::Vector2d p2(p1.x() + 3.0,
                             p1.y() + Polynomial(p1_ddt, p1_dt, 0.0)(3.0));
    const auto p2_ddt = 4.0;
    const auto connector =
        PST_Connector::computePL_0P(p1, p1_dt, p1_ddt, p2, p2_ddt, EPS);
    ASSERT_TRUE(!connector);
  }

  {
    const Eigen::Vector2d p1(1.0, 3.0);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 4.0;
    const Eigen::Vector2d p2(p1.x() + 3.0,
                             p1.y() + Polynomial(p1_ddt, p1_dt, 0.0)(3.0));
    const auto p2_ddt = 4.0;
    const auto connector =
        PST_Connector::computePL_0P(p1, p1_dt, p1_ddt, p2, p2_ddt, EPS);
    ASSERT_FALSE(!connector);

    std::stringstream ss;
    ss << *connector;
    EXPECT_EQ(
        ss.str(),
        "{\"parabolic_segments\": [{\"a\": 4.00000, \"b\": -8.00000, \"c\": "
        "7.00000, \"domain\": {\"min\": 1.00000, \"max\": 1.00000}}, {\"a\": "
        "0.00000, \"b\": 0.00000, \"c\": 3.00000, \"domain\": {\"min\": "
        "1.00000, \"max\": 1.00000}}, {\"a\": 4.00000, \"b\": -8.00000, \"c\": "
        "7.00000, \"domain\": {\"min\": 1.00000, \"max\": 4.00000}}]}");
  }

  {
    const Eigen::Vector2d p1(0, 0);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 4.0;
    const Eigen::Vector2d p2(10, 2);
    const auto p2_ddt = 4.0;
    const auto connector =
        PST_Connector::computePL_0P(p1, p1_dt, p1_ddt, p2, p2_ddt, EPS);
    ASSERT_FALSE(!connector);

    std::stringstream ss;
    ss << *connector;
    EXPECT_EQ(
        ss.str(),
        "{\"parabolic_segments\": [{\"a\": 4.00000, \"b\": 0.00000, \"c\": "
        "0.00000, \"domain\": {\"min\": 0.00000, \"max\": -0.00000}}, {\"a\": "
        "0.00000, \"b\": 0.00000, \"c\": 0.00000, \"domain\": {\"min\": "
        "-0.00000, \"max\": 9.29289}}, {\"a\": 4.00000, \"b\": -74.34315, "
        "\"c\": 345.43146, \"domain\": {\"min\": 9.29289, \"max\": "
        "10.00000}}]}");
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_PST_Connector, testComputeLP) {
  {
    const Eigen::Vector2d p1(0, 0);
    const Eigen::Vector2d p2(5, 5);
    const auto p2_dt = 1.0;
    const auto p2_ddt = 4.0;

    const auto connector = PST_Connector::computeLP(p1, p2, p2_dt, p2_ddt);
    ASSERT_FALSE(!connector);

    std::stringstream ss;
    ss << *connector;
    const auto expected_str = std::string(
        "{\"parabolic_segments\": [{\"a\": 0.00000, \"b\": 1.00000, \"c\": "
        "0.00000, \"domain\": {\"min\": 0.00000, \"max\": 0.00000}}, {\"a\": "
        "0.00000, \"b\": 1.00000, \"c\": 0.00000, \"domain\": {\"min\": "
        "0.00000, \"max\": 5.00000}}, {\"a\": 4.00000, \"b\": -39.00000, "
        "\"c\": 100.00000, \"domain\": {\"min\": 5.00000, \"max\": "
        "5.00000}}]}");
    EXPECT_EQ(ss.str(), expected_str);
  }

  {
    const Eigen::Vector2d p1(3, 4);
    const Eigen::Vector2d p2(8, 9);
    const auto p2_dt = 5.0;
    const auto p2_ddt = 4.0;

    const auto connector = PST_Connector::computeLP(p1, p2, p2_dt, p2_ddt);
    ASSERT_FALSE(!connector);

    std::stringstream ss;
    ss << *connector;
    const auto expected_str = std::string(
        "{\"parabolic_segments\": [{\"a\": 0.00000, \"b\": 0.77709, \"c\": "
        "1.66874, \"domain\": {\"min\": 3.00000, \"max\": 3.00000}}, {\"a\": "
        "0.00000, \"b\": 0.77709, \"c\": 1.66874, \"domain\": {\"min\": "
        "3.00000, \"max\": 7.47214}}, {\"a\": 4.00000, \"b\": -59.00000, "
        "\"c\": 225.00000, \"domain\": {\"min\": 7.47214, \"max\": "
        "8.00000}}]}");
    EXPECT_EQ(ss.str(), expected_str);
  }

  {
    const Eigen::Vector2d p1(3, 4);
    const Eigen::Vector2d p2(8, 9);
    const auto p2_dt = 0.0;
    const auto p2_ddt = -4.0;

    const auto connector = PST_Connector::computeLP(p1, p2, p2_dt, p2_ddt);
    ASSERT_FALSE(!connector);

    std::stringstream ss;
    ss << *connector;
    const auto expected_str = std::string(
        "{\"parabolic_segments\": [{\"a\": 0.00000, \"b\": 1.01282, \"c\": "
        "0.96153, \"domain\": {\"min\": 3.00000, \"max\": 3.00000}}, {\"a\": "
        "0.00000, \"b\": 1.01282, \"c\": 0.96153, \"domain\": {\"min\": "
        "3.00000, \"max\": 7.87340}}, {\"a\": -4.00000, \"b\": 64.00000, "
        "\"c\": -247.00000, \"domain\": {\"min\": 7.87340, \"max\": "
        "8.00000}}]}");
    EXPECT_EQ(ss.str(), expected_str);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_PST_Connector, testEndPoints) {
  {
    const auto p = Polynomial(1, 1, 1);
    try {
      auto pc = PST_Connector({0.0, 1.0, 2.0, 3.0}, {p, p, p});
      EXPECT_NEAR(PST_Connector::initialSpeed(pc), 1.0, EPS);
      EXPECT_NEAR(PST_Connector::terminalSpeed(pc), 7.0, EPS);
    } catch (const std::exception& e) {
      ASSERT_FALSE(true) << "Unexpected exception: " << e.what();
    }
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Dynamics_PST_Connector, testConstruction) {
  {  // Test that switching times are non decreasing *and* domains are connected
    const auto t2 = 1.707106709480286;
    const auto t3 = 3.121320247650146;
    const auto P1 = Polynomial(1.0, 0.0, 0.0);
    const auto L = Polynomial::from_point_with_derivatives(
        Eigen::Vector2d(t2, P1(t2)), Polynomial::dx(P1, t2), 0.0,
        Interval_d(t2, t2));
    const auto P3 = Polynomial(-2.0, 10.242640687119284, -8.742641037942761,
                               Interval_d(t2, t3));

    auto exception_thrown = false;
    try {
      auto pc =
          PST_Connector({P1, L, P3}, PST_Connector::SpeedConstraint::NONE);
      std::cerr << "should throw: " << pc << std::endl;
    } catch (const std::exception& e) {
      std::cout << "Exception correctly thrown: " << e.what() << std::endl;
      exception_thrown = true;
    }
    EXPECT_TRUE(exception_thrown);
  }

  {
    auto exception_thrown = false;
    try {
      auto pc = PST_Connector(
          {0.0, 0.34808, 0.34808, 1.0},
          {Polynomial(2.0, 1.0, 0.0), Polynomial(0.0, 2.39232, -0.24232),
           Polynomial(-2.0, 3.78464, -0.48464)});
    } catch (const std::exception& e) {
      std::cout << "Exception correctly thrown: " << e.what() << std::endl;
      exception_thrown = true;
    }
    EXPECT_TRUE(exception_thrown);
  }

  {
    const auto p = Polynomial(1, 1, 1);
    auto exception_thrown = false;
    try {
      auto pc = PST_Connector({0.0, 1.0, 3.0, 2.0}, {p, p, p});
    } catch (const std::exception& e) {
      std::cout << "Exception correctly thrown: " << e.what() << std::endl;
      exception_thrown = true;
    }
    EXPECT_TRUE(exception_thrown);
  }

  EXPECT_NO_THROW({
    const auto p = Polynomial(1, 1, 1);
    auto pc = PST_Connector({0.0, 1.0, 2.0, 3.0}, {p, p, p});
  });
}

//------------------------------------------------------------------------------

}  // namespace open_maeve
