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

#include "maeve_automation_core/maeve_dynamics/pst_connector.h"

namespace maeve_automation_core {
namespace {
const auto epsilon = 0.0001;
}  // namespace

TEST(Maeve_Dynamics_PST_Connector, testComputePP) {
  {
    const Eigen::Vector2d p1(3, 2);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 1.0;
    const auto p2_ddt = -p1_ddt;
    const auto P1 = Polynomial::fromPointWithDerivatives(p1, p1_dt, p1_ddt);
    const auto P2 = Polynomial::fromPointWithDerivatives(
        Eigen::Vector2d(4.0, 3.0), 2.0, p2_ddt);
    const auto p2 = Polynomial::uniqueCriticalPoint(P2);
    ASSERT_FALSE(!p2);

    ASSERT_NO_THROW({
      const auto connector =
          PST_Connector::computePP(p1, p1_dt, p1_ddt, *p2, p2_ddt);
      ASSERT_FALSE(!connector);
      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(ss.str(),
                "{switching times: [3, 4, 4, 5], parabola coefficients: [{a: "
                "1.00000, b:-6.00000, c:11.00000}, {a: 0.00000, b:2.00000, "
                "c:-5.00000}, {a: -1.00000, b:10.00000, c:-21.00000}]}");
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
          PST_Connector::computePP(p1, p1_dt, p1_ddt, p2, p2_ddt);
      EXPECT_TRUE(!connector);
    });
  }

  {
    const Eigen::Vector2d p1(0.0, 0.0);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 1.0;
    const auto p2_ddt = -p1_ddt;
    const auto P1 = Polynomial::fromPointWithDerivatives(p1, p1_dt, p1_ddt);
    const auto P2 = Polynomial::fromPointWithDerivatives(
        Eigen::Vector2d(1.0, 1.0), 2.0, p2_ddt);
    const auto p2 = Polynomial::uniqueCriticalPoint(P2);
    ASSERT_FALSE(!p2);

    ASSERT_NO_THROW({
      const auto connector =
          PST_Connector::computePP(p1, p1_dt, p1_ddt, *p2, p2_ddt);
      ASSERT_FALSE(!connector);
      std::stringstream ss;
      ss << *connector;
      EXPECT_EQ(ss.str(),
                "{switching times: [0, 1, 1, 2], parabola coefficients: [{a: "
                "1.00000, b:0.00000, c:0.00000}, {a: 0.00000, b:2.00000, "
                "c:-1.00000}, {a: -1.00000, b:4.00000, c:-2.00000}]}");
    });
  }
}

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

TEST(Maeve_Dynamics_PST_Connector, testComputePL_0P) {
  {
    const Eigen::Vector2d p1(1.0, 3.0);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 4.0;
    const Eigen::Vector2d p2(p1.x() + 3.0,
                             p1.y() + Polynomial(p1_ddt, p1_dt, 0.0)(3.0));
    const auto p2_ddt = -4.0;
    const auto connector =
        PST_Connector::computePL_0P(p1, p1_dt, p1_ddt, p2, p2_ddt);
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
        PST_Connector::computePL_0P(p1, p1_dt, p1_ddt, p2, p2_ddt);
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
        PST_Connector::computePL_0P(p1, p1_dt, p1_ddt, p2, p2_ddt);
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
        PST_Connector::computePL_0P(p1, p1_dt, p1_ddt, p2, p2_ddt);
    ASSERT_FALSE(!connector);

    std::stringstream ss;
    ss << *connector;
    EXPECT_EQ(ss.str(),
              std::string("{switching times: [1, 1, 1, 4], parabola "
                          "coefficients: [{a: 4.00000, b:-8.00000, c:7.00000}, "
                          "{a: 0.00000, b:0.00000, c:3.00000}, {a: 4.00000, "
                          "b:-8.00000, c:7.00000}]}"));
  }

  {
    const Eigen::Vector2d p1(0, 0);
    const auto p1_dt = 0.0;
    const auto p1_ddt = 4.0;
    const Eigen::Vector2d p2(10, 2);
    const auto p2_ddt = 4.0;
    const auto connector =
        PST_Connector::computePL_0P(p1, p1_dt, p1_ddt, p2, p2_ddt);
    ASSERT_FALSE(!connector);

    std::stringstream ss;
    ss << *connector;
    EXPECT_EQ(ss.str(),
              std::string("{switching times: [0, -0, 9.29289, 10], parabola "
                          "coefficients: [{a: 4.00000, b:0.00000, c:0.00000}, "
                          "{a: 0.00000, b:0.00000, c:0.00000}, {a: 4.00000, "
                          "b:-74.34315, c:345.43146}]}"));
  }
}

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
        "{switching times: [0, 0, 5, 5], parabola coefficients: [{a: 0.00000, "
        "b:1.00000, c:0.00000}, {a: 0.00000, b:1.00000, c:0.00000}, {a: "
        "4.00000, b:-39.00000, c:100.00000}]}");
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
        "{switching times: [3, 3, 7.47214, 8], parabola coefficients: [{a: "
        "0.00000, b:0.77709, c:1.66874}, {a: 0.00000, b:0.77709, c:1.66874}, "
        "{a: 4.00000, b:-59.00000, c:225.00000}]}");
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
        "{switching times: [3, 3, 7.8734, 8], parabola coefficients: [{a: "
        "0.00000, b:1.01282, c:0.96153}, {a: 0.00000, b:1.01282, c:0.96153}, "
        "{a: -4.00000, b:64.00000, c:-247.00000}]}");
    EXPECT_EQ(ss.str(), expected_str);
  }
}

TEST(Maeve_Dynamics_PST_Connector, testEndPoints) {
  {
    const auto p = Polynomial(1, 1, 1);
    try {
      auto pc = PST_Connector({0.0, 1.0, 2.0, 3.0}, {p, p, p});
      EXPECT_NEAR(PST_Connector::initialSpeed(pc), 1.0, epsilon);
      EXPECT_NEAR(PST_Connector::terminalSpeed(pc), 7.0, epsilon);
    } catch (const std::exception& e) {
      ASSERT_FALSE(true) << "Unexpected exception: " << e.what();
    }
  }
}

TEST(Maeve_Dynamics_PST_Connector, testConstruction) {
  {
    const auto p = Polynomial(1, 1, 1);
    auto exception_thrown = false;
    try {
      auto pc = PST_Connector({0.0, 1.0, 3.0, 2.0}, {p, p, p});
    } catch (const std::exception& e) {
      std::cout << "Exception caught: " << e.what() << std::endl;
      exception_thrown = true;
    }
    EXPECT_TRUE(exception_thrown);
  }

  {
    const auto p = Polynomial(1, 1, 1);
    auto exception_thrown = false;
    try {
      auto pc = PST_Connector({0.0, 1.0, 2.0, 3.0}, {p, p, p});
    } catch (...) {
      exception_thrown = true;
    }
    EXPECT_FALSE(exception_thrown);
  }

  {
    const auto p = Polynomial(1, 1, 1);
    auto exception_thrown = false;
    try {
      auto pc = PST_Connector({0.0, 1.0, 2.0, 3.0}, {p, Polynomial(), p});
    } catch (...) {
      exception_thrown = true;
    }
    EXPECT_TRUE(exception_thrown);
  }
}
}  // namespace maeve_automation_core
