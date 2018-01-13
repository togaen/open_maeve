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

#include "maeve_automation_core/maeve_dynamics/pst_connector.h"

namespace maeve_automation_core {
namespace {
const auto epsilon = 0.0001;
}  // namespace

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
