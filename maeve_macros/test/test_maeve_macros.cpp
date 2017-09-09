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

#include "maeve_automation_core/maeve_macros/checks.h"

namespace maeve_automation_core {
TEST(MaeveMacros, testNotNaN) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testFinite) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testInfinite) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testNE) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testEQ) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testEven) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testOdd) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testLT) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testLE) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testGT) { EXPECT_TRUE(false); }
TEST(MaeveMacros, testGE) { EXPECT_TRUE(false); }
TEST(MaeveMacros, testContainsClosed) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testContainsOpen) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testStrictlyPositive) { EXPECT_TRUE(false); }

TEST(MaeveMacros, testNonempty) { EXPECT_TRUE(false); }
}  // namespace maeve_automation_core

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
