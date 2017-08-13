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

#include "maeve_automation_core/maeve_time_queue/maeve_time_queue.h"

namespace maeve_automation_core {
TEST(MTQ, tests) {
  const auto buffer_size = 3;
  const auto max_time_gap = 0.75;
  const auto epsilon = 0.0001;

  MaeveTimeQueue<double> mtq(buffer_size, max_time_gap);

  // Insert into empty queue.
  EXPECT_TRUE(mtq.insert(0.0, 0.0));

  // Insert another.
  EXPECT_TRUE(mtq.insert(0.1, 1.0));

  // Try bad time stamps.
  EXPECT_FALSE(mtq.insert(0.1, 1.0));
  EXPECT_FALSE(mtq.insert(0.05, 1.0));

  // Right size?
  EXPECT_EQ(mtq.size(), 2);

  // Insert some more and test size.
  EXPECT_TRUE(mtq.insert(0.2, 2.0));
  EXPECT_TRUE(mtq.insert(0.3, 3.0));
  EXPECT_TRUE(mtq.insert(0.4, 4.0));
  EXPECT_TRUE(mtq.insert(0.5, 5.0));
  EXPECT_EQ(mtq.size(), 3);

  // Test interpolation.
  const auto point_three = mtq.get(0.3);
  ASSERT_FALSE(!point_three);
  EXPECT_EQ(*point_three, 3.0);
  const auto point_fourfive = mtq.get(0.45);
  ASSERT_FALSE(!point_fourfive);
  EXPECT_NEAR(*point_fourfive, 4.5, epsilon);
  const auto point_fourninethree = mtq.get(0.493);
  ASSERT_FALSE(!point_fourninethree);
  EXPECT_NEAR(*point_fourninethree, 4.93, epsilon);
  const auto point_five = mtq.get(0.5);
  ASSERT_FALSE(!point_five);
  EXPECT_EQ(*point_five, 5.0);

  // Test clearing.
  EXPECT_TRUE(mtq.insert(10.0, 1.0));
  EXPECT_EQ(mtq.size(), 1);
}
}  // namespace maeve_automation_core

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
