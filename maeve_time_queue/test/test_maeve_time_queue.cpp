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
TEST(MTQ, testDifferentiation1) {
  const auto buffer_size = 5;
  const auto max_time_gap = 10.0;
  const auto epsilon = 0.0001;

  MaeveTimeQueue<double> mtq(buffer_size, max_time_gap);

  EXPECT_TRUE(mtq.insert(0.0, 4.0));
  EXPECT_TRUE(mtq.insert(1.0, 3.0));
  EXPECT_TRUE(mtq.insert(2.0, 2.0));
  EXPECT_TRUE(mtq.insert(3.0, 1.0));
  EXPECT_TRUE(mtq.insert(4.0, 0.0));

  // Should work.
  if (const auto dt = mtq.bfd_dt(0.5)) {
    EXPECT_NEAR(std::get<0>(*dt), 1.0, epsilon);
    EXPECT_NEAR(std::get<1>(*dt), -1.0, epsilon);
  } else {
    EXPECT_TRUE(false);
  }

  // Shouldn't work for first element, or outside elements.
  EXPECT_TRUE(!mtq.bfd_dt(0.0));
  EXPECT_TRUE(!mtq.bfd_dt(-1.0));
  EXPECT_TRUE(!mtq.bfd_dt(4.5));

  // Should work.
  if (const auto dt = mtq.bfd_dt(4.0)) {
    EXPECT_NEAR(std::get<0>(*dt), 1.0, epsilon);
    EXPECT_NEAR(std::get<1>(*dt), -1.0, epsilon);
  } else {
    EXPECT_TRUE(false);
  }
}

TEST(MTQ, testDifferentiation2) {
  const auto buffer_size = 5;
  const auto max_time_gap = 10.0;
  const auto epsilon = 0.0001;

  MaeveTimeQueue<double> mtq(buffer_size, max_time_gap);

  EXPECT_TRUE(mtq.insert(0.0, 0.0));
  EXPECT_TRUE(mtq.insert(1.0, 1.0));
  EXPECT_TRUE(mtq.insert(2.0, 4.0));
  EXPECT_TRUE(mtq.insert(3.0, 9.0));
  EXPECT_TRUE(mtq.insert(4.0, 16.0));

  // Should work.
  {
    const auto dt = mtq.bfd_dt(0.5);
    ASSERT_FALSE(!dt);
    EXPECT_NEAR(std::get<0>(*dt), 1.0, epsilon);
    EXPECT_NEAR(std::get<1>(*dt), 1.0, epsilon);
  }

  {
    const auto dt = mtq.bfd_dt(2.5);
    ASSERT_FALSE(!dt);
    EXPECT_NEAR(std::get<0>(*dt), 1.0, epsilon);
    EXPECT_NEAR(std::get<1>(*dt), 5.0, epsilon);
  }

  // Test boundary cases.
  EXPECT_TRUE(!mtq.bfd_dt(0.0));   // first element
  EXPECT_TRUE(!mtq.bfd_dt(-1.0));  // outside element
  EXPECT_TRUE(!mtq.bfd_dt(4.5));   // outside element
  {
    const auto dt = mtq.bfd_dt(4.0);  // last element
    ASSERT_FALSE(!dt);
    EXPECT_NEAR(std::get<0>(*dt), 1.0, epsilon);
    EXPECT_NEAR(std::get<1>(*dt), 7.0, epsilon);
  }
}

TEST(MTQ, testException) {
  const auto buffer_size = 3;

  try {
    MaeveTimeQueue<double> mtq;
  } catch (...) {
    EXPECT_TRUE(false) << "Unexpected exception thrown.";
  }

  try {
    MaeveTimeQueue<double> mtq(buffer_size, -1.0);
  } catch (std::domain_error& e) {
  } catch (...) {
    EXPECT_TRUE(false) << "Wrong exception thrown.";
  }

  try {
    MaeveTimeQueue<double> mtq(buffer_size, 0.0);
  } catch (std::domain_error& e) {
  } catch (...) {
    EXPECT_TRUE(false) << "Wrong exception thrown.";
  }

  try {
    MaeveTimeQueue<double> mtq(buffer_size, 1.0);
  } catch (...) {
    EXPECT_TRUE(false) << "Unexpected exception thrown.";
  }
}

TEST(MTQ, testEmpty) {
  MaeveTimeQueue<double> mtq;
  EXPECT_TRUE(mtq.empty());
  EXPECT_FALSE(mtq.insert(0.0, 1.0));
  EXPECT_TRUE(mtq.empty());
}

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

  // Test boundary cases.
  const auto exact_match = mtq.get(0.4);
  ASSERT_FALSE(!exact_match);
  EXPECT_EQ(*exact_match, 4.0);
  const auto last_el = mtq.get(0.5);
  ASSERT_FALSE(!last_el);
  EXPECT_EQ(*last_el, 5.0);
  const auto first_el = mtq.get(0.3);
  ASSERT_FALSE(!first_el);
  EXPECT_EQ(*first_el, 3.0);

  // Test clearing.
  EXPECT_TRUE(mtq.insert(10.0, 1.0));
  EXPECT_EQ(mtq.size(), 1);
}
}  // namespace maeve_automation_core

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
