/*
 * Copyright 2019 Maeve Automation
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

#include "maeve_automation_core/maeve_geometry/partitioned_interval.h"

namespace maeve_automation_core {

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Partitioned_Interval, throwing) {
  EXPECT_THROW(
      { const auto p_i = PartitionedInterval<double>(Interval<double>()); },
      std::runtime_error);

  EXPECT_THROW(
      {
        const auto p_i =
            PartitionedInterval<double>(Interval<double>(0.0, 1.0), 2.0);
      },
      std::runtime_error);

  EXPECT_THROW(
      {
        const auto p_i = PartitionedInterval<double>(Interval<double>(), 2.0);
      },
      std::runtime_error);

  EXPECT_NO_THROW({
    const auto p_i = PartitionedInterval<double>(Interval<double>(0.0, 3.0));
  });

  EXPECT_NO_THROW({
    const auto p_i =
        PartitionedInterval<double>(Interval<double>(0.0, 3.0), 2.0);
  });

  EXPECT_NO_THROW(
      { const auto p_i = PartitionedInterval<double>(0.0, 2.0, 3.0); });
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Partitioned_Interval, construction) {
  const auto i = Interval<double>(0.0, 1.0);
  const auto p_i = PartitionedInterval<double>(i);
  EXPECT_EQ(PartitionedInterval<double>::get_partition_point(p_i), 0.5);
  EXPECT_EQ(PartitionedInterval<double>::get_partition_point(p_i),
            Interval<double>::center(i));
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Partitioned_Interval, get_lower_partition) {
  {
    const auto p_i = PartitionedInterval<double>(-8.0, 0.0, 4.0);
    const auto expected_partition = Interval<double>(-8.0, 0.0);
    EXPECT_EQ(PartitionedInterval<double>::get_lower_partition(p_i),
              expected_partition);
  }

  {
    const auto p_i = PartitionedInterval<double>(-8.0, 1.0, 4.0);
    const auto expected_partition = Interval<double>(-8.0, 1.0);
    EXPECT_EQ(PartitionedInterval<double>::get_lower_partition(p_i),
              expected_partition);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Partitioned_Interval, get_upper_partition) {
  {
    const auto p_i = PartitionedInterval<double>(-8.0, 0.0, 4.0);
    const auto expected_partition = Interval<double>(0.0, 4.0);
    EXPECT_EQ(PartitionedInterval<double>::get_upper_partition(p_i),
              expected_partition);
  }

  {
    const auto p_i = PartitionedInterval<double>(-8.0, 1.0, 4.0);
    const auto expected_partition = Interval<double>(1.0, 4.0);
    EXPECT_EQ(PartitionedInterval<double>::get_upper_partition(p_i),
              expected_partition);
  }
}

//------------------------------------------------------------------------------

TEST(Maeve_Geometry_Partitioned_Interval, project_to_range) {
  const auto p_i_from = PartitionedInterval<double>(-8.0, 0.0, 4.0);
  const auto p_i_to = PartitionedInterval<double>(-1.0, 0.0, 1.0);

  EXPECT_EQ(
      PartitionedInterval<double>::project_to_range(-4.0, p_i_from, p_i_to),
      -0.5);
  EXPECT_EQ(
      PartitionedInterval<double>::project_to_range(4.0, p_i_from, p_i_to),
      1.0);
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
