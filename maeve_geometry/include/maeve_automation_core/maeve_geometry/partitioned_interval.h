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
#pragma once

#include "interval.h"

#include <iostream>

namespace maeve_automation_core {
template <typename T>
class PartitionedInterval {
 public:
  /** @brief Overload stream output. */
  friend std::ostream& operator<<(
      std::ostream& os, const PartitionedInterval& partitioned_interval) {
    return (os << "{interval: " << partitioned_interval.interval
               << ", partition point: " << partitioned_interval.partition_point
               << "}");
  }

  /**
   * @brief Equality/inequality comparison operators.
   * @{
   */
  friend bool operator==(const PartitionedInterval& partitioned_interval1,
                         const PartitionedInterval& partitioned_interval2) {
    const auto intervals_equal =
        (partitioned_interval1.interval == partitioned_interval2.interval);
    const auto partition_points_equal =
        (partitioned_interval1.partition_point ==
         partitioned_interval2.partition_point);
    return (intervals_equal && partition_points_equal);
  }

  friend bool operator!=(const PartitionedInterval& partitioned_interval1,
                         const PartitionedInterval& partitioned_interval2) {
    return !(partitioned_interval1 == partitioned_interval2);
  }
  /** @} */

  /**
   * @brief Accessors.
   * @{
   */
  static const T& get_partition_point(
      const PartitionedInterval& partitioned_interval);
  static const Interval<T>& get_interval(
      const PartitionedInterval& partitioned_interval);
  /** @} */

  /** @brief Return [min(interval), partition_point] */
  static Interval<T> get_lower_partition(
      const PartitionedInterval& partitioned_interval);

  /** @brief Return [partition_point, max(interval)] */
  static Interval<T> get_upper_partition(
      const PartitionedInterval& partitioned_interval);

  /**
   * @brief Project 'val' from its spot on the from_range to the to_range
   * maintaining its partition membership.
   */
  static T project_to_range(const T& val, const PartitionedInterval& from_range,
                            const PartitionedInterval& to_range);

  /**
   * @brief Various constructors.
   *
   * @note If no parition_point is specified, it is assumed to be the center of
   * the given interval.
   *
   * @pre The specified interval must not be empty
   * @pre The specified partition point must be contained on the interval
   *
   * @post The interval is non-empty.
   *
   * @{
   */
  explicit PartitionedInterval(Interval<T> interval);
  PartitionedInterval(Interval<T> interval, T partition_point);
  PartitionedInterval(T minimum, T partition_point, T maximum);
  /** @} */

 private:
  Interval<T> interval;
  T partition_point;
};

//------------------------------------------------------------------------------

template <typename T>
PartitionedInterval<T>::PartitionedInterval(Interval<T> interval,
                                            T partition_point)
    : interval(std::move(interval)),
      partition_point(std::move(partition_point)) {
  if (Interval<T>::empty(interval)) {
    throw std::runtime_error(
        "Attempted to partition an empty interval. Cannot continue.");
  }
  if (!Interval<T>::contains(interval, partition_point)) {
    std::stringstream ss;
    ss << "Attempted to set partition point " << partition_point
       << " outside of partitioned interval " << interval
       << ". Cannot continue.";
    throw std::runtime_error(ss.str());
  }
}

//------------------------------------------------------------------------------

template <typename T>
PartitionedInterval<T>::PartitionedInterval(T minimum, T partition_point,
                                            T maximum)
    : PartitionedInterval(Interval<T>(std::move(minimum), std::move(maximum)),
                          std::move(partition_point)) {}

//------------------------------------------------------------------------------

template <typename T>
PartitionedInterval<T>::PartitionedInterval(Interval<T> interval)
    : PartitionedInterval(std::move(interval), Interval<T>::center(interval)) {}

//------------------------------------------------------------------------------

template <typename T>
const T& PartitionedInterval<T>::get_partition_point(
    const PartitionedInterval& partitioned_interval) {
  return partitioned_interval.partition_point;
}

//------------------------------------------------------------------------------

template <typename T>
const Interval<T>& PartitionedInterval<T>::get_interval(
    const PartitionedInterval& partitioned_interval) {
  return partitioned_interval.interval;
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> PartitionedInterval<T>::get_lower_partition(
    const PartitionedInterval& partitioned_interval) {
  const auto min = Interval<T>::min(partitioned_interval.interval);
  const auto max =
      PartitionedInterval::get_partition_point(partitioned_interval);

  return Interval<T>(min, max);
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> PartitionedInterval<T>::get_upper_partition(
    const PartitionedInterval& partitioned_interval) {
  const auto min =
      PartitionedInterval::get_partition_point(partitioned_interval);
  const auto max = Interval<T>::max(partitioned_interval.interval);

  return Interval<T>(min, max);
}

//------------------------------------------------------------------------------

template <typename T>
T PartitionedInterval<T>::project_to_range(
    const T& val, const PartitionedInterval& from_range,
    const PartitionedInterval& to_range) {
  const auto val_in_lower_partition =
      (val < PartitionedInterval::get_partition_point(from_range));

  const Interval<T> from_partition =
      (val_in_lower_partition
           ? PartitionedInterval::get_lower_partition(from_range)
           : PartitionedInterval::get_upper_partition(from_range));

  const Interval<T> to_partition =
      (val_in_lower_partition
           ? PartitionedInterval::get_lower_partition(to_range)
           : PartitionedInterval::get_upper_partition(to_range));

  return Interval<T>::project_to_range(val, from_partition, to_partition);
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
