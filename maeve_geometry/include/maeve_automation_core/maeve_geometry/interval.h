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
#pragma once

#include <algorithm>
#include <cmath>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <tuple>
#include <type_traits>

#include <boost/io/ios_state.hpp>
#include <boost/optional.hpp>

#include "maeve_automation_core/maeve_geometry/comparisons.h"

namespace maeve_automation_core {
/**
 * @brief Data structure to contain scalar interval bounds.
 *
 * @note An exception is thrown on construction if the provided interval bounds
 * are invalid (i.e., if min > max).
 */
template <typename T>
class Interval {
  static_assert(std::is_floating_point<T>::value,
                "Intervals only support floating point types.");

 public:
  /** @name Special interval bound values
   *
   * These values are used to construct special types of intervals.
   */
  static constexpr T MIN = std::numeric_limits<T>::lowest();
  static constexpr T MAX = std::numeric_limits<T>::max();
  static constexpr T SUPREMUM = std::numeric_limits<T>::infinity();
  static constexpr T INFIMUM = -SUPREMUM;
  /** @} */

  /** @name Comparison operations
   *
   * Comparison operations for Interval types.
   *
   * @note Like comparisons of NaN, invalid intervals always compare false.
   *
   * @{
   */

  /**
   * @brief Compute equality.
   *
   * Two intervals compare equal iff they are both valid and they are both
   * either empty or have equal bounds.
   *
   * @param interval1 The first interval to compare.
   * @param interval2 The second interval to compare.
   *
   * @return True if the intervals compare equal; otherwise false.
   */
  friend bool operator==(const Interval& interval1, const Interval& interval2) {
    // Capture properties.
    const auto min_eq = (Interval::min(interval1) == Interval::min(interval2));
    const auto max_eq = (Interval::max(interval1) == Interval::max(interval2));
    const auto both_empty =
        (Interval::empty(interval1) && Interval::empty(interval2));

    // Compute equality: for empty intervals min_eq and max_eq are both 'false'
    // because the bounds are all NaN.
    return (both_empty || (min_eq && max_eq));
  }

  /**
   * @brief Compute inequality.
   *
   * This always returns the negation of the == operator.
   *
   * @param interval1 The first interval to compare.
   * @param interval2 the second interval to compare.
   *
   * @return True if the intervals do not compare equal; otherwise false.
   */
  friend bool operator!=(const Interval& interval1, const Interval& interval2) {
    return !(interval1 == interval2);
  }

  /** }@ */

  /**
   * @brief Stream overload for Interval types.
   *
   * @note Output precision is fixed inside the function definition, and the
   * serialization is JSON compatible.
   *
   * @param os The output stream.
   * @param interval The interval to serialize.
   *
   * @return The output stream with the serialized interval.
   */
  friend std::ostream& operator<<(std::ostream& os, const Interval& interval) {
    static const auto PRECISION = 5;

    // Restore stream state on exit.
    boost::io::ios_all_saver guard(os);

    // Temporarily set desired flags and precision.
    os.setf(std::ios::fixed, std::ios::floatfield);
    os.precision(PRECISION);

    // Do stream output.
    if (Interval::empty(interval)) {
      return os << "{}";
    }

    // Improve readability.
    if (interval == Interval::max_representable_reals()) {
      return os << "{\"min\": REAL_MIN, \"max\": REAL_MAX}";
    }

    return os << "{\"min\": " << Interval::min(interval)
              << ", \"max\": " << Interval::max(interval) << "}";
  }

  /**
   * @brief Define interval addition as the addition of interval bounds.
   *
   * @note For the purpose of addition, the following special behaviors hold:
   *
   *     1) When both intervals are empty:         [] + [] = []
   *
   *     2) When exactly one interval is empty:    [] == [0, 0]
   */
  friend Interval operator+(const Interval& interval1,
                            const Interval& interval2) {
    // TODO(me): This check comes up a lot in interval operations. Need to give
    // it its own definition.
    if (Interval::empty(interval1)) {
      return interval2;
    }
    if (Interval::empty(interval2)) {
      return interval1;
    }
    const auto min = (Interval::min(interval1) + Interval::min(interval2));
    const auto max = (Interval::max(interval1) + Interval::max(interval2));
    return Interval(min, max);
  }

  /**
   * @brief Define scalar addition as the addition of a scalar to the bounds.
   * @{
   */
  friend Interval operator+(const Interval& interval, const T& scalar) {
    const auto min = (Interval::min(interval) + scalar);
    const auto max = (Interval::max(interval) + scalar);
    return Interval(min, max);
  }

  friend Interval operator-(const Interval& interval, const T& scalar) {
    return (interval + -scalar);
  }
  /** @} */

  /**
   * @brief Test whether the two intervals have bounds within epsilon of each
   * other.
   *
   * @note If both intervals are empty, this returns true. If only one is empty,
   * this returns false.
   */
  static bool approx_eq(const Interval& interval1, const Interval& interval2,
                        const T& epsilon);

  /**
   * @brief The minimum bound of the interval.
   *
   * @param interval The interval to retrieve the min bound from.
   *
   * @return The minimum bound.
   */
  static T min(const Interval& interval);

  /**
   * @brief The maximum bound of the interval.
   *
   * @param interval The interval to retrieve the max bound from.
   *
   * @return The maximum bound.
   */
  static T max(const Interval& interval);

  /**
   * @brief Utility to retrieve both interval bounds at once.
   *
   * @param interval The interval to retrieve the bounds from.
   *
   * @return A tuple of the bounds.
   */
  static const std::tuple<T, T>& bounds(const Interval& interval);

  /**
   * @brief Return the length of the interval.
   *
   * @note For empty or invalid intervals, NaN is returned. See Interval::empty
   * for note on distinction between length zero and the emptiness property.
   *
   * @param interval The interval to check.
   *
   * @return The length of the interval.
   */
  static T length(const Interval& interval);

  /**
   * @brief Utility for checking whether an interval has zero length.
   *
   * @note For empty or invalid intervals, false is returned. See
   * Interval::empty for note on distinction between length zero and the
   * emptiness property.
   *
   * @param interval The interval to check.
   *
   * @return True if the interval has zero length; otherwise false.
   */
  static bool zero_length(const Interval& interval);

  /** @brief Get the interval value nearest to the given value. */
  static T nearest_value(const Interval& interval, const T& value);

  /**
   * @brief Whether the interval is empty or not.
   *
   * @note Emptiness refers to whether the interval contains any points and is
   * thus a distinct property from length: an interval is non-empty if contains
   * only a single point even though its length in that case is zero.
   *
   * @param interval The interval to test for emptiness.
   *
   * @return True if the interval is empty; otherwise false.
   */
  static bool empty(const Interval& interval);

  /**
   * @brief Compute and return the center of the given interval.
   *
   * @note If the interval is empty, NaN is returned.
   */
  static T center(const Interval& interval);

  /**
   * @brief Test for whether a given interval contains a given value.
   *
   * @param interval The interval.
   * @param value The value.
   *
   * @return True if 'interval' contains 'value'; otherwise false.
   */
  static bool contains(const Interval& interval, const T& value);

  /**
   * @brief Test for whether 'interval1' \subseteq 'interval2'
   *
   * @param interval1 The first interval.
   * @param interval2 The second interval.
   *
   * @return True if interval1 \subseteq interval2; otherwise false.
   */
  static bool is_subset_eq(const Interval& interval1,
                           const Interval& interval2);

  /**
   * @brief Compute the intersection of two intervals as a new interval.
   *
   * @param interval1 The first interval.
   * @param interval2 The second interval.
   *
   * @return The newly constructed interval intersection.
   */
  static Interval intersect(const Interval& interval1,
                            const Interval& interval2);

  /**
   * @brief Test whether two intervals overlap.
   *
   * @note If either interval is empty, this returns true.
   */
  static bool overlap(const Interval& interval1, const Interval& interval2);

  /**
   * @brief Project a scalar 'val' onto 'interval'.
   *
   * @param interval The interval to project onto.
   * @param val The value to project.
   *
   * @return If 'val' \in 'interval', return 'val'; otherwise return the nearer
   * interval bound.
   */
  static T project_to_interval(const Interval& interval, const T& val);

  /**
   * @brief Project a value 'val' from one range onto another.
   *
   * @tparam T The type of numbers being projected.
   * @param val The value being projected.
   * @param from_range Interval representing the originating range.
   * @param to_range Interval representing the target range.
   *
   * @return The value in the target range with the same proportional offset as
   * 'val' in the originating range.
   */
  static T project_to_range(const T& val, const Interval& from_range,
                            const Interval& to_range);

  /**
   * @brief Compute the scaling along the interval that corresponds to 'val'
   *
   * @returns s \in [0, 1] for val \in interval, or s < 0 or s > 1 for val < min
   * or val > max
   */
  static T scale_to_interval(const Interval& interval, const T& val);

  /**
   * @brief Compute the convex hull of two intervals as a new interval.
   *
   * @note For intervals that overlap, this is equivalent to a union.
   *
   * @param interval1 The first interval.
   * @param interval2 The second interval.
   *
   * @return The newly constructed interval hull.
   */
  static Interval convex_hull(const Interval& interval1,
                              const Interval& interval2);

  /**
   * @brief Compute the interval that exactly contains the input intervals.
   *
   * This method implements interval union. It computes and returns a convex
   * hull of two intervals iff they intersect. If the two intervals do not
   * intersect, there is no interval that exactly contains the input intervals,
   * so an invalid interval is returned.
   *
   * @param interval1 The first interval to merge.
   * @param interval2 The second interval to merge.
   *
   * @return The merged interval.
   */
  static boost::optional<Interval> merge(const Interval& interval1,
                                         const Interval& interval2);

  /**
   * @brief Construct and return an origin-centered symmetric interval of the
   * given length.
   */
  static Interval origin_centered_symmetric(const T& length);

  /**
   * @brief Compute and return [min - margin, max + margin].
   *
   * @pre The margin will not result in an invalid interval, i.e., one in which
   * min > max
   */
  static Interval grow(const Interval& interval, const T& margin);

  /**
   * @brief Factory method to build the set of affinely extended reals.
   *
   * @return [-\infty, +\infty]
   */
  static Interval affinely_extended_reals();

  /**
   * @brief Factory method to build the set of non-negative affinely extended
   * reals.
   *
   * @return [0, +\infty]
   */
  static Interval nonnegative_affinely_extended_reals();

  /**
   * @brief Factory method to build the set of non-negative affinely extended
   * reals.
   *
   * @return [-\infty, 0]
   */
  static Interval nonpositive_affinely_extended_reals();

  /**
   * @brief Factory method to build the set of machine representable reals.
   *
   * @return [-DBL_MAX, DBL_MAX]
   */
  static Interval max_representable_reals();

  /**
   * @brief Factory method to build the set of machine representatble
   * non-negative reals.
   *
   * @return [0, DBL_MAX]
   */
  static Interval non_negative_reals();

  /**
   * @brief Factory method to build the set of machine representatble
   * non-positive reals.
   *
   * @return [-DBL_MAX, 0]
   */
  static Interval non_positive_reals();

  /**
   * @brief Constructor: initialize an empty interval with members set to NaN.
   */
  Interval();

  /**
   * @brief Constructor: specify exact interval bounds.
   *
   * @note An empty interval is specified by setting both bounds to NaN.
   *
   * @note An exception is thrown if the specified bounds are invalid.
   *
   * @post Construction is successful iff the interval is valid.
   *
   * @param minimum The minimum bound.
   * @param maximum The maximum bound.
   */
  Interval(const T& minimum, const T& maximum);

 protected:
  static constexpr auto NaN = std::numeric_limits<T>::quiet_NaN();

  /** @brief Scalar interval minimum bounds: min, max. */
  std::tuple<T, T> bounds_;

  /** @brief Interval constructor overload. */
  Interval(const std::tuple<T, T>& bounds);

  /** @brief What the bounds would be if the intervals were intersected. */
  static std::tuple<T, T> get_intersection_bounds(const Interval& interval1,
                                                  const Interval& interval2);

  /** @brief Verify that the bounds are valid for an interval. */
  static bool bounds_valid(const std::tuple<T, T>& bounds);
};  // class Interval

//------------------------------------------------------------------------------

typedef Interval<double> Interval_d;
typedef Interval<float> Interval_f;

//------------------------------------------------------------------------------

template <typename T>
constexpr T Interval<T>::MIN;

template <typename T>
constexpr T Interval<T>::MAX;

template <typename T>
constexpr T Interval<T>::SUPREMUM;

template <typename T>
constexpr T Interval<T>::INFIMUM;

//------------------------------------------------------------------------------

template <typename T>
bool Interval<T>::approx_eq(const Interval& interval1,
                            const Interval& interval2, const T& epsilon) {
  const auto i1_empty = Interval::empty(interval1);
  const auto i2_empty = Interval::empty(interval2);
  if (i1_empty && i2_empty) {
    return true;
  }
  if (exclusiveOr(i1_empty, i2_empty)) {
    return false;
  }

  const auto min1 = Interval::min(interval1);
  const auto min2 = Interval::min(interval2);
  const auto mins_equal = approxEq(min1, min2, epsilon);

  const auto max1 = Interval::max(interval1);
  const auto max2 = Interval::max(interval2);
  const auto maxs_equal = approxEq(max1, max2, epsilon);

  return (mins_equal && maxs_equal);
}

//------------------------------------------------------------------------------

template <typename T>
T Interval<T>::min(const Interval& interval) {
  return std::get<0>(interval.bounds_);
}

//------------------------------------------------------------------------------

template <typename T>
T Interval<T>::max(const Interval& interval) {
  return std::get<1>(interval.bounds_);
}

//------------------------------------------------------------------------------

template <typename T>
const std::tuple<T, T>& Interval<T>::bounds(const Interval& interval) {
  return interval.bounds_;
}

//------------------------------------------------------------------------------

template <typename T>
bool Interval<T>::zero_length(const Interval& interval) {
  const auto is_not_empty = !Interval::empty(interval);
  const auto length_is_zero =
      (Interval::min(interval) == Interval::max(interval));
  return (is_not_empty && length_is_zero);
}

//------------------------------------------------------------------------------

template <typename T>
T Interval<T>::nearest_value(const Interval& interval, const T& value) {
  if (std::isnan(value)) {
    return value;
  }
  if (std::isinf(value)) {
    return (std::signbit(value) ? Interval::min(interval)
                                : Interval::max(interval));
  }
  if (Interval::contains(interval, value)) {
    return value;
  }

  const auto distance_from_min = std::abs(Interval::min(interval) - value);
  const auto distance_from_max = std::abs(value - Interval::max(interval));

  const auto min_is_nearer = (distance_from_min < distance_from_max);

  return (min_is_nearer ? Interval::min(interval) : Interval::max(interval));
}

//------------------------------------------------------------------------------

template <typename T>
bool Interval<T>::empty(const Interval& interval) {
  return (std::isnan(Interval::min(interval)) &&
          std::isnan(Interval::max(interval)));
}

//------------------------------------------------------------------------------

template <typename T>
T Interval<T>::center(const Interval& interval) {
  const auto mid_offset = (T(0.5) * Interval::length(interval));
  return (Interval::min(interval) + mid_offset);
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> Interval<T>::affinely_extended_reals() {
  return Interval(INFIMUM, SUPREMUM);
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> Interval<T>::nonnegative_affinely_extended_reals() {
  return Interval(T(0), SUPREMUM);
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> Interval<T>::nonpositive_affinely_extended_reals() {
  return Interval(INFIMUM, T(0));
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> Interval<T>::max_representable_reals() {
  return Interval(MIN, MAX);
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> Interval<T>::non_negative_reals() {
  return Interval(T(0), MAX);
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> Interval<T>::non_positive_reals() {
  return Interval(MIN, T(0));
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T>::Interval() : bounds_(std::make_tuple(NaN, NaN)) {}

//------------------------------------------------------------------------------

template <typename T>
Interval<T>::Interval(const T& minimum, const T& maximum)
    : bounds_(std::make_tuple(minimum, maximum)) {
  if (!Interval::bounds_valid(bounds_)) {
    std::stringstream ss;
    ss << "Attempted to construct an invalid interval: " << *this;
    throw std::runtime_error(ss.str());
  }
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T>::Interval(const std::tuple<T, T>& bounds)
    : Interval(std::get<0>(bounds), std::get<1>(bounds)) {}

//------------------------------------------------------------------------------

template <typename T>
bool Interval<T>::bounds_valid(const std::tuple<T, T>& bounds) {
  const auto min = std::get<0>(bounds);
  const auto max = std::get<1>(bounds);
  const auto is_empty = (std::isnan(min) && std::isnan(max));
  const auto is_ordered = (min <= max);
  return (is_empty || is_ordered);
}

//------------------------------------------------------------------------------

template <typename T>
bool Interval<T>::is_subset_eq(const Interval& interval1,
                               const Interval& interval2) {
  const auto lower_contained =
      (Interval::min(interval1) >= Interval::min(interval2));
  const auto upper_contained =
      (Interval::max(interval1) <= Interval::max(interval2));
  return (lower_contained && upper_contained);
}

//------------------------------------------------------------------------------

template <typename T>
bool Interval<T>::contains(const Interval& interval, const T& value) {
  return ((value >= Interval::min(interval)) &&
          (value <= Interval::max(interval)));
}

//------------------------------------------------------------------------------

template <typename T>
T Interval<T>::length(const Interval& interval) {
  return (Interval::max(interval) - Interval::min(interval));
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> Interval<T>::convex_hull(const Interval& interval1,
                                     const Interval& interval2) {
  // If either interval is empty, the hull is trivially the other interval.
  if (Interval::empty(interval1)) {
    return interval2;
  }
  if (Interval::empty(interval2)) {
    return interval1;
  }

  // Otherwise, the hull contains both.
  return Interval(std::min(Interval::min(interval1), Interval::min(interval2)),
                  std::max(Interval::max(interval1), Interval::max(interval2)));
}

//------------------------------------------------------------------------------

template <typename T>
boost::optional<Interval<T>> Interval<T>::merge(const Interval& interval1,
                                                const Interval& interval2) {
  // If the intervals do not overlap, they cannot be merged.
  if (!Interval::overlap(interval1, interval2)) {
    return boost::none;
  }

  // Done.
  return Interval::convex_hull(interval1, interval2);
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> Interval<T>::origin_centered_symmetric(const T& length) {
  const auto half_length = (static_cast<T>(0.5) * length);
  return Interval<T>(-half_length, half_length);
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> Interval<T>::grow(const Interval& interval, const T& margin) {
  const T min = (Interval::min(interval) - margin);
  const T max = (Interval::max(interval) + margin);
  return Interval(min, max);
}

//------------------------------------------------------------------------------

template <typename T>
Interval<T> Interval<T>::intersect(const Interval& interval1,
                                   const Interval& interval2) {
  // Construct intersection assuming an intersection exists.
  try {
    return Interval(Interval::get_intersection_bounds(interval1, interval2));
  } catch (...) {
  }

  // Otherwise, the intersection is empty.
  return Interval();
}

//------------------------------------------------------------------------------

template <typename T>
std::tuple<T, T> Interval<T>::get_intersection_bounds(
    const Interval& interval1, const Interval& interval2) {
  const auto either_empty =
      (Interval::empty(interval1) || Interval::empty(interval2));

  const auto min = std::max(Interval::min(interval1), Interval::min(interval2));
  const auto max = std::min(Interval::max(interval1), Interval::max(interval2));

  return (either_empty ? std::make_tuple(NaN, NaN) : std::make_tuple(min, max));
}

//------------------------------------------------------------------------------

template <typename T>
bool Interval<T>::overlap(const Interval& interval1,
                          const Interval& interval2) {
  const auto bounds = Interval::get_intersection_bounds(interval1, interval2);
  return Interval::bounds_valid(bounds);
}

//------------------------------------------------------------------------------

template <typename T>
T Interval<T>::project_to_interval(const Interval& interval, const T& val) {
  return std::min(Interval::max(interval),
                  std::max(val, Interval::min(interval)));
}

//------------------------------------------------------------------------------

template <typename T>
T Interval<T>::project_to_range(const T& val, const Interval<T>& from_range,
                                const Interval<T>& to_range) {
  const auto val_offset = (val - Interval<T>::min(from_range));
  const auto s = (val_offset / Interval<T>::length(from_range));
  return (Interval<T>::min(to_range) + s * Interval<T>::length(to_range));
}

//------------------------------------------------------------------------------

template <typename T>
T Interval<T>::scale_to_interval(const Interval& interval, const T& val) {
  if (std::isnan(val) || Interval::empty(interval)) {
    return NaN;
  }

  const auto l = Interval::length(interval);
  const auto offset = (val - Interval::min(interval));

  return (offset / l);
}

//------------------------------------------------------------------------------

}  // namespace maeve_automation_core
