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
#include <array>
#include <iostream>

#include "maeve_automation_core/maeve_geometry/interval.h"

namespace maeve_automation_core {
/**
 * @brief This class describes interval bounds for time and arc length s.
 *
 * @tparam Order The desired order up to which to specify arc length bounds.
 *
 * The bounds for arc length are specified to a given order. All values within a
 * given interval are considered feasible, and all values outside the interval
 * infeasible.
 */
template <unsigned int Order>
class IntervalConstraints {
 public:
  /**
   * @brief Stream overload for interval constraints.
   *
   * @param os The output stream.
   * @param constraints The constraint object.
   *
   * @return The output stream with the constraint object serialized.
   */
  friend std::ostream& operator<<(std::ostream& os,
                                  const IntervalConstraints& constraints) {
    os << "{time: " << constraints.t_bounds_ << ", arc length: {";
    for (auto i = 0; i < Order; ++i) {
      os << "(order: " << i << ", bounds: " << constraints.s_bounds_[i]
         << "), ";
    }
    return os << "(order: " << Order
              << ", bounds: " << constraints.s_bounds_[Order] << ")}";
  }

  /**
   * @brief Equality test for interval constraint sets.
   *
   * Two interval constraint sets are equivalent iff all corresponding member
   * intervals of both sets are equivalent.
   *
   * @param constraints1 The first interval constraint set.
   * @param constraints2 The second interval constraint set.
   *
   * @return True if the constraint sets are equal; otherwise false.
   */
  friend bool operator==(const IntervalConstraints& constraints1,
                         const IntervalConstraints& constraints2) {
    const auto t_eq = (constraints1.t_bounds_ == constraints2.t_bounds_);
    auto s_eq = true;
    for (auto i = 0; i <= Order; ++i) {
      s_eq = (s_eq && (constraints1.s_bounds_[i] == constraints2.s_bounds_[i]));
    }
    return t_eq && s_eq;
  }

  /**
   * @brief Inequality test for interval constraint sets.
   *
   * Two interval constraint sets are unequal iff they are not equal. See
   * IntervalConstraints<Order>::operator==.
   *
   * @param constraints1 The first interval constraint set.
   * @param constraints2 The second interval constraint set.
   *
   * @return True if the constraint sets are not equal; otherwise false.
   */
  friend bool operator!=(const IntervalConstraints& constraints1,
                         const IntervalConstraints& constraints2) {
    return !(constraints1 == constraints2);
  }

  /**
   * @brief Constructor: Build and initialize constraint object explicitly.
   *
   * @param t_bounds Feasible time interval.
   * @param s_bounds Array of feasible s intervals indexed by order.
   */
  IntervalConstraints(Interval&& t_bounds,
                      std::array<Interval, Order + 1>&& s_bounds);

  /**
   * @brief A constraint set is valid iff all intervals are valid.
   *
   * @note Empty interval bounds are considered valid. That means it is possible
   * that a valid constraint set cannot be satisfied.
   *
   * @param constraints The constraint set to check for validity.
   *
   * @return True if the constraint set is valid; otherwise false.
   */
  static bool valid(const IntervalConstraints& constraints);

  /**
   * @brief A constraint set is satisfiable iff it is valid and all intervals
   * are non-emtpy.
   *
   * @param constraints The constraint set to check for satisfiability.
   *
   * @return True if the constraint set is satisfiable; otherwise false.
   */
  static bool satisfiable(const IntervalConstraints& constraints);

  /**
   * @brief Compute and return the intersection of two constraint objects.
   *
   * The intersection of two constraint objects is a constraint
   * bounds are the intersection of the bounds of each of the argument
   * constraint objects.
   *
   * @param constraints1 The first constraint object to intersect.
   * @param constraints2 The second constraint object to intersect.
   *
   * @return The constraint object whose bounds are equal to the intersection of
   * those of constraint1 and constraint2.
   */
  static IntervalConstraints intersect(const IntervalConstraints& constraints1,
                                       const IntervalConstraints& constraints2);

  /**
   * @brief Whether a give time value is feasible according to this constraint
   * set.
   *
   * @param constraints The constraint set to check against.
   * @param time The time value to test.
   *
   * @return True if 'time' is feasible; otherwise false.
   */
  static bool feasibleT(const IntervalConstraints& constraints,
                        const double time);

  /**
   * @brief Whether a given s value of the given order is feasible according to
   * this constraint set.
   *
   * @note Behavior is undefined if the given order exceeds the order specified
   * by this constraint type.
   *
   * @param constraints The constraint set to check against.
   * @param order The order of the s value.
   * @param s The s value to test.
   *
   * @return True if the s value to the give order is feasible; otherwise false.
   */
  static bool feasibleS(const IntervalConstraints& constraints, const int order,
                        const double s);

 private:
  /**
   * @brief Constructor: default.
   */
  IntervalConstraints() = default;

  /** @brief Interval of feasible time values. */
  Interval t_bounds_;

  /** @brief
   * Intervals for feasible arc length (s) values indexed by order.
   *
   * In this scheme, 0th order is arch length, 1st order is first time
   * derivative, 2nd order is second time derivative, etc.
   * */
  std::array<Interval, Order + 1> s_bounds_;
};  // class DynamicConstraints

template <unsigned int Order>
IntervalConstraints<Order>::IntervalConstraints(
    Interval&& t_bounds, std::array<Interval, Order + 1>&& s_bounds)
    : t_bounds_(std::move(t_bounds)), s_bounds_(std::move(s_bounds)) {}

template <unsigned int Order>
bool IntervalConstraints<Order>::valid(const IntervalConstraints& constraints) {
  const auto t_valid = Interval::valid(constraints.t_bounds_);
  const auto s_valid = std::all_of(
      std::begin(constraints.s_bounds_), std::end(constraints.s_bounds_),
      [&](const Interval& interval) { return Interval::valid(interval); });
  return t_valid && s_valid;
}

template <unsigned int Order>
bool IntervalConstraints<Order>::satisfiable(
    const IntervalConstraints& constraints) {
  const auto is_valid = IntervalConstraints<Order>::valid(constraints);
  const auto t_non_empty = !Interval::empty(constraints.t_bounds_);
  const auto s_non_empty = std::all_of(
      std::begin(constraints.s_bounds_), std::end(constraints.s_bounds_),
      [&](const Interval& interval) { return !Interval::empty(interval); });
  return is_valid && t_non_empty && s_non_empty;
}

template <unsigned int Order>
IntervalConstraints<Order> IntervalConstraints<Order>::intersect(
    const IntervalConstraints& constraints1,
    const IntervalConstraints& constraints2) {
  auto c = IntervalConstraints();
  c.t_bounds_ =
      Interval::intersect(constraints1.t_bounds_, constraints2.t_bounds_);
  for (auto i = 0; i <= Order; ++i) {
    c.s_bounds_[i] = Interval::intersect(constraints1.s_bounds_[i],
                                         constraints2.s_bounds_[i]);
  }
  return c;
}

template <unsigned int Order>
bool IntervalConstraints<Order>::feasibleT(
    const IntervalConstraints& constraints, const double time) {
  return Interval::contains(constraints.t_bounds_, time);
}

template <unsigned int Order>
bool IntervalConstraints<Order>::feasibleS(
    const IntervalConstraints& constraints, const int order, const double s) {
  // Throw an exception if 'order' violates array bounds.
  return Interval::contains(constraints.s_bounds_.at(order), s);
}
}  // namespace maeve_automation_core
