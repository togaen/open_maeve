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
#include <stdexcept>

#include "open_maeve/maeve_geometry/interval.h"

namespace open_maeve {
/**
 * @brief This class describes interval bounds for time and arc length s.
 *
 * @tparam Order The desired order up to which to specify arc length bounds.
 * @tparam T     The scalar type (float or double)
 *
 * The bounds for arc length are specified to a given order. All values within a
 * given interval are considered feasible, and all values outside the interval
 * infeasible.
 */
template <unsigned int Order, typename T>
class IntervalConstraints {
 public:
  /**
   * @brief Stream overload for interval constraints.
   *
   * @note The serialization is JSON compatible.
   *
   * @param os The output stream.
   * @param constraints The constraint object.
   *
   * @return The output stream with the constraint object serialized.
   */
  friend std::ostream& operator<<(std::ostream& os,
                                  const IntervalConstraints& constraints) {
    os << "{\"time\": " << constraints.t_bounds_ << ", \"arc_length\": {";
    for (auto i = 0; i < Order; ++i) {
      os << "{\"order\": " << i << ", \"bounds\": " << constraints.s_bounds_[i]
         << "}, ";
    }
    return os << "{\"order\": " << Order
              << ", \"bounds\": " << constraints.s_bounds_[Order] << "}}";
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
   * @param epsilon_bounds Padding values for approximate floating point
   * comparisons.
   * @param t_bounds Feasible time interval.
   * @param s_bounds Array of feasible s intervals indexed by order.
   */
  IntervalConstraints(const Interval<T>& epsilon_bounds,
                      const Interval<T>& t_bounds,
                      const std::array<Interval<T>, Order + 1>& s_bounds);

  /**
   * @brief Create an object where all intervals are [0, 0]
   *
   * @note This is useful when delayed initialization is needed.
   */
  static IntervalConstraints create_zeroed_set();

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
   * @brief Get a reference to the epsilon bounds of this constraint set.
   *
   * @param constraints The constraint set.
   *
   * @return A reference to the epsilon padding values.
   */
  static const Interval<T>& epsilon(const IntervalConstraints& constraints);

  /**
   * @brief Get a reference to the temporal bounds of this constraint set.
   *
   * @param constraints The constraint set.
   *
   * @return A reference to the temporal bounds.
   */
  static const Interval<T>& boundsT(const IntervalConstraints& constraints);

  /**
   * @brief Get a reference to dynamic bounds of a given order.
   *
   * @tparam QueryOrder The order being queried.
   *
   * @param constraints The constraint set.
   *
   * @return A reference to the dynamic bounds of the given order.
   */
  template <unsigned int QueryOrder>
  static const Interval<T>& boundsS(const IntervalConstraints& constraints);

 private:
  /**
   * @brief A constraint set is satisfiable iff all intervals are non-emtpy.
   *
   * @param constraints The constraint set to check for satisfiability.
   *
   * @return True if the constraint set is satisfiable; otherwise false.
   */
  static bool satisfiable(const IntervalConstraints& constraints);

  /** @brief Zero-centered padding range for floating point error. */
  Interval<T> epsilon_bounds_;

  /** @brief Interval of feasible time values. */
  Interval<T> t_bounds_;

  /** @brief
   * Intervals for feasible arc length (s) values indexed by order.
   *
   * In this scheme, 0th order is arch length, 1st order is first time
   * derivative, 2nd order is second time derivative, etc.
   * */
  std::array<Interval<T>, Order + 1> s_bounds_;
};  // class DynamicConstraints

//------------------------------------------------------------------------------

template <unsigned int Order, typename T>
IntervalConstraints<Order, T>::IntervalConstraints(
    const Interval<T>& epsilon_bounds, const Interval<T>& t_bounds,
    const std::array<Interval<T>, Order + 1>& s_bounds)
    : epsilon_bounds_(epsilon_bounds),
      t_bounds_(t_bounds),
      s_bounds_(s_bounds) {
  if (!IntervalConstraints::satisfiable(*this)) {
    std::stringstream ss;
    ss << "Attempted to construct a constraint object that is not satisfiable: "
       << *this;
    throw std::runtime_error(ss.str());
  }
}

//------------------------------------------------------------------------------

template <unsigned int Order, typename T>
IntervalConstraints<Order, T>
IntervalConstraints<Order, T>::create_zeroed_set() {
  const Interval<T> zero_interval(T(0), T(0));
  constexpr auto N = (Order + 1);
  std::array<Interval<T>, N> bounds_set;
  for (auto i = 0; i < N; ++i) {
    bounds_set[i] = zero_interval;
  }
  return IntervalConstraints(zero_interval, zero_interval, bounds_set);
}

//------------------------------------------------------------------------------

template <unsigned int Order, typename T>
bool IntervalConstraints<Order, T>::satisfiable(
    const IntervalConstraints& constraints) {
  const auto eps_non_empty = !Interval<T>::empty(constraints.epsilon_bounds_);
  const auto t_non_empty = !Interval<T>::empty(constraints.t_bounds_);
  const auto s_non_empty = std::all_of(std::begin(constraints.s_bounds_),
                                       std::end(constraints.s_bounds_),
                                       [&](const Interval<T>& interval) {
                                         return !Interval<T>::empty(interval);
                                       });
  return (eps_non_empty && t_non_empty && s_non_empty);
}

//------------------------------------------------------------------------------

template <unsigned int Order, typename T>
IntervalConstraints<Order, T> IntervalConstraints<Order, T>::intersect(
    const IntervalConstraints& constraints1,
    const IntervalConstraints& constraints2) {
  const auto eps_bounds = Interval<T>::intersect(constraints1.epsilon_bounds_,
                                                 constraints2.epsilon_bounds_);
  const auto t_bounds =
      Interval<T>::intersect(constraints1.t_bounds_, constraints2.t_bounds_);
  auto s_bounds = constraints1.s_bounds_;
  for (auto i = 0; i <= Order; ++i) {
    s_bounds[i] =
        Interval<T>::intersect(s_bounds[i], constraints2.s_bounds_[i]);
  }
  return IntervalConstraints(eps_bounds, t_bounds, s_bounds);
}

//------------------------------------------------------------------------------

template <unsigned int Order, typename T>
const Interval<T>& IntervalConstraints<Order, T>::epsilon(
    const IntervalConstraints& constraints) {
  return constraints.epsilon_bounds_;
}

//------------------------------------------------------------------------------

template <unsigned int Order, typename T>
const Interval<T>& IntervalConstraints<Order, T>::boundsT(
    const IntervalConstraints<Order, T>& constraints) {
  return constraints.t_bounds_;
}

//------------------------------------------------------------------------------

template <unsigned int Order, typename T>
template <unsigned int QueryOrder>
const Interval<T>& IntervalConstraints<Order, T>::boundsS(
    const IntervalConstraints<Order, T>& constraints) {
  static_assert(
      (QueryOrder <= Order),
      "This constraint set does not define bounds of the specified order.");
  return constraints.s_bounds_[QueryOrder];
}

//------------------------------------------------------------------------------

}  // namespace open_maeve
