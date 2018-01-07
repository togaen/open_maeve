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

#include <Eigen/Core>
#include <boost/optional.hpp>

#include "maeve_automation_core/maeve_dynamics/interval_constraints.h"
#include "maeve_automation_core/maeve_dynamics/pst_connector.h"

namespace maeve_automation_core {
/**
 * @brief This namespace contains function for computing reachability under
 * second order constraints.
 */
namespace pst {
/**
 * @brief Enumerate the types of trajectories that determine reachability in PST
 * space.
 */
enum class Type {
  // Type I: P^+LP^+ Trajectories with an initial acceleration, a constant speed
  // portion, and a terminal acceleration
  I,
  // Type II: P^+LP^- Trajectories with an initial acceleration, a constant
  // speed portion, and a terminal deceleration.
  II,
  // Type III: P^-LP^+ Trajectories with an initial deceleration, a constant
  // speed portion, and a terminal acceleration.
  III,
  // Type IV: P^-LP^- Trajectories with an initial deceleration, a constant
  // speed portion, and a terminal deceleration.
  IV,
  // Type V: P^+P^+ Trajectories with an initial acceleration followed by a
  // terminal deceleration.
  V,
  // Type VI: P^+P^- Trajectories with an initial acceleration followed by a
  // terminal deceleration.
  VI,
  // Type VII: P^-P^+ Trajectories with an initial deceleration followed by a
  // terminal acceleration.
  VII,
  // Type VIII: P^-P^- Trajectories with an initial deceleration followed by a
  // terminal deceleration.
  VIII
};

/**
 * @brief Compute reachability.
 *
 * @tparam T The reachability type.
 *
 * @param p1 The initial point in PT space.
 * @param v_i The initial speed interval for p1.
 * @param p2 The terminal point in PT space.
 * @param constraints The dynamic constraints describing 1st and 2nd order
 * bounds.
 *
 * @return A nullable object of either the connecting trajectory for
 * boost::none.
 */
template <Type T>
boost::optional<PST_Connector> reachability(
    const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

/**
 * This group defines template specializations for each connector type.
 * @{
 */
template <>
boost::optional<PST_Connector> reachability<Type::I>(
    const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector> reachability<Type::II>(
    const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector> reachability<Type::III>(
    const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector> reachability<Type::IV>(
    const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector> reachability<Type::V>(
    const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector> reachability<Type::VI>(
    const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector> reachability<Type::VII>(
    const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector> reachability<Type::VIII>(
    const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);
/** @} */
}  // namespace pst
}  // namespace maeve_automation_core
