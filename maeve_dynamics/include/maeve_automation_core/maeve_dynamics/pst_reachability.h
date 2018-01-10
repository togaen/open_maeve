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
 * @brief This class contains function for computing reachability under second
 * order constraints.
 */
class PST_Reachability {
 public:
  /**
   * @brief Compute reachability.
   *
   * @param p1 The initial point in PT space.
   * @param v_i The initial speed interval for p1.
   * @param p2 The terminal point in PT space.
   * @param constraints The dynamic constraints describing 1st and 2nd order
   * bounds.
   *
   * @return A nullable object of either the reachability object or boost::none.
   */
  static boost::optional<PST_Reachability> compute(
      const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  /**
   * @brief The interval of reachable speeds.
   *
   * @return An interval object of reachable speeds.
   */
  static Interval reachableInterval(const PST_Reachability& reachability);

  /**
   * @brief Get a reference to the connector of min terminal speed.
   *
   * @param reachability The reachability object.
   *
   * @return A const reference to the min connector.
   */
  static const PST_Connector& minConnector(
      const PST_Reachability& reachability);

  /**
   * @brief Get a reference to the connector of max terminal speed.
   *
   * @param reachability The reachability object.
   *
   * @return A const reference to the max connector.
   */
  static const PST_Connector& maxConnector(
      const PST_Reachability& reachability);

 private:
  /**
   * @brief Constructor: explicit initialization.
   *
   * @param min_terminal The connecting trajectory with min terminal speed.
   * @param max_terminal The connecting trajectory with max terminal speed.
   */
  PST_Reachability(PST_Connector&& min_terminal, PST_Connector&& max_terminal);

  /**
   * @brief Enumerate the types of trajectories that determine reachability in
   * PST space.
   */
  enum class Type {
    // Type I: P+LP+ Trajectories with an initial acceleration, a constant
    // speed portion, and a terminal acceleration
    I,
    // Type II: P+LP- Trajectories with an initial acceleration, a constant
    // speed portion, and a terminal deceleration.
    II,
    // Type III: P-LP+ Trajectories with an initial deceleration, a constant
    // speed portion, and a terminal acceleration.
    III,
    // Type IV: P-LP- Trajectories with an initial deceleration, a constant
    // speed portion, and a terminal deceleration.
    IV,
    // Type V: P+P+ Trajectories with an initial acceleration followed by a
    // terminal deceleration.
    V,
    // Type VI: P+P- Trajectories with an initial acceleration followed by a
    // terminal deceleration.
    VI,
    // Type VII: P-P+ Trajectories with an initial deceleration followed by a
    // terminal acceleration.
    VII,
    // Type VIII: P-P- Trajectories with an initial deceleration followed by a
    // terminal deceleration.
    VIII
  };

  /**
   * @brief Compute a connector for various types of trajectories for a fixed
   * initial speed.
   *
   * @tparam T The connector type.
   *
   * @param p1 The initial point in PT space.
   * @param v_i The initial speed for p1.
   * @param p2 The terminal point in PT space.
   * @param constraints The dynamic constraints describing 1st and 2nd order
   * bounds.
   *
   * @return A nullable object of either the connector object or boost::none.
   */
  template <Type T>
  boost::optional<PST_Connector> connector(
      const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  /**
   * @brief Compute a connector for various types of trajectories for a given
   * initial speed interval.
   *
   * @tparam T The connector type.
   *
   * @param p1 The initial point in PT space.
   * @param V_i The initial speed interval for p1.
   * @param p2 The temrinal point in PT space.
   * @param constraints The dynamic constraints describing 1st and 2nd order
   * bounds.
   *
   * @return A nullable object of either the connector object or boost::none.
   */
  template <Type T>
  boost::optional<PST_Connector> connector(
      const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  /**
   * @brief Compute a connector that connects two points with the max temrinal
   * speed.
   *
   * This methods computes a special connector of the form PLP+, where the
   * initial parabolic function is of length zero, and a linear segment extends
   * from p1 and becomes tangent to the terminal P+ curve. If no such feasible
   * connector exists, a null object is returned.
   *
   * @param p1 The initial point in PT space.
   * @param p2 The terminal point in PT space.
   * @param constraints The dynamic constraints describing 1st and 2nd order
   * bounds.
   *
   * @return A nullable object of either the connector object or boost::none.
   */
  boost::optional<PST_Connector> maxTerminalSpeed(
      const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  /**
   * @brief Compute a connector that connects two points with the min temrinal
   * speed.
   *
   * This methods computes a special connector of the form PLP-, where the
   * initial parabolic function is of length zero, and a linear segment extends
   * from p1 and becomes tangent to the terminal P- curve. If no such feasible
   * connector exists, a null object is returned.
   *
   * @param p1 The initial point in PT space.
   * @param p2 The terminal point in PT space.
   * @param constraints The dynamic constraints describing 1st and 2nd order
   * bounds.
   *
   * @return A nullable object of either the connector object or boost::none.
   */
  boost::optional<PST_Connector> minTerminalSpeed(
      const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

  /** @brief The PST connector that achieves minimum terminal speed. */
  PST_Connector min_terminal_;
  /** @brief the PST connector that achieves maximum terminal speed. */
  PST_Connector max_terminal_;
};  // class PST_Reachability

/**
 * Specializations for fixed initial speed connectors.
 * @{
 */
template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::I>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::II>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::III>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::IV>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::V>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VI>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VII>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VIII>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);
/** @} */

/**
 * Specializations for connectors with initial speed intervals.
 * @{
 */
template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::I>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::II>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::III>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::IV>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::V>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VI>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VII>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VIII>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints);
/** @} */
}  // namespace maeve_automation_core
