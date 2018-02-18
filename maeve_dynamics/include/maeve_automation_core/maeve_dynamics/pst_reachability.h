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
 *
 * @note Reachability computations assume strictly non-negative minimum speeds
 * are permitted.
 */
class PST_Reachability {
 public:
  /**
   * @brief Stream overload for PST reachability objects.
   *
   * @note The serialization is JSON compatible.
   *
   * @param os The output stream.
   * @param reachability The PST reachability object.
   *
   * @return The output stream with the object serialized.
   */
  friend std::ostream& operator<<(std::ostream& os,
                                  const PST_Reachability& reachability);

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

  /**
   * @brief Compute a connector that connects two points with the desired target
   * speed.
   *
   * @param I_i The interval of available initial speeds.
   * @param p1 The initial point in PT space.
   * @param p2 The terminal point in PT space.
   * @param target_speed Desired target speed.
   * @param constraints The dynamic constraints describing 1st and 2nd order
   * bounds.
   *
   * @return A nullable object of either the connector object or boost::none.
   */
  static boost::optional<PST_Connector> targetTerminalSpeed(
      const Interval& I_i, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
      const double target_speed, const IntervalConstraints<2>& constraints);

  /**
   * @brief Compute the reachability from 'p1' to 'p2' under the given
   * constraints.
   *
   * @param I_i The interval of speeds available at p1.
   * @param p1  The initial path-time point.
   * @param p2  The terminal path-time point.
   * @param constraints The set of dynamic constraints.
   *
   * @return The reachability objects, or a disengaged optional.
   */
  static boost::optional<PST_Reachability> compute(
      const Interval& I_i, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
      const IntervalConstraints<2>& constraints);

 private:
  /**
   * @brief Constructor: explicit initialization.
   *
   * @param min_terminal The connecting trajectory with min terminal speed.
   * @param max_terminal The connecting trajectory with max terminal speed.
   * @param constraints  The dynamic constraints reachability should satisfy.
   */
  PST_Reachability(const PST_Connector& min_terminal,
                   const PST_Connector& max_terminal,
                   const IntervalConstraints<2>& constraints);

  /**
   * @brief Factory method for constructing PST_Reachability objects that
   * swallows exceptions.
   *
   * @param min_terminal The connecting trajectory with min terminal speed.
   * @param max_terminal The connecting trajectory with max terminal speed.
   * @param constraints  The dynamic constraints reachability should satisfy.
   *
   * @return An optional that is disengaged on exception, or that contains the
   * object.
   */
  static boost::optional<PST_Reachability> noExceptionConstructor(
      const PST_Connector& min_terminal, const PST_Connector& max_terminal,
      const IntervalConstraints<2>& constraints) noexcept;

  /**
   * @brief Check whether the speeds of the interior along the connector satisfy
   * constraints.
   *
   *
   * @param connector The connector being checked.
   * @param constraints The constraints to check againts.
   *
   * @return True if the speeds bounds are nowhere violated on the interior of
   * the connector; otherwise false.
   */
  static bool validInteriorSpeeds(const PST_Connector& connector,
                                  const IntervalConstraints<2>& constraints);

  /**
   * @brief Given an LP connector, verify that it satisfies constraints, or
   * attempt to compute a PLP connector that does.
   *
   * @param LP The LP connector.
   * @param I_dt The interval of valid speeds.
   * @param I_ddt The interval of valid accelerations.
   *
   * @return A nullable object of either the connector or boost::none.
   */
  static boost::optional<PST_Connector> LPorPLP(const PST_Connector& LP,
                                                const Interval& I_dt,
                                                const Interval& I_ddt);

  /** @brief The PST connector that achieves minimum terminal speed. */
  PST_Connector min_terminal_;
  /** @brief the PST connector that achieves maximum terminal speed. */
  PST_Connector max_terminal_;
};  // class PST_Reachability

if (const auto reachability =
        PST_Reachability::compute(initial_speeds, p1, p2, constraints)) {
  std::cout << *reachability << std::endl;
}

}  // namespace maeve_automation_core
