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
#include "maeve_automation_core/maeve_dynamics/pst_reachability.h"

#include <cmath>
#include <limits>
#include <sstream>

#include "maeve_automation_core/maeve_geometry/comparisons.h"

namespace maeve_automation_core {
namespace {
const auto Inf = std::numeric_limits<double>::infinity();
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

std::ostream& operator<<(std::ostream& os,
                         const PST_Reachability& reachability) {
  os << "{\"min_speed_connector\": " << reachability.min_terminal_
     << ", \"max_speed_connector\": " << reachability.max_terminal_ << "}";
  return os;
}

PST_Reachability::PST_Reachability(
    const PST_Connector& min_terminal, const PST_Connector& max_terminal,
    const IntervalConstraints<2, double>& constraints)
    : min_terminal_(min_terminal), max_terminal_(max_terminal) {
  try {
    const auto I_reachable = PST_Reachability::reachableInterval(*this);

    if (!PST_Connector::dynamicallyFeasible(min_terminal, constraints)) {
      std::stringstream ss;
      ss << min_terminal;
      throw std::runtime_error(
          "Attempted to construct reachability object with infeasible "
          "min_terminal connector: " +
          ss.str());
    }

    if (!PST_Connector::dynamicallyFeasible(max_terminal, constraints)) {
      std::stringstream ss;
      ss << max_terminal;
      throw std::runtime_error(
          "Attempted to construct reachibility object with infeasible "
          "max_terminal connector: " +
          ss.str());
    }
  } catch (...) {
    throw std::domain_error(
        "Attempted to construct reachability object with invalid "
        "information.");
  }
}

boost::optional<PST_Reachability> PST_Reachability::noExceptionConstructor(
    const PST_Connector& min_terminal, const PST_Connector& max_terminal,
    const IntervalConstraints<2, double>& constraints) noexcept {
  try {
    return PST_Reachability(min_terminal, max_terminal, constraints);
  } catch (...) {
    return boost::none;
  }
}

Interval<double> PST_Reachability::reachableInterval(
    const PST_Reachability& reachability) {
  const auto& min_connector = PST_Reachability::minConnector(reachability);
  const auto& max_connector = PST_Reachability::maxConnector(reachability);
  const auto min_speed = PST_Connector::terminalSpeed(min_connector);
  const auto max_speed = PST_Connector::terminalSpeed(max_connector);

  return Interval<double>(min_speed, max_speed);
}

const PST_Connector& PST_Reachability::minConnector(
    const PST_Reachability& reachability) {
  return reachability.min_terminal_;
}

const PST_Connector& PST_Reachability::maxConnector(
    const PST_Reachability& reachability) {
  return reachability.max_terminal_;
}

bool PST_Reachability::validInteriorSpeeds(
    const PST_Connector& connector,
    const IntervalConstraints<2, double>& constraints) {
  // Intervals for dynamic bounds.
  const auto& I_dt = IntervalConstraints<2, double>::boundsS<1>(constraints);

  return PST_Connector::boundedInteriorSpeeds(connector, I_dt);
}

boost::optional<PST_Connector> PST_Reachability::LPorPLP(
    const PST_Connector& LP, const Interval<double>& I_dt,
    const Interval<double>& I_ddt, const double epsilon) {
  const auto ddt_min = Interval<double>::min(I_ddt);
  const auto ddt_max = Interval<double>::max(I_ddt);
  const auto terminal_ddt = PST_Connector::terminalAcceleration(LP);

  // If LP is valid, just return it.
  const auto initial_speed = PST_Connector::initialSpeed(LP);
  if (Interval<double>::contains(I_dt, initial_speed)) {
    return LP;
  }

  // Get start and end points for connector.
  Eigen::Vector2d p1, p2;
  std::tie(p1, p2) = PST_Connector::boundaryPoints(LP);

  //
  // LP is invalid, attempt building a PLP.
  //

  // Get valid starting speed.
  const auto dt_valid =
      Interval<double>::project_to_interval(I_dt, initial_speed);

  // Initial acceleration should move toward dt_valid.
  const auto ddt = ((dt_valid < initial_speed) ? ddt_max : ddt_min);

  // Get appropriate terminal acceleration.
  const auto terminal_speed = PST_Connector::terminalSpeed(LP);
  const auto PLP_min = PST_Connector::computePLP(
      p1, dt_valid, ddt, p2, terminal_speed, ddt_min, epsilon);
  const auto PLP_max = PST_Connector::computePLP(
      p1, dt_valid, ddt, p2, terminal_speed, ddt_max, epsilon);

  // Find anything?
  if (PLP_min) {
    return *PLP_min;
  }
  if (PLP_max) {
    return *PLP_max;
  }

  // Done.
  return boost::none;
}

boost::optional<PST_Reachability> PST_Reachability::compute(
    const Interval<double>& I_i, const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2,
    const IntervalConstraints<2, double>& constraints) {
  // Intervals for dynamic bounds.
  const auto& I_dt = IntervalConstraints<2, double>::boundsS<1>(constraints);

  // Extremal speeds.
  const auto dt_max = Interval<double>::max(I_dt);
  const auto dt_min = Interval<double>::min(I_dt);

  // Compute connectors.
  auto min_connector =
      PST_Reachability::targetTerminalSpeed(I_i, p1, p2, dt_min, constraints);
  auto max_connector =
      PST_Reachability::targetTerminalSpeed(I_i, p1, p2, dt_max, constraints);

  // Not reachable.
  if (!min_connector && !max_connector) {
    return boost::none;
  }

  // One reachable.
  if (exclusiveOr(min_connector, max_connector)) {
    if (min_connector) {
      return PST_Reachability::noExceptionConstructor(
          *min_connector, *min_connector, constraints);
    }
    return PST_Reachability::noExceptionConstructor(
        *max_connector, *max_connector, constraints);
  }

  // Both reachable.
  return PST_Reachability::noExceptionConstructor(*min_connector,
                                                  *max_connector, constraints);
}

boost::optional<PST_Connector> PST_Reachability::targetTerminalSpeed(
    const Interval<double>& I_i, const Eigen::Vector2d& p1,
    const Eigen::Vector2d& p2, const double target_speed,
    const IntervalConstraints<2, double>& constraints) {
  const auto& epsilon_interval =
      IntervalConstraints<2, double>::epsilon(constraints);
  const auto epsilon = Interval_d::max(epsilon_interval);

  // Intervals for dynamic bounds.
  const auto& I_ddt = IntervalConstraints<2, double>::boundsS<2>(constraints);

  // Initial extremal speeds.
  const auto initial_dt_max = Interval<double>::max(I_i);
  const auto initial_dt_min = Interval<double>::min(I_i);

  // Extremal accelerations.
  const auto ddt_max = Interval<double>::max(I_ddt);
  const auto ddt_min = Interval<double>::min(I_ddt);

  // Check for LP or PLP connectivity: these will hit target speed exactly,
  // so return if connector found.
  if (const auto LP = PST_Connector::computeLP(p1, p2, target_speed, ddt_min)) {
    if (const auto PLP = LPorPLP(*LP, I_i, I_ddt, epsilon)) {
      return *PLP;
    }
  }
  if (const auto LP = PST_Connector::computeLP(p1, p2, target_speed, ddt_max)) {
    if (const auto PLP = LPorPLP(*LP, I_i, I_ddt, epsilon)) {
      return *PLP;
    }
  }

  // Check for PL_0P or PP: these will not hit target speed exactly, so compute
  // all and choose nearest.
  const auto PL_0P = PST_Connector::computePL_0P(p1, initial_dt_min, ddt_min,
                                                 p2, ddt_max, epsilon);
  const auto PP_max = PST_Connector::computePP(p1, initial_dt_min, ddt_min, p2,
                                               ddt_max, epsilon);
  const auto PP_min = PST_Connector::computePP(p1, initial_dt_max, ddt_max, p2,
                                               ddt_min, epsilon);

  // Choose the connector that gets nearest the target speed.
  const auto PL_0P_delta =
      !PL_0P ? Inf
             : std::abs(target_speed - PST_Connector::terminalSpeed(*PL_0P));
  const auto PP_max_delta =
      !PP_max ? Inf
              : std::abs(target_speed - PST_Connector::terminalSpeed(*PP_max));
  const auto PP_min_delta =
      !PP_min ? Inf
              : std::abs(target_speed - PST_Connector::terminalSpeed(*PP_min));

  // No solution, done.
  if (!std::isfinite(PL_0P_delta) && !std::isfinite(PP_max_delta) &&
      !std::isfinite(PP_min_delta)) {
    return boost::none;
  }

  //
  // There is at least one valid solution.
  //

  // Comparator for finding best target speed.
  const auto comp =
      [](const std::tuple<double, boost::optional<PST_Connector>>& a,
         const std::tuple<double, boost::optional<PST_Connector>>& b) {
        return (std::get<0>(a) < std::get<0>(b));
      };

  // Pair the connectors with their distances to the target speed.
  const auto PL_0P_tuple = std::make_tuple(PL_0P_delta, PL_0P);
  const auto PP_max_tuple = std::make_tuple(PP_max_delta, PP_max);
  const auto PP_min_tuple = std::make_tuple(PP_min_delta, PP_min);

  // Find the best one.
  const auto best_tuple =
      std::min(PL_0P_tuple, std::min(PP_max_tuple, PP_min_tuple, comp), comp);

  // Done.
  return *std::get<1>(best_tuple);
}
}  // namespace maeve_automation_core
