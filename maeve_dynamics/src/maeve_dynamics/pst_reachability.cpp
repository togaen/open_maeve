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

#include <limits>

namespace maeve_automation_core {
namespace {
const auto Inf = std::numeric_limits<double>::infinity();
const auto NaN = std::numeric_limits<double>::quiet_NaN();
}

PST_Reachability::PST_Reachability(PST_Connector&& min_terminal,
                                   PST_Connector&& max_terminal)
    : min_terminal_(std::move(min_terminal)),
      max_terminal_(std::move(max_terminal)) {}

Interval PST_Reachability::reachableInterval(
    const PST_Reachability& reachability) {
  const auto& min_connector = PST_Reachability::minConnector(reachability);
  const auto& max_connector = PST_Reachability::maxConnector(reachability);
  const auto min_speed = PST_Connector::terminalSpeed(min_connector);
  const auto max_speed = PST_Connector::terminalSpeed(max_connector);

  return Interval(min_speed, max_speed);
}

const PST_Connector& PST_Reachability::minConnector(
    const PST_Reachability& reachability) {
  return reachability.min_terminal_;
}

const PST_Connector& PST_Reachability::maxConnector(
    const PST_Reachability& reachability) {
  return reachability.max_terminal_;
}

namespace {
boost::optional<PST_Connector> LPorPLP(const PST_Connector& LP,
                                       const double target_speed,
                                       const Interval& I_dt,
                                       const Interval& I_ddt) {
  const auto ddt_min = Interval::min(I_ddt);
  const auto ddt_max = Interval::max(I_ddt);
  const auto terminal_ddt = PST_Connector::terminalAcceleration(LP);

  // If LP is valid, just return it.
  const auto v = PST_Connector::initialSpeed(LP);
  if (Interval::contains(I_dt, v)) {
    return LP;
  }

  // Get start and end points for connector.
  Eigen::Vector2d p1, p2;
  std::tie(p1, p2) = PST_Connector::boundaryPoints(LP);

  //
  // LP is invalid, attempt building a PLP.
  //

  // Get valid starting speed.
  const auto dt_valid = Interval::projectToInterval(I_dt, v);

  // Initial acceleration should move toward dt_valid.
  const auto ddt = ((v < dt_valid) ? ddt_max : ddt_min);

  // Get appropriate terminal acceleration.
  const auto PLP_min =
      PST_Connector::computePLP(p1, dt_valid, ddt, p2, target_speed, ddt_min);
  const auto PLP_max =
      PST_Connector::computePLP(p1, dt_valid, ddt, p2, target_speed, ddt_max);

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
}  // namespace

// LP+
template <>
boost::optional<PST_Connector>
PST_Reachability::maxTerminalSpeed<PST_Reachability::Type::VII>(
    const Interval& I_i, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  // Intervals for dynamic bounds.
  const auto& I_dt = IntervalConstraints<2>::boundsS<1>(constraints);
  const auto& I_ddt = IntervalConstraints<2>::boundsS<2>(constraints);

  // Extremal speeds.
  const auto dt_max = Interval::max(I_dt);
  const auto dt_min = Interval::min(I_dt);

  const auto target_speed = dt_max;

  // Extremal accelerations.
  const auto ddt_max = Interval::max(I_ddt);
  const auto ddt_min = Interval::min(I_ddt);

  // Check for LP or PLP connectivity: these will hit target speed exactly,
  // so return if connector found.
  if (const auto LP = PST_Connector::computeLP(p1, p2, target_speed, ddt_min)) {
    if (const auto PLP = LPorPLP(*LP, target_speed, I_dt, I_ddt)) {
      return *PLP;
    }
  }
  if (const auto LP = PST_Connector::computeLP(p1, p2, target_speed, ddt_max)) {
    if (const auto PLP = LPorPLP(*LP, target_speed, I_dt, I_ddt)) {
      return *PLP;
    }
  }

  // Check for PL_0P or PP: these will not hit target speed exactly, so compute
  // all and choose nearest.
  const auto PL_0P =
      PST_Connector::computePL_0P(p1, dt_min, ddt_min, p2, ddt_max);
  const auto PP_max =
      PST_Connector::computePP(p1, dt_min, ddt_min, p2, ddt_max);
  const auto PP_min =
      PST_Connector::computePP(p1, dt_max, ddt_max, p2, ddt_min);

  // TODO(me): choose the connector that gets nearest the target speed.

  // Done.
  return boost::none;
}

}  // namespace maeve_automation_core
