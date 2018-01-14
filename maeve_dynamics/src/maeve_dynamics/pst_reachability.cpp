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

namespace maeve_automation_core {
PST_Reachability::PST_Reachability(PST_Connector&& min_terminal,
                                   PST_Connector&& max_terminal)
    : min_terminal_(std::move(min_terminal)),
      max_terminal_(std::move(max_terminal)) {}

boost::optional<PST_Reachability> PST_Reachability::compute(
    const Eigen::Vector2d& p1, const Interval& v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

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

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::I>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::II>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::III>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::IV>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::V>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VI>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VII>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VIII>(
    const Eigen::Vector2d& p1, const double v_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

/***/

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::I>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::II>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::III>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::IV>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::V>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VI>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VII>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

template <>
boost::optional<PST_Connector>
PST_Reachability::connector<PST_Reachability::Type::VIII>(
    const Eigen::Vector2d& p1, const Interval& V_i, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

/***/

template <>
boost::optional<PST_Connector>
PST_Reachability::maxTerminalSpeed<PST_Reachability::Type::V>(
    const Interval& I_i, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

// LP+
template <>
boost::optional<PST_Connector>
PST_Reachability::maxTerminalSpeed<PST_Reachability::Type::VII>(
    const Interval& I_i, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  // Intervals for dynamic bounds.
  const auto& I_dt = IntervalConstraints<2>::boundsS<1>(constraints);
  const auto& I_ddt = IntervalConstraints<2>::boundsS<2>(constraints);

  // Terminal speed is max feasible speed.
  const auto dt = Interval::max(I_dt);

  // Constant max acceleration.
  const auto ddt = Interval::max(I_ddt);

  // If it's LP, we're done.
  if (const auto connector = PST_Connector::computeLP(p1, p2, dt, ddt, I_dt)) {
    return connector;
  }

  // For PLP, need initial speed interval bounds.
  double p1_dt_min, p1_dt_max;
  std::tie(p1_dt_min, p1_dt_max) = Interval::bounds(I_i);

  // If it's PLP, we're done.
  const auto connector_min =
      PST_Connector::computePLP(p1, p1_dt_min, p2, dt, ddt, I_dt);
  const auto connector_max =
      PST_Connector::computePLP(p1, p1_dt_max, p2, dt, ddt, I_dt);
  if (connector_min && connector_max) {
    const auto s1 = PST_Connector::terminalSpeed(*connector_min);
    const auto s2 = PST_Connector::terminalSpeed(*connector_max);
    return (s1 > s2) ? *connector_min : *connector_max;
  }
  if (connector_min) {
    return connector_min;
  }
  if (connector_max) {
    return connector_max;
  }

  // TODO(me): PP

  // Done.
  return boost::none;
}

/***/

template <>
boost::optional<PST_Connector>
PST_Reachability::minTerminalSpeed<PST_Reachability::Type::VI>(
    const Interval& I_i, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  return boost::none;
}

// LP-
template <>
boost::optional<PST_Connector>
PST_Reachability::minTerminalSpeed<PST_Reachability::Type::VIII>(
    const Interval& I_i, const Eigen::Vector2d& p1, const Eigen::Vector2d& p2,
    const IntervalConstraints<2>& constraints) {
  // Intervals for dynamic bounds.
  const auto& I_dt = IntervalConstraints<2>::boundsS<1>(constraints);
  const auto& I_ddt = IntervalConstraints<2>::boundsS<2>(constraints);

  // Terminal speed is max feasible speed.
  const auto dt = Interval::min(I_dt);

  // Constant max acceleration.
  const auto ddt = Interval::min(I_ddt);

  // If it's LP, we're done.
  if (const auto connector = PST_Connector::computeLP(p1, p2, dt, ddt, I_dt)) {
    return connector;
  }

  // For PLP, need initial speed interval bounds.
  double p1_dt_min, p1_dt_max;
  std::tie(p1_dt_min, p1_dt_max) = Interval::bounds(I_i);

  // If it's PLP, we're done.
  const auto connector_min =
      PST_Connector::computePLP(p1, p1_dt_min, p2, dt, ddt, I_dt);
  const auto connector_max =
      PST_Connector::computePLP(p1, p1_dt_max, p2, dt, ddt, I_dt);
  if (connector_min && connector_max) {
    const auto s1 = PST_Connector::terminalSpeed(*connector_min);
    const auto s2 = PST_Connector::terminalSpeed(*connector_max);
    return (s1 > s2) ? *connector_min : *connector_max;
  }
  if (connector_min) {
    return connector_min;
  }
  if (connector_max) {
    return connector_max;
  }

  // TODO(me): PP

  // Done.
  return boost::none;
}
}  // namespace maeve_automation_core
