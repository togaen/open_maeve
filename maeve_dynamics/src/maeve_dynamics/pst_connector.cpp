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
#include "maeve_automation_core/maeve_dynamics/pst_connector.h"

#include <limits>

#include "maeve_automation_core/maeve_geometry/comparisons.h"

namespace maeve_automation_core {
bool PST_Connector::switchingTimesNonDecreasing(
    const PST_Connector& connector) {
  // Check monotonicity.
  auto non_decreasing = true;
  for (auto i = 1; i < connector.switching_times_.size(); ++i) {
    non_decreasing = (non_decreasing && (connector.switching_times_[i] >=
                                         connector.switching_times_[i - 1]));
  }
  return non_decreasing;
}

bool PST_Connector::segmentsConnected(const PST_Connector& connector) {
  // Absolute error for doing approximate floating point comparisons.
  static const auto epsilon = 0.00001;

  // Record switching times for convenience.
  const auto t1 = connector.switching_times_[1];
  const auto t2 = connector.switching_times_[2];

  // Compute paths value of segments 0 and 1 at time t1.
  const auto s01 = connector.functions_[0](t1);
  const auto s11 = connector.functions_[1](t1);

  // Compute path values of segments 1 and 2 at time t2.
  const auto s12 = connector.functions_[1](t2);
  const auto s22 = connector.functions_[2](t2);

  // The path values at t1 and at t2 should be equal.
  return approxEq(s01, s11, epsilon) && approxEq(s12, s22, epsilon);
}

bool PST_Connector::segmentsTangent(const PST_Connector& connector) {
  // Absolute error for doing approximate floating point comparisons.
  static const auto epsilon = 0.00001;

  // Record switching times for convenience.
  const auto t1 = connector.switching_times_[1];
  const auto t2 = connector.switching_times_[2];

  // Compute \dot{s} value of segments 0 and 1 at time t1.
  const auto s_dot01 = Parabola::dt(connector.functions_[0], t1);
  const auto s_dot11 = Parabola::dt(connector.functions_[1], t1);

  // Compute \dot{s} values of segments 1 and 2 at time t2.
  const auto s_dot12 = Parabola::dt(connector.functions_[1], t2);
  const auto s_dot22 = Parabola::dt(connector.functions_[2], t2);

  // The path values at t1 and at t2 should be equal.
  return approxEq(s_dot01, s_dot11, epsilon) &&
         approxEq(s_dot12, s_dot22, epsilon);
}

bool PST_Connector::checkSegmentCoefficients(const PST_Connector& connector) {
  const auto seg1_coeff_zero = (Parabola::a(connector.functions_[0]) == 0.0);
  const auto seg2_coeff_zero = (Parabola::a(connector.functions_[1]) == 0.0);
  const auto seg3_coeff_zero = (Parabola::a(connector.functions_[2]) == 0.0);

  return !seg1_coeff_zero && seg2_coeff_zero && !seg3_coeff_zero;
}

bool PST_Connector::valid(const PST_Connector& connector) {
  // Check monotonicity.
  const auto non_decreasing =
      PST_Connector::switchingTimesNonDecreasing(connector);

  // Check connectivity.
  const auto segments_connected = PST_Connector::segmentsConnected(connector);

  // Check tangency.
  const auto segments_tangent = PST_Connector::segmentsTangent(connector);

  // Check coefficients.
  const auto segment_coefficients =
      PST_Connector::checkSegmentCoefficients(connector);

  // Done.
  return non_decreasing && segments_connected && segments_tangent &&
         segment_coefficients;
}

PST_Connector::PST_Connector(std::array<double, 4>&& switching_times,
                             std::array<Parabola, 3>&& functions)
    : switching_times_(std::move(switching_times)),
      functions_(std::move(functions)) {
  const auto is_valid = PST_Connector::valid(*this);
  if (!is_valid) {
    throw;
  }
}
}  // namespace maeve_automation_core
