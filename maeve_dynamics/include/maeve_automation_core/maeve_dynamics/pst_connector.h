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

#include <array>
#include <iostream>

#include "maeve_automation_core/maeve_dynamics/parabola.h"

namespace maeve_automation_core {
/**
 * @brief This class defines the canonical form of a PST connecting trajectory.
 *
 * The canonical form of a PST connecting trajectory is
 * Parabolic-Linear-Parabolic. The trajectory begins at the first switching
 * time, changes to the linear portion at the second switching time, changes to
 * the terminal parabolic portion at the third switching time, and ends at the
 * fourth switching time. The linear portion of the trajectory is represented as
 * a parabola with a zero coefficient for the quadratic term.
 */
class PST_Connector {
 public:
  /**
   * @brief Constructor: explicitly initialize the connector.
   *
   * @note This constructor checks for validity of the arguments and throws an
   * exception if they do not meet basic necessary conditions.
   *
   * @param switching_times Trajectory switching times.
   * @param functions Trajectory functional segments.
   */
  PST_Connector(std::array<double, 4>&& switching_times,
                std::array<Parabola, 3>&& functions);

 private:
  /**
   * @brief Check whether switching times are strictly non-decreasing.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the switching times are non-decreasing; otherwise false.
   */
  static bool switchingTimesNonDecreasing(const PST_Connector& connector);

  /**
   * @brief Check whether the function segments of the connector intersect at
   * the switching times.
   *
   * @note An approximate floating point comparison is used.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the segments are connected; otherwise false.
   */
  static bool segmentsConnected(const PST_Connector& connector);

  /**
   * @brief Check whether the function segments of the connector have equal
   * first derivatives at the switching times.
   *
   * @note An approximate floating point comparison is used.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the segments have equal first derivatives; otherwise false.
   */
  static bool segmentsTangent(const PST_Connector& connector);

  /**
   * @brief Check whether all parabola coefficients are real valued.
   *
   * @param connector The connecting trajectory to check.
   *
   * @return True if the segments all have real coefficients; otherwise false.
   */
  static bool realCoefficients(const PST_Connector& connector);

  /**
   * @brief Perform basic checks for validity of the connecting trajectory.
   *
   * In order to be valid, the following are necessary conditions:
   *
   *   1) The switching times must be strictly non-decreasing.
   *   2) The values and tangents of parabolas at indices 0 and 1 must be equal
   *      at the switching time at index 1.
   *   3) The values and tangents of parabolas at indices 1 and 2 must be equal
   *      at the switching time at index 2.
   *   4) All parabola coefficients must be real valued.
   *
   * @note These are necessary, not sufficient, conditions for the connector to
   * be valid.
   *
   * @param connecting_trajectory
   *
   * @return True if the connector passes necessary checks; otherwise false.
   */
  static bool valid(const PST_Connector& connector);

  /**
   * @brief Switching times, in order, of the trajectory.
   *
   * The switching times are ordered as follows:
   *
   *   0: Trajectory begins, switches to initial parabolic portion
   *   1: Trajectory switches to linear portion
   *   2: Trajectory switches to second parabolic portion
   *   3: Trajectory ends
   */
  std::array<double, 4> switching_times_;

  /**
   * @brief Ordered functional segments of trajectory.
   *
   * The functional segments are ordered as follows:
   *
   *   0: Initial parabolic segment
   *   1: Interstitial linear segment
   *   2: Terminal parabolic segment
   */
  std::array<Parabola, 3> functions_;
};  // class PST_Connector
}  // namespace maeve_automation_core
