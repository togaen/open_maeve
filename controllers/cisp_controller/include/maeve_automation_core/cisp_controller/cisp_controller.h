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

#include <opencv2/opencv.hpp>

#include "maeve_automation_core/cisp_field/shape_parameters.h"

namespace maeve_automation_core {
class CISP_Controller {
 public:
  /**
   * @brief Container for passing control commands.
   */
  struct ControlCommand {
    /** @brief The commanded throttle. */
    double throttle;
    /** @brief The commanded steering. */
    double steering;
    /**
     * @brief Constructor: initialize to invalid values.
     */
    ControlCommand();
    /**
     * @brief Constructor: initialize to explicit values.
     *
     * @param t The throttle command.
     * @param s The steering command.
     */
    ControlCommand(const double t, const double s);
  };  // struct ControlCommand

  /**
   * @brief Constructor: initialize to invalid values.
   */
  CISP_Controller() = default;

  /**
   * @brief Constructor: initialize with shape parameters.
   *
   * @param shape_parameters Shape parameters for performing control horizon
   * projection.
   * @param initial_commanded_control Initialize the state of the controller
   * with this control command.
   */
  CISP_Controller(const ShapeParameters& shape_parameters,
                  const ControlCommand& initial_commanded_control);

  /**
   * @brief For a given CISP field compute a control command.
   *
   * @param CISP The input CISP field.
   *
   * @return The computed control command as a tuple of <throttle command,
   * steering command>.
   */
  ControlCommand computeControlCommand(const cv::Mat& CISP);

 private:
  /** @brief Shape parameters used for control horizon projection. */
  ShapeParameters shape_parameters_;
  /** @brief The previously computed control command. */
  ControlCommand commanded_control_;
};  // class CISP_Controller
}  // namespace maeve_automation_core
