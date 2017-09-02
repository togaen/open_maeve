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

#include "maeve_automation_core/isp_field/shape_parameters.h"

namespace maeve_automation_core {
class ISP_Controller {
 public:
  /**
   * @brief Container for controller parameters.
   */
  struct Params {
    /** @brief Proportional gain. */
    double K_P;
    /** @brief Derivative gain. */
    double K_D;
    /** @brief Max filter kernel width. */
    int kernel_width;
    /** @brief Max filter kernel height. */
    int kernel_height;
    /** @brief Max filter kernel horizon line. */
    int kernel_horizon;
    /** @brief Shape parameters used for safe control computation. */
    ShapeParameters shape_parameters;
    /**
     * @brief Constructor: initialize to invalid values.
     */
    Params();
  };  // struct Params

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
   * @brief Constructor: default.
   */
  ISP_Controller() = default;

  /**
   * @brief Constructor: initialize with shape parameters.
   *
   * @param params Parameters for control computation from ISP field.
   * @param initial_commanded_control Initialize the state of the controller
   * with this control command.
   */
  ISP_Controller(const Params& params,
                 const ControlCommand& initial_commanded_control);

  /**
   * @brief For a given ISP field compute a control command.
   *
   * @param ISP The input ISP field.
   *
   * @return The computed control command as a tuple of <throttle command,
   * steering command>.
   */
  ControlCommand computeControlCommand(const cv::Mat& ISP);

 private:
  /** @brief Controller parameters. */
  Params params_;
  /** @brief The previously computed control command. */
  ControlCommand commanded_control_;
};  // class ISP_Controller
}  // namespace maeve_automation_core
