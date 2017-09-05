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

#include "maeve_automation_core/isp_controller/control_command.h"
#include "maeve_automation_core/isp_field/shape_parameters.h"

namespace maeve_automation_core {
class ISP_Controller {
 public:
  /**
   * @brief Container for controller parameters.
   */
  struct Params {
    /** @brief Max filter kernel width. */
    int kernel_width;
    /** @brief Max filter kernel height. */
    int kernel_height;
    /** @brief Max filter kernel horizon line. */
    int kernel_horizon;
    /** @brief Camera focal length along x (pixels). */
    double focal_length_x;
    /** @brief The x-coordinate of the camera principal point (pixels). */
    double principal_point_x;
    /** @brief Yaw bias left decay. */
    double yaw_decay_left;
    /** @brief Yaw bias right decay. */
    double yaw_decay_right;
    /** @brief Proportional gain. */
    double K_P;
    /** @brief Derivative gain. */
    double K_D;
    /** @brief Potential delta must exceed this to trigger control change. */
    double potential_inertia;
    /** @brief Shape parameters used for safe control computation. */
    ShapeParameters shape_parameters;
    /**
     * @brief Constructor: initialize to invalid values.
     */
    Params();
    /**
     * @brief Constructor: explicit initialization.
     *
     * @param sp The shape parameters object.
     * @param k_w The max filter kernel width.
     * @param k_ht The max filter kernel height.
     * @param k_hr The max filter kernel horizon line.
     * @param fx The camera focal length along x (pixels).
     * @param px The x-coordinate of the camera principal point (pixels).
     * @param ld The yaw bias left decay.
     * @param rd The yaw bias right decay.
     * @param kp Proportional gain for control projection.
     * @param kd Derivative gain for control projection.
     * @param pi Potential inertia to overcome to change direction.
     */
    Params(const ShapeParameters& sp, const int k_w, const int k_ht,
           const int k_hr, const double fx, const double px, const double ld,
           const double rd, const double kp, const double kd, const double pi);
  };  // struct Params

  /**
   * @brief Constructor: default.
   */
  ISP_Controller() = default;

  /**
   * @brief Constructor: initialize with shape parameters.
   *
   * @param params Parameters for control computation from ISP field.
   */
  ISP_Controller(const Params& params);

  /**
   * @brief For a given ISP field compute a selective determinism control.
   *
   * @note For reasonable results, the desired control commands should be in the
   * range [-1, 1] indicating minimum and maximum actuation values,
   * respectively.
   *
   * @param ISP The input ISP field.
   * @param u_d The desired control command.
   *
   * @return The computed control command with each member in the range [-1, 1]
   * indicating minimum and maximum actuation, respectively.
   */
  ControlCommand SD_Control(const cv::Mat& ISP, const ControlCommand& u_d);

 private:
  /** @brief Controller parameters. */
  Params p_;
};  // class ISP_Controller

/**
 * @brief Overload output stream operator for controller parameter set.
 *
 * @param o The output stream.
 * @param sp The controller parameters object.
 *
 * @return The stream.
 */
std::ostream& operator<<(std::ostream& o, const ISP_Controller::Params& p);
}  // namespace maeve_automation_core
