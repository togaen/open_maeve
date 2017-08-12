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

#include <string>

#include "maeve_automation_core/ros_parameter_loading/params_base.h"

namespace maeve_automation_core {

/** @brief Parameter object to load ROS params.*/
struct AR_CISPFieldParams : public ParamsBase {
  /**
   * @brief Container for potential transform parameters.
   */
  struct PotentialTransform {
    /** @brief The alpha shape parameter. */
    double alpha;
    /** @brief The beta shape parameter. */
    double beta;
    /** @brief The min of the constraint range. */
    double range_min;
    /** @brief The max of the constraint range. */
    double range_max;
    /**
     * @brief Constructor: initialize to invalid values.
     */
    PotentialTransform();
    /**
     * @brief Sanity check whether parameters make sense.
     *
     * @return True if parameters pass sanity check; otherwise false.
     */
    __attribute__((warn_unused_result)) bool valid() const;
  };  // struct PotentialTransform

  /** @brief Hard constraint transform parameters. */
  PotentialTransform hard_constraint_transform;

  /** @brief Soft constraint transform parameters. */
  PotentialTransform soft_constraint_transform;

  /** @brief Publish rate for transformed measurement field (Hz). */
  double measurement_field_publish_rate;

  /** @brief The image sequence topic. */
  std::string camera_topic;

  /** @brief The CISP field visualization topic. */
  std::string viz_cisp_field_topic;

  /**
   * @copydoc ParamsBase::ParamsBase()
   */
  __attribute__((warn_unused_result)) bool load(
      const ros::NodeHandle& nh) override;
};  // struct AR_CISPFieldParams

}  // namespace maeve_automation_core
