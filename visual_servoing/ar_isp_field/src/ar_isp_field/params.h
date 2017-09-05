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
#include <vector>

#include "maeve_automation_core/isp_field/shape_parameters.h"
#include "maeve_automation_core/ros_parameter_loading/params_base.h"

namespace maeve_automation_core {

/** @brief Parameter object to load ROS params.*/
struct AR_ISPFieldParams : public ParamsBase {
  /** @brief Hard constraint transform parameters. */
  ShapeParameters hard_constraint_transform;

  /** @brief Soft constraint transform parameters. */
  ShapeParameters soft_constraint_transform;

  /** @brief Print terminal output? */
  bool verbose;

  /** @brief Number of elements to reserve for circular buffer. */
  int ar_time_queue_size;

  /** @brief Max time gap for elements in the time queue. */
  double ar_time_queue_max_gap;

  /** @brief Max age of an AR tag timestamp to use before signalling error. */
  double ar_tag_max_age;

  /** @brief Size along one edge of AR tag (meters). */
  double ar_tag_size;

  /** @brief Name of the parameter specifying the AR tag size. */
  std::string marker_size_param_name;

  /** @brief Unique IDs for the AR tags corresponding to obstacles. */
  std::vector<int> ar_tag_obstacle_ids;

  /** @brief Unique IDs for the AR tags corresponding to targets. */
  std::vector<int> ar_tag_target_ids;

  /** @brief Prefix string for AR tag coordinate frames. */
  std::string ar_frame_prefix;

  /** @brief Name of the camera's coordinate frame. */
  std::string camera_frame_name;

  /** @brief Name of the parameter specifying the camera's coordinate frame. */
  std::string output_frame_param_name;

  /** @brief The image sequence topic. */
  std::string camera_topic;

  /** @brief The ISP field visualization topic. */
  std::string viz_isp_field_topic;

  /** @brief Control command output topic. */
  std::string control_command_topic;

  /** @brief The scaling bounds for visualizing potential values. */
  std::vector<double> viz_potential_bounds;

  /**
   * @brief Default constructor: initialize to invalid values.
   */
  AR_ISPFieldParams();

  /**
   * @copydoc ParamsBase::ParamsBase()
   */
  __attribute__((warn_unused_result)) bool load(
      const ros::NodeHandle& nh) override;
};  // struct AR_ISPFieldParams

}  // namespace maeve_automation_core
