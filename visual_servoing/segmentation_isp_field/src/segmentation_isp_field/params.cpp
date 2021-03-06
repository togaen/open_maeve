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
#include "segmentation_isp_field/params.h"

#include <boost/filesystem.hpp>

#include "open_maeve/isp_controller_2d/ros_interface.h"
#include "open_maeve/isp_field/ros_interface.h"

namespace open_maeve {
bool SegmentationFieldParams::load(const ros::NodeHandle& nh) {
  // Load node parameters.
  LOAD_PARAM(segmentation_sequence_topic);
  LOAD_PARAM(viz_isp_field_topic);
  LOAD_PARAM(visualize_horizons);
  LOAD_PARAM(control_command_input_topic);
  LOAD_PARAM(control_command_output_topic);
  LOAD_PARAM(segmentation_taxonomy);
  LOAD_PARAM(data_set_name);
  LOAD_PARAM(viz_potential_bounds_);
  LOAD_PARAM(horizon_viz_height);
  LOAD_NS_PARAM(default_guidance_control, throttle);
  LOAD_NS_PARAM(default_guidance_control, yaw);

  // Load potential transform parameters.
  if (!loadShapeParamsROS_Params(nh, "hard_constraint_transform",
                                 hard_constraint_transform)) {
    ROS_ERROR_STREAM("Failed to load hard constraint params.");
    return false;
  }
  if (!loadShapeParamsROS_Params(nh, "soft_constraint_transform",
                                 soft_constraint_transform)) {
    ROS_ERROR_STREAM("Failed to load soft constraint params.");
    return false;
  }

  // Load controller parameters.
  if (!loadISP_ControllerROS_Params(nh, "isp_controller_params",
                                    isp_controller_params)) {
    ROS_ERROR_STREAM("Failed to load ISP controller params.");
    return false;
  }

  // These parameters are set at run time by the camera callback. To bypass
  // validity checking, set them temporarily to valid values.
  const auto org_focal_length_x = isp_controller_params.focal_length_x;
  isp_controller_params.focal_length_x = 1.0;
  const auto org_principal_point_x = isp_controller_params.principal_point_x;
  isp_controller_params.principal_point_x = 1.0;

  // Get parameter validity.
  const auto all_valid = default_guidance_control.valid() &&
                         isp_controller_params.valid() &&
                         hard_constraint_transform.valid() &&
                         soft_constraint_transform.valid() && valid();

  // Reset run-time parameters.
  isp_controller_params.focal_length_x = org_focal_length_x;
  isp_controller_params.principal_point_x = org_principal_point_x;

  CHECK_EQ(viz_potential_bounds_.size(), 2);
  viz_potential_bounds =
      Interval_d(viz_potential_bounds_[0], viz_potential_bounds_[1]);

  // Done.
  return all_valid;
}

bool SegmentationFieldParams::valid() const {
  // Check this object's parameters.
  CHECK_NONEMPTY(segmentation_sequence_topic);
  CHECK_NONEMPTY(control_command_output_topic);
  CHECK_NONEMPTY(control_command_input_topic);
  CHECK_NONEMPTY(segmentation_taxonomy);

  if (!boost::filesystem::exists(segmentation_taxonomy)) {
    ROS_ERROR_STREAM("File '" << segmentation_taxonomy << "' does not exist.");
    return false;
  }

  CHECK_NONEMPTY(data_set_name);
  CHECK_GT(horizon_viz_height, 0);
  CHECK_FINITE(Interval_d::min(viz_potential_bounds));
  CHECK_FINITE(Interval_d::max(viz_potential_bounds));

  // All good.
  return true;
}
}  // namespace open_maeve
