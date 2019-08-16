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
#include "ar_isp_field/params.h"

#include <cstdlib>
#include <limits>

#include "open_maeve/isp_controller_2d/ros_interface.h"
#include "open_maeve/isp_field/ros_interface.h"

namespace open_maeve {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

AR_ISPFieldParams::AR_ISPFieldParams()
    : verbose(false),
      ar_time_queue_size(-1),
      ar_time_queue_max_gap(NaN),
      ar_tag_max_age(NaN),
      ar_tag_size(NaN),
      target_reward(NaN) {}

bool AR_ISPFieldParams::load(const ros::NodeHandle& nh) {
  // Load parameters.
  LOAD_PARAM(camera_topic);
  LOAD_PARAM(viz_isp_field_topic);
  LOAD_PARAM(control_command_input_topic);
  LOAD_PARAM(control_command_output_topic);
  LOAD_PARAM(ar_tag_obstacle_ids);
  LOAD_PARAM(ar_tag_target_ids);
  LOAD_PARAM(ar_frame_prefix);
  LOAD_PARAM(output_frame_param_name);
  LOAD_PARAM(marker_size_param_name);
  LOAD_PARAM(ar_time_queue_size);
  LOAD_PARAM(ar_time_queue_max_gap);
  LOAD_PARAM(viz_potential_bounds_);
  LOAD_PARAM(verbose);
  LOAD_PARAM(potential_only_guidance);
  LOAD_PARAM(ar_tag_max_age);
  LOAD_PARAM(target_reward);
  LOAD_PARAM(horizon_viz_height);
  LOAD_PARAM(visualize_horizons);

  LOAD_NS_PARAM(default_guidance_control, throttle);
  LOAD_NS_PARAM(default_guidance_control, yaw);

  // Load potential transform parameters.
  if (!loadShapeParamsROS_Params(nh, "hard_constraint_transform",
                                 hard_constraint_transform)) {
    return false;
  }
  if (!loadShapeParamsROS_Params(nh, "soft_constraint_transform",
                                 soft_constraint_transform)) {
    return false;
  }

  // Load ISP controller params.
  if (!loadISP_ControllerROS_Params(nh, "isp_controller_params",
                                    isp_controller_params)) {
    return false;
  }

  // Load AR tag parameters.
  if (!nh.getParam(output_frame_param_name, camera_frame_name)) {
    return false;
  }
  if (!nh.getParam(marker_size_param_name, ar_tag_size)) {
    return false;
  }

  // Convert cm -> m
  ar_tag_size /= 100.0;

  // These parameters are set at run time by the camera callback. To bypass
  // validity checking, set them temporarily to valid values.
  const auto org_focal_length_x = isp_controller_params.focal_length_x;
  isp_controller_params.focal_length_x = 1.0;
  const auto org_principal_point_x = isp_controller_params.principal_point_x;
  isp_controller_params.principal_point_x = 1.0;

  // Get parameter validity.
  const auto all_valid = valid();

  // Reset run-time parameters.
  isp_controller_params.focal_length_x = org_focal_length_x;
  isp_controller_params.principal_point_x = org_principal_point_x;

  CHECK_EQ(viz_potential_bounds_.size(), 2);
  viz_potential_bounds =
      Interval_d(viz_potential_bounds_[0], viz_potential_bounds_[1]);

  // Done.
  return all_valid;
}

bool AR_ISPFieldParams::valid() const {
  // Check this object's parameters.
  CHECK_GT(ar_time_queue_size, 0);
  CHECK_GT(ar_time_queue_max_gap, 0);
  CHECK_GT(ar_tag_max_age, 0);
  CHECK_GT(ar_tag_size, 0);
  CHECK_NOT_NAN(target_reward);
  CHECK_NONEMPTY(ar_frame_prefix);
  CHECK_NONEMPTY(camera_frame_name);
  CHECK_NONEMPTY(camera_topic);
  CHECK_NONEMPTY(viz_isp_field_topic);
  CHECK_NONEMPTY(control_command_output_topic);
  CHECK_NONEMPTY(control_command_input_topic);
  CHECK_FINITE(Interval_d::min(viz_potential_bounds));
  CHECK_FINITE(Interval_d::max(viz_potential_bounds));

  // Return okay if all members are ok.
  return default_guidance_control.valid() &&
         hard_constraint_transform.valid() &&
         soft_constraint_transform.valid() && isp_controller_params.valid();
}

}  // namespace open_maeve
