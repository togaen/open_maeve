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

#include <limits>

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

AR_ISPFieldParams::AR_ISPFieldParams()
    : verbose(false),
      ar_time_queue_size(-1),
      ar_time_queue_max_gap(NaN),
      ar_tag_max_age(NaN),
      ar_tag_size(NaN) {}

bool AR_ISPFieldParams::load(const ros::NodeHandle& nh) {
  // Load parameters.
  LOAD_PARAM(camera_topic);
  LOAD_PARAM(viz_isp_field_topic);
  LOAD_PARAM(control_command_topic);
  LOAD_PARAM(ar_tag_obstacle_ids);
  LOAD_PARAM(ar_tag_target_ids);
  LOAD_PARAM(ar_frame_prefix);
  LOAD_PARAM(output_frame_param_name);
  LOAD_PARAM(marker_size_param_name);
  LOAD_PARAM(ar_time_queue_size);
  LOAD_PARAM(ar_time_queue_max_gap);
  LOAD_PARAM(viz_potential_bounds);
  LOAD_PARAM(verbose);
  LOAD_PARAM(ar_tag_max_age);

  LOAD_NS_PARAM(hard_constraint_transform, translation);
  LOAD_NS_PARAM(hard_constraint_transform, alpha);
  LOAD_NS_PARAM(hard_constraint_transform, beta);
  LOAD_NS_PARAM(hard_constraint_transform, range_min);
  LOAD_NS_PARAM(hard_constraint_transform, range_max);

  LOAD_NS_PARAM(soft_constraint_transform, translation);
  LOAD_NS_PARAM(soft_constraint_transform, alpha);
  LOAD_NS_PARAM(soft_constraint_transform, beta);
  LOAD_NS_PARAM(soft_constraint_transform, range_min);
  LOAD_NS_PARAM(soft_constraint_transform, range_max);

  LOAD_NS_PARAM(isp_controller_params.shape_parameters, translation);
  LOAD_NS_PARAM(isp_controller_params.shape_parameters, range_min);
  LOAD_NS_PARAM(isp_controller_params.shape_parameters, range_max);
  LOAD_NS_PARAM(isp_controller_params.shape_parameters, alpha);
  LOAD_NS_PARAM(isp_controller_params.shape_parameters, beta);

  LOAD_NS_PARAM(isp_controller_params.erosion_kernel, width);
  LOAD_NS_PARAM(isp_controller_params.erosion_kernel, height);
  LOAD_NS_PARAM(isp_controller_params.erosion_kernel, horizon);

  LOAD_NS_PARAM(isp_controller_params.yaw_decay, left);
  LOAD_NS_PARAM(isp_controller_params.yaw_decay, right);

  LOAD_NS_PARAM(isp_controller_params.guidance_gains, throttle);
  LOAD_NS_PARAM(isp_controller_params.guidance_gains, yaw);
  LOAD_NS_PARAM(isp_controller_params.guidance_gains, control_set);

  LOAD_NS_PARAM(isp_controller_params, K_P);
  LOAD_NS_PARAM(isp_controller_params, K_D);
  LOAD_NS_PARAM(isp_controller_params, potential_inertia);

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

  // Done.
  return all_valid;
}

bool AR_ISPFieldParams::valid() const {
  return hard_constraint_transform.valid() &&
         soft_constraint_transform.valid() && isp_controller_params.valid();
}

}  // namespace maeve_automation_core
