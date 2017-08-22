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
#include "ar_cisp_field/params.h"

#include <limits>

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
}  // namespace

AR_CISPFieldParams::AR_CISPFieldParams()
    : verbose(false),
      ar_time_queue_size(-1),
      ar_time_queue_max_gap(NaN),
      ar_tag_max_age(NaN),
      ar_tag_size(NaN) {}

bool AR_CISPFieldParams::load(const ros::NodeHandle& nh) {
  // Load parameters.
  LOAD_PARAM(camera_topic);
  LOAD_PARAM(viz_cisp_field_topic);
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

  LOAD_NS_PARAM(hard_constraint_transform, alpha);
  LOAD_NS_PARAM(hard_constraint_transform, beta);
  LOAD_NS_PARAM(hard_constraint_transform, range_min);
  LOAD_NS_PARAM(hard_constraint_transform, range_max);
  hard_constraint_transform.computeMidPoint();

  LOAD_NS_PARAM(soft_constraint_transform, alpha);
  LOAD_NS_PARAM(soft_constraint_transform, beta);
  LOAD_NS_PARAM(soft_constraint_transform, range_min);
  LOAD_NS_PARAM(soft_constraint_transform, range_max);
  soft_constraint_transform.computeMidPoint();

  // Sanity check params.
  CHECK_GE(ar_tag_max_age, 0.0);
  CHECK_STRICTLY_POSITIVE(ar_time_queue_size);
  CHECK_STRICTLY_POSITIVE(ar_time_queue_max_gap);
  CHECK_NONEMPTY(output_frame_param_name);
  CHECK_NONEMPTY(marker_size_param_name);
  CHECK_NONEMPTY(camera_topic);
  CHECK_NONEMPTY(ar_frame_prefix);
  CHECK_EQ(viz_potential_bounds.size(), 2);
  CHECK_LT(viz_potential_bounds[0], 0.0);
  CHECK_GT(viz_potential_bounds[1], 0.0);

  // Load AR tag parameters.
  if (!nh.getParam(output_frame_param_name, camera_frame_name)) {
    return false;
  }
  if (!nh.getParam(marker_size_param_name, ar_tag_size)) {
    return false;
  }

  // Convert cm -> m
  ar_tag_size /= 100.0;

  // All good?
  const auto hc_valid = hard_constraint_transform.valid();
  const auto sc_valid = soft_constraint_transform.valid();
  return hc_valid && sc_valid;
}

}  // namespace maeve_automation_core
