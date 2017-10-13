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

#include "maeve_automation_core/isp_controller_2d/ros_interface.h"

namespace maeve_automation_core {
SegmentationFieldParams::SegmentationFieldParams() {}

bool SegmentationFieldParams::load(const ros::NodeHandle& nh) {
  // Load node parameters.
  LOAD_PARAM(segmentation_sequence_topic);
  LOAD_PARAM(viz_isp_field_topic);
  LOAD_PARAM(control_command_input_topic);
  LOAD_PARAM(control_command_output_topic);

  // Load controller parameters.
  if (!loadISP_ControllerROS_Params(nh, "isp_controller_params",
                                    isp_controller_params)) {
    return false;
  }

  // Done.
  return valid();
}

bool SegmentationFieldParams::valid() const {
  // Check this object's parameters.
  CHECK_NONEMPTY(segmentation_sequence_topic);
  CHECK_NONEMPTY(viz_isp_field_topic);
  CHECK_NONEMPTY(control_command_output_topic);
  CHECK_NONEMPTY(control_command_input_topic);

  // All good.
  return true;
}
}
