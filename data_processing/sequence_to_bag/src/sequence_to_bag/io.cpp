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
#include "sequence_to_bag/io.h"

namespace maeve_automation_core {

sensor_msgs::CameraInfo synthesizeCameraInfoFromImageMsg(
    const sensor_msgs::ImagePtr& img_ptr) {
  sensor_msgs::CameraInfo cam_info_msg;

  cam_info_msg.header = img_ptr->header;
  cam_info_msg.width = img_ptr->width;
  cam_info_msg.height = img_ptr->height;
  cam_info_msg.distortion_model = "plumb_bob";

  cam_info_msg.D.resize(5);
  cam_info_msg.D[0] = 0.0;
  cam_info_msg.D[1] = 0.0;
  cam_info_msg.D[2] = 0.0;
  cam_info_msg.D[3] = 0.0;
  cam_info_msg.D[4] = 0.0;

  cam_info_msg.K[0] = 1.0;
  cam_info_msg.K[1] = 0.0;
  cam_info_msg.K[2] = static_cast<double>(img_ptr->width) / 2.0;
  cam_info_msg.K[3] = 0.0;
  cam_info_msg.K[4] = 1.0;
  cam_info_msg.K[5] = static_cast<double>(img_ptr->height) / 2.0;
  cam_info_msg.K[6] = 0.0;
  cam_info_msg.K[7] = 0.0;
  cam_info_msg.K[8] = 1.0;

  cam_info_msg.R[0] = 1.0;
  cam_info_msg.R[1] = 0.0;
  cam_info_msg.R[2] = 0.0;
  cam_info_msg.R[3] = 0.0;
  cam_info_msg.R[4] = 1.0;
  cam_info_msg.R[5] = 0.0;
  cam_info_msg.R[6] = 0.0;
  cam_info_msg.R[7] = 0.0;
  cam_info_msg.R[8] = 1.0;

  cam_info_msg.P[0] = 1.0;
  cam_info_msg.P[1] = 0.0;
  cam_info_msg.P[2] = static_cast<double>(img_ptr->width) / 2.0;
  cam_info_msg.P[3] = 0.0;

  cam_info_msg.P[4] = 1.0;
  cam_info_msg.P[5] = static_cast<double>(img_ptr->height) / 2.0;
  cam_info_msg.P[6] = 0.0;
  cam_info_msg.P[7] = 0.0;

  cam_info_msg.P[8] = 0.0;
  cam_info_msg.P[9] = 0.0;
  cam_info_msg.P[10] = 1.0;
  cam_info_msg.P[11] = 0.0;

  cam_info_msg.binning_x = 1;
  cam_info_msg.binning_y = 1;

  cam_info_msg.roi.width = img_ptr->width;
  cam_info_msg.roi.height = img_ptr->height;
  cam_info_msg.roi.x_offset = 0;
  cam_info_msg.roi.y_offset = 0;
  cam_info_msg.roi.do_rectify = false;

  return cam_info_msg;
}

}  // namespace maeve_automation_core
