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
#include "maeve_expansion_segmentation/node_handler.h"

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>

namespace maeve_automation_core {
MaeveExpansionSegmentationNodeHandler::MaeveExpansionSegmentationNodeHandler(
    const ros::NodeHandle& nh) {
  if (!params_.load(nh)) {
    ROS_FATAL_STREAM("Failed to load parameters. Fatal error.");
    return;
  }

  // Image transport interface.
  image_transport::ImageTransport it(nh);

  // Register callback.
  camera_sub =
      it.subscribe(params_.camera_topic, 1,
                   &MaeveExpansionSegmentationNodeHandler::callback, this);
}

void MaeveExpansionSegmentationNodeHandler::callback(
    const sensor_msgs::Image::ConstPtr& msg) {
  ROS_INFO_STREAM("entered callback");
}
}  // namespace maeve_automation_core
