/*
 * Copyright 2018 Maeve Automation
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

#include <geometry_msgs/Transform.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>

namespace maeve_automation_core {
class KarlsruheDatasetPublisherNodeHandler {
 public:
  static constexpr auto CAMERA_FRAME_ID = "camera";
  static constexpr auto ODOM_FRAME_ID = "odom";
  static const geometry_msgs::Transform CAMERA_T_ODOM;

  /**
   * @brief Construct the ROS interface.
   */
  KarlsruheDatasetPublisherNodeHandler(const ros::NodeHandle& nh);

  /**
   * @brief Publish the GPS and left/right image messages.
   */
  void publish();

 private:
};  // class KarlsruheDatasetPublisherNodeHandler
}  // namespace maeve_automation_core
