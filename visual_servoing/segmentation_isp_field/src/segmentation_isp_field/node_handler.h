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

#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Header.h>

#include <string>

#include "maeve_automation_core/controller_interface_msgs/command2d_manager.h"
#include "maeve_automation_core/isp_controller_2d/isp_controller_2d.h"
#include "segmentation_isp_field/params.h"

namespace maeve_automation_core {
/**
 * @brief Interface between ROS and the segmentation-based ISP controller.
 */
class SegmentationFieldNodeHandler {
 public:
  /**
   * @brief Construct handler instance and register callbacks/subscribers.
   *
   * @param node_name The node name used to construct the ROS node handle.
   */
  explicit SegmentationFieldNodeHandler(const std::string& node_name);

 private:
  /**
   * @brief Callback that fires when a new image segmentation is received.
   *
   * @param msg The segmentation image message.
   */
  void segmentationSequenceCallback(const sensor_msgs::ImageConstPtr& msg);

  /** @brief Manager for retrieving most recent control commands. */
  Command2D_Manager command2d_mgr_;

  /** @brief Compute control commands from ISP field. */
  ISP_Controller isp_controller_;

  /** @brief Camera image subscriber. */
  image_transport::Subscriber segmentation_sub_;
  /** @brief ISP field visualization publisher. */
  image_transport::Publisher viz_isp_field_pub_;
  /** @brief The ROS node handle. */
  ros::NodeHandle nh_;
  /** @brief The ROS image transport object. */
  image_transport::ImageTransport it_;
  /** @brief Desired control command subscriber. */
  ros::Subscriber control_command_input_sub_;
  /** @brief Control command publisher. */
  ros::Publisher control_command_output_pub_;
  /** @brief Node params. */
  SegmentationFieldParams params_;
};  // class SegmentationFieldNodeHandler
}  // namespace maeve_automation_core
