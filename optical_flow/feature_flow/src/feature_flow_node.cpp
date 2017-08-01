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
#include "feature_flow/feature_flow_node_handler.h"
#include "maeve_automation_core/feature_flow/feature_flow.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <string>

#include "feature_flow/params.h"

int main(int argc, char* argv[]) {
  const auto node_name = std::string("feature_flow");

  // Initialize ROS node.
  ros::init(argc, argv, node_name);
  auto nh = ros::NodeHandle(node_name);

  // Load params.
  maeve_automation_core::FeatureFlowParams params;
  if (!params.load(nh)) {
    ROS_ERROR_STREAM("Failed to load params.");
    return EXIT_FAILURE;
  }
  ROS_INFO_STREAM("Loaded:\n" << params);

  // Create handler.
  maeve_automation_core::FeatureFlowNodeHandler handler(params, nh);

  // Listen to message stream.
  image_transport::ImageTransport it(nh);
  const auto sub = it.subscribe(
      params.camera_topic, 1,
      &maeve_automation_core::FeatureFlowNodeHandler::callback, &handler);

  // Kick it off.
  ros::spin();

  // Done.
  return EXIT_SUCCESS;
}
