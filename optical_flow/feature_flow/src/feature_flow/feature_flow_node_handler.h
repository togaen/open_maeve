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
#include "maeve_automation_core/feature_flow/feature_flow.h"

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <memory>
#include <string>

#include "feature_flow/params.h"

namespace maeve_automation_core {
/**
 * @brief Interface between ROS node and Feature Flow class.
 */
class FeatureFlowNodeHandler {
 public:
  /**
   * @brief Construct the ROS interface.
   *
   * @param params The ROS node params.
   * @param nh The ROS node handle.
   */
  FeatureFlowNodeHandler(const ros::NodeHandle& nh);

  /**
   * @brief Callback to convert ROS image message to OpenCV and feed it to
   * Feature Flow instance.
   *
   * @param msg The ROS image message.
   */
  void callback(const sensor_msgs::Image::ConstPtr& msg);

  /**
   * @brief Visualize the current state of the Feature Flow object.
   */
  void visualize() const;

 private:
  /** @brief Camera image subscriber. */
  image_transport::Subscriber camera_sub;
  /** @brief Visualization publisher. */
  image_transport::Publisher viz_pub;
  /** @brief The Feature Flow object parameters. */
  FeatureFlowParams params;
  /** @brief The Feature Flow object segmentation and tracker. */
  std::unique_ptr<maeve_automation_core::FeatureFlow> feature_flow_ptr;
};  // class FeatureFlowNodeHandler
}  // namespace maeve_automation_core
