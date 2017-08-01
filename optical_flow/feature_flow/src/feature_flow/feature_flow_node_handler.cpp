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

#include <cv_bridge/cv_bridge.h>

namespace maeve_automation_core {
FeatureFlowNodeHandler::FeatureFlowNodeHandler(const ros::NodeHandle& nh)
    : feature_flow_ptr(nullptr) {
  if (!params.load(nh)) {
    ROS_FATAL_STREAM("Failed to load parameters. Fatal error.");
    return;
  }
  ROS_INFO_STREAM("Loaded:\n" << params);

  // Instantiate feature flow object.
  feature_flow_ptr = std::unique_ptr<FeatureFlow>(new FeatureFlow(params.ff));

  // Set up ROS graph interactions.
  image_transport::ImageTransport it(nh);
  camera_sub = it.subscribe(params.camera_topic, 1,
                            &FeatureFlowNodeHandler::callback, this);

  if (!params.viz_topic.empty()) {
    viz_pub = it.advertise(params.viz_topic, 1);
  }
}

void FeatureFlowNodeHandler::visualize() const {
  // If visualizations disabled, done.
  if (params.viz_topic.empty()) {
    return;
  }

  // do stuff.
}

void FeatureFlowNodeHandler::callback(const sensor_msgs::Image::ConstPtr& msg) {
  // Error guard.
  if (!feature_flow_ptr) {
    return;
  }

  // Convert to OpenCV.
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }

  // Add to feature flow instance, compute.
  feature_flow_ptr->addFrame(cv_ptr->image);

  // Publish visualization.
  visualize();
}
}  // namespace maeve_automation_core
