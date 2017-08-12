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
#include "ar_cisp_field/node_handler.h"

#include <ros/ros.h>

#include "maeve_automation_core/cisp_field/isp.h"
#include "maeve_automation_core/cisp_field/potential_transforms.h"
#include "maeve_automation_core/cisp_field/visualize.h"

namespace maeve_automation_core {
AR_CISPFieldNodeHandler::AR_CISPFieldNodeHandler(const ros::NodeHandle& nh) {
  if (!params_.load(nh)) {
    ROS_FATAL_STREAM("Failed to load parameters. Fatal error.");
    return;
  }

  // Image transport interface.
  image_transport::ImageTransport it(nh);

  // Register callback.
  camera_sub = it.subscribe(params_.camera_topic, 1,
                            &AR_CISPFieldNodeHandler::callback, this);

  // Visualize?
  if (!params_.viz_cisp_field_topic.empty()) {
    viz_cisp_field_pub = it.advertise(params_.viz_cisp_field_topic, 1);
  }
}

void AR_CISPFieldNodeHandler::callback(
    const sensor_msgs::Image::ConstPtr& msg) {
  // ROS_INFO_STREAM("entered callback");

  // Convert to OpenCV.
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }

  // Convert incoming image to CV_64F.
  cv::Mat field;
  cv_ptr->image.convertTo(field, CV_64F, 1.0 / 255.0);

  // Compute ISP.
  const auto& hc = params_.hard_constraint_transform;
  const auto tx = PotentialTransform<ConstraintType::HARD>(
      std::make_tuple(hc.range_min, hc.range_max), hc.alpha, hc.beta);
  auto ISP = ImageSpacePotentialField::build(field, tx);

  // Visualize ISP.
  const auto visual = computeISPFieldVisualization(ISP, 1.0, 1.0);

  // Convert visualiation to ROS message.
  const auto viz_msg =
      cv_bridge::CvImage(msg->header, "bgr8", visual).toImageMsg();

  // Convert to ROS message and publish
  if (!params_.viz_cisp_field_topic.empty()) {
    viz_cisp_field_pub.publish(viz_msg);
  }
}
}  // namespace maeve_automation_core
