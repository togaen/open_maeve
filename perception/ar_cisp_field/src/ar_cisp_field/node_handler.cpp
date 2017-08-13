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
#include <tf2_eigen/tf2_eigen.h>

#include <string>

#include "maeve_automation_core/cisp_field/isp.h"
#include "maeve_automation_core/cisp_field/potential_transforms.h"
#include "maeve_automation_core/cisp_field/visualize.h"

namespace maeve_automation_core {
AR_CISPFieldNodeHandler::AR_CISPFieldNodeHandler(const std::string& node_name)
    : nh_(node_name), tf2_listener_(tf2_buffer_) {
  if (!params_.load(nh_)) {
    ROS_FATAL_STREAM("Failed to load parameters. Fatal error.");
    return;
  }

  // Set up transform storage.
  std::for_each(
      std::begin(params_.ar_tag_ids), std::end(params_.ar_tag_ids),
      [&](const int id) {
        const auto ar_frame_name = params_.ar_frame_prefix + std::to_string(id);
        ar_tag_transforms_[ar_frame_name] = boost::none;
        ar_max_extent_time_queue_[ar_frame_name] = MaeveTimeQueue<double>(
            params_.ar_time_queue_size, params_.ar_time_queue_max_gap);
      });

  // Compute raw tag corner points (CW ordering).
  const auto half_extent = params_.ar_tag_size / 2.0;
  ar_corner_points_[0] = Eigen::Vector3d(half_extent, -half_extent, 0.0);
  ar_corner_points_[1] = Eigen::Vector3d(half_extent, half_extent, 0.0);
  ar_corner_points_[2] = Eigen::Vector3d(-half_extent, half_extent, 0.0);
  ar_corner_points_[3] = Eigen::Vector3d(-half_extent, -half_extent, 0.0);

  // Image transport interface.
  image_transport::ImageTransport it(nh_);

  // Register callback.
  camera_sub_ = it.subscribe(params_.camera_topic, 1,
                             &AR_CISPFieldNodeHandler::callback, this);

  // Visualize?
  if (!params_.viz_cisp_field_topic.empty()) {
    viz_cisp_field_pub_ = it.advertise(params_.viz_cisp_field_topic, 1);
  }
}

AR_CISPFieldNodeHandler::AR_Points AR_CISPFieldNodeHandler::arTagCornerPoints(
    const Eigen::Affine3d& camera_T_tag) const {
  AR_Points points;

  // Transform each of ar_corner_points_ into points.
  for (auto i = 0; i < 4; ++i) {
    points[i] = camera_T_tag * ar_corner_points_[i];
  }

  return points;
}

bool AR_CISPFieldNodeHandler::fillAR_TagTransforms(const ros::Time& timestamp) {
  if (!nh_.ok()) {
    return false;
  }

  // Look up all AR tag transforms.
  std::for_each(std::begin(ar_tag_transforms_), std::end(ar_tag_transforms_),
                [&](TxMap::value_type& pair) {
                  try {
                    pair.second =
                        tf2::transformToEigen(tf2_buffer_.lookupTransform(
                            params_.camera_frame_name, pair.first, timestamp));
                  } catch (const tf2::TransformException& ex) {
                    // ROS_WARN_STREAM(ex.what());
                    pair.second = boost::none;
                  }
                });

  return true;
}

void AR_CISPFieldNodeHandler::computePotentialField(
    const ros::Time& timestamp) {
  ros::Rate rate(params_.measurement_field_publish_rate);

  while (fillAR_TagTransforms(timestamp)) {
    // Project AR tag onto image plane
    // Get max extent
    // Add to time queue
    // Compute \dot{s} and \ddot{s} from queue
    // Generate field
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
    viz_cisp_field_pub_.publish(viz_msg);
  }
}
}  // namespace maeve_automation_core
