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

#include <cv_bridge/cv_bridge.h>
#include <tf2_eigen/tf2_eigen.h>

#include <algorithm>
#include <limits>
#include <string>
#include <vector>

#include "maeve_automation_core/ar_cisp_field/geometry.h"
#include "maeve_automation_core/cisp_field/tau.h"
#include "maeve_automation_core/cisp_field/visualize.h"

namespace maeve_automation_core {
namespace {
static const auto INF = std::numeric_limits<double>::infinity();
}  // namespace

void AR_CISPFieldNodeHandler::initializeTimeQueues(
    const std::vector<int>& id_list) {
  std::for_each(std::begin(id_list), std::end(id_list), [&](const int id) {
    const auto ar_frame_name = params_.ar_frame_prefix + std::to_string(id);
    ar_tag_frames_.insert(ar_frame_name);
    ar_max_extent_time_queue_[ar_frame_name] = MaeveTimeQueue<double>(
        params_.ar_time_queue_size, params_.ar_time_queue_max_gap);
  });
}

AR_CISPFieldNodeHandler::AR_CISPFieldNodeHandler(const std::string& node_name)
    : nh_(node_name), tf2_listener_(tf2_buffer_) {
  if (!params_.load(nh_)) {
    ROS_FATAL_STREAM("Failed to load parameters. Fatal error.");
    return;
  }

  // Set up frame list and time queues.
  initializeTimeQueues(params_.ar_tag_obstacle_ids);
  initializeTimeQueues(params_.ar_tag_target_ids);

  // Compute raw tag corner points (CW ordering).
  const auto half_extent = params_.ar_tag_size / 2.0;
  ar_corner_points_[0] = Eigen::Vector3d(half_extent, -half_extent, 0.0);
  ar_corner_points_[1] = Eigen::Vector3d(half_extent, half_extent, 0.0);
  ar_corner_points_[2] = Eigen::Vector3d(-half_extent, half_extent, 0.0);
  ar_corner_points_[3] = Eigen::Vector3d(-half_extent, -half_extent, 0.0);

  // Instantiate transforms.
  const auto& hc_params = params_.hard_constraint_transform;
  const auto& sc_params = params_.soft_constraint_transform;
  hc_ = PotentialTransform<ConstraintType::HARD>(
      hc_params.range_min, hc_params.range_max, hc_params.alpha,
      hc_params.beta);
  sc_ = PotentialTransform<ConstraintType::SOFT>(
      sc_params.range_min, sc_params.range_max, sc_params.alpha,
      sc_params.beta);

  // Image transport interface.
  image_transport::ImageTransport it(nh_);

  // Register callback.
  camera_sub_ = it.subscribeCamera(
      params_.camera_topic, 1, &AR_CISPFieldNodeHandler::cameraCallback, this);

  // Visualize?
  if (!params_.viz_cisp_field_topic.empty()) {
    // ROS_INFO_STREAM("publishing visualization");
    viz_cisp_field_pub_ = it.advertise(params_.viz_cisp_field_topic, 1);
  }
}

std::vector<cv::Point2d> AR_CISPFieldNodeHandler::projectPoints(
    const AR_Points& ar_points) const {
  std::vector<cv::Point2d> image_points;
  image_points.reserve(ar_points.size());

  // Convert Eigen -> OpenCV.
  const auto camera_points = arEigenPoints2OpenCV(ar_points);

  // Project camera_points into image_points.
  for (auto i = 0; i < 4; ++i) {
    image_points.push_back(camera_model_.project3dToPixel(camera_points[i]));
  }

  // Done.
  return image_points;
}

void AR_CISPFieldNodeHandler::computePotentialFields(
    const ros::Time& timestamp) {
  // Compute max extents for each AR tag and add to time queues.
  std::for_each(
      std::begin(ar_tag_frames_), std::end(ar_tag_frames_),
      [&](const std::string& frame_name) {
        // Initialize the potential field for this tag by zeroing it
        // out.
        auto& field = field_map_[frame_name];
        field.setTo(0.0);

        // Only use valid transforms.
        Eigen::Affine3d T;
        ros::Time T_timestamp;
        try {
          const auto T_msg = tf2_buffer_.lookupTransform(
              params_.camera_frame_name, frame_name, ros::Time(0));
          T = tf2::transformToEigen(T_msg);
          T_timestamp = T_msg.header.stamp;
          const auto age = (timestamp - T_timestamp).toSec();

          // If the transform is too old, the detection is stale.
          if (age > params_.ar_tag_max_age) {
            return;
          }
        } catch (const tf2::TransformException& ex) {
          // ROS_WARN_STREAM(ex.what());
          return;
        }

        // Project AR tag onto image plane
        const auto camera_points = arTagCornerPoints(T, ar_corner_points_);
        const auto image_corner_points = projectPoints(camera_points);
        // Get max extent
        const auto s = arComputeMaxXY_Extent(image_corner_points);
        // Add to time queue
        const auto t = T_timestamp.toSec();
        ar_max_extent_time_queue_[frame_name].insert(t, s);
        // With time queue full, compute measurement values.
        // TODO: should put a filter on these dt values.
        const auto dt = ar_max_extent_time_queue_[frame_name].dt(t);
        if (!dt) {
          // If this happens, that backward differencing operation
          // failed; probably the queue has recently become empty.
          // It's expected and probably not an error.
          return;
        }
        const auto t_delta = std::get<0>(*dt);
        const auto s_dot = std::get<1>(*dt);
        const auto tau = tauFromDiscreteScaleDt(s, s_dot, t_delta);
        const auto tau_dot = 0.0;

        // Compute potential values.
        const auto p_value = hc_(cv::Scalar(tau, tau_dot));

        // Print output?
        if (params_.verbose && std::isfinite(tau)) {
          ROS_INFO_STREAM(frame_name << " - tau: " << tau
                                     << ", tau_dot: " << tau_dot);
          ROS_INFO_STREAM(frame_name << " - k0: " << p_value[0]
                                     << ", k1: " << p_value[1]);
        }

        // Create ISP.
        {
          std::vector<cv::Point2i> pts;
          pts.reserve(image_corner_points.size());
          std::for_each(std::begin(image_corner_points),
                        std::end(image_corner_points),
                        [&](const cv::Point2d& pt) {
                          pts.push_back(cv::Point2i(static_cast<int>(pt.x),
                                                    static_cast<int>(pt.y)));
                        });
          cv::fillConvexPoly(field, pts, p_value);
        }
      });
}

void AR_CISPFieldNodeHandler::initFieldStorage(const cv::Size& size) {
  static bool storage_set = false;
  if (!storage_set) {
    std::for_each(std::begin(ar_tag_frames_), std::end(ar_tag_frames_),
                  [&](const std::string& frame_name) {
                    field_map_[frame_name] = cv::Mat::zeros(size, CV_64FC2);
                  });
    storage_set = true;
  }
}

void AR_CISPFieldNodeHandler::cameraCallback(
    const sensor_msgs::Image::ConstPtr& msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // ROS_INFO_STREAM("entered callback");

  // Initialize camera model.
  camera_model_.fromCameraInfo(info_msg);

  // Make sure field maps have storage allocated.
  initFieldStorage(camera_model_.fullResolution());

  // Compute a potential field for each tag.
  computePotentialFields(msg->header.stamp);

  // Compose fields into a composite ISP.
  cv::Mat ISP = cv::Mat::zeros(camera_model_.fullResolution(), CV_64FC2);
  std::for_each(
      std::begin(field_map_), std::end(field_map_),
      [&](const FieldMap::value_type& pair) { ISP = ISP + pair.second; });

  // Visualize ISP.
  const auto visual = computeISPFieldVisualization(
      ISP, params_.viz_potential_bounds[0], params_.viz_potential_bounds[1]);

  // Convert visualiation to ROS message.
  const auto viz_msg =
      cv_bridge::CvImage(msg->header, "bgr8", visual).toImageMsg();

  // Convert to ROS message and publish
  if (!params_.viz_cisp_field_topic.empty()) {
    viz_cisp_field_pub_.publish(viz_msg);
  }
}
}  // namespace maeve_automation_core
