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
#include "ar_isp_field/node_handler.h"

#include <cv_bridge/cv_bridge.h>
#include <tf2_eigen/tf2_eigen.h>
#include <boost/optional.hpp>

#include <algorithm>
#include <limits>
#include <string>
#include <tuple>
#include <vector>

#include "ar_isp_field/geometry.h"
#include "maeve_automation_core/isp_controller/ros_interface.h"
#include "maeve_automation_core/isp_field/tau.h"
#include "maeve_automation_core/isp_field/visualize.h"

namespace maeve_automation_core {
namespace {
static const auto NaN = std::numeric_limits<double>::quiet_NaN();
static const auto INF = std::numeric_limits<double>::infinity();
}  // namespace

std::vector<std::string> AR_ISPFieldNodeHandler::initializeTimeQueues(
    const std::vector<int>& id_list) {
  std::vector<std::string> frame_names;
  frame_names.reserve(id_list.size());

  std::for_each(std::begin(id_list), std::end(id_list), [&](const int id) {
    const auto ar_frame_name = params_.ar_frame_prefix + std::to_string(id);
    frame_names.push_back(ar_frame_name);
    ar_max_extent_time_queue_[ar_frame_name] = MaeveTimeQueue<double>(
        params_.ar_time_queue_size, params_.ar_time_queue_max_gap);
  });

  return frame_names;
}

AR_ISPFieldNodeHandler::AR_ISPFieldNodeHandler(const std::string& node_name)
    : nh_(node_name), tf2_listener_(tf2_buffer_) {
  if (!params_.load(nh_)) {
    ROS_FATAL_STREAM("Failed to load parameters. Fatal error.");
    return;
  }

  // Set up frame list and time queues.
  ar_obstacle_tag_frames_ = initializeTimeQueues(params_.ar_tag_obstacle_ids);
  ar_target_tag_frames_ = initializeTimeQueues(params_.ar_tag_target_ids);

  // Compute raw tag corner points (CW ordering).
  const auto half_extent = params_.ar_tag_size / 2.0;
  ar_corner_points_[0] = Eigen::Vector3d(half_extent, -half_extent, 0.0);
  ar_corner_points_[1] = Eigen::Vector3d(half_extent, half_extent, 0.0);
  ar_corner_points_[2] = Eigen::Vector3d(-half_extent, half_extent, 0.0);
  ar_corner_points_[3] = Eigen::Vector3d(-half_extent, -half_extent, 0.0);

  // Instantiate transforms.
  hc_ = PotentialTransform<ConstraintType::HARD>(
      params_.hard_constraint_transform);
  sc_ = PotentialTransform<ConstraintType::SOFT>(
      params_.soft_constraint_transform);

  // Image transport interface.
  image_transport::ImageTransport it(nh_);

  // Register callback.
  camera_sub_ = it.subscribeCamera(
      params_.camera_topic, 1, &AR_ISPFieldNodeHandler::cameraCallback, this);

  // Set up command handler.
  command2d_mgr_.initialize(nh_, params_.control_command_input_topic);

  // Control command output publisher.
  control_command_output_pub_ =
      nh_.advertise<controller_interface_msgs::Command2D>(
          params_.control_command_output_topic, 1);

  // Visualize?
  if (!params_.viz_isp_field_topic.empty()) {
    viz_isp_field_pub_ = it.advertise(params_.viz_isp_field_topic, 1);
  }
}

std::vector<cv::Point2d> AR_ISPFieldNodeHandler::projectPoints(
    const Eigen::Affine3d& camera_T_artag) const {
  // Compute the corner points under the given transform.
  const auto corner_points =
      arTagCornerPoints(camera_T_artag, ar_corner_points_);

  // Allocate return structure.
  std::vector<cv::Point2d> image_points;
  image_points.reserve(corner_points.size());

  // Convert Eigen -> OpenCV.
  const auto camera_points = arEigenPoints2OpenCV(corner_points);

  // Project camera_points into image_points.
  for (auto i = 0; i < 4; ++i) {
    image_points.push_back(camera_model_.project3dToPixel(camera_points[i]));
  }

  // Done.
  return image_points;
}

boost::optional<std::tuple<Eigen::Affine3d, double>>
AR_ISPFieldNodeHandler::getTransformAndStamp(const std::string& ar_tag_frame,
                                             const ros::Time& timestamp) const {
  Eigen::Affine3d T;
  ros::Time T_timestamp;
  try {
    const auto T_msg = tf2_buffer_.lookupTransform(params_.camera_frame_name,
                                                   ar_tag_frame, ros::Time(0));
    T = tf2::transformToEigen(T_msg);
    T_timestamp = T_msg.header.stamp;
    const auto age = (timestamp - T_timestamp).toSec();

    // If the transform is too old, the detection is stale.
    if (age > params_.ar_tag_max_age) {
      return boost::none;
    }
  } catch (const tf2::TransformException& ex) {
    return boost::none;
  }

  return std::make_tuple(T, T_timestamp.toSec());
}

bool AR_ISPFieldNodeHandler::computePotentialFields(
    const ros::Time& timestamp, const ConstraintType& constraint_type,
    FieldMap& field_map) {
  auto updated = false;

  // Compute max extents for each AR tag and add to time queues.
  std::for_each(
      std::begin(field_map), std::end(field_map),
      [&](FieldMap::value_type& pair) {
        // Get references and initialize field.
        const auto& frame_name = pair.first;
        auto& field = pair.second;
        field.setTo(0.0);

        // Only use valid transforms.
        auto t = NaN;
        Eigen::Affine3d camera_T_artag = Eigen::Affine3d::Identity();
        if (const auto vals = getTransformAndStamp(frame_name, timestamp)) {
          std::tie(camera_T_artag, t) = *vals;
        } else {
          return;
        }

        // Project AR tag onto image plane.
        const auto image_corner_points = projectPoints(camera_T_artag);

        // Get max extent and add to time queue.
        const auto s = arComputeMaxXY_Extent(image_corner_points);
        ar_max_extent_time_queue_[frame_name].insert(t, s);

        // Compute measurement values.
        // TODO(me): should put a filter on these dt values.
        auto s_dot = NaN;
        auto t_delta = NaN;
        if (const auto dt = ar_max_extent_time_queue_[frame_name].bfd_dt(t)) {
          std::tie(t_delta, s_dot) = *dt;
        } else {
          // If the backward differencing operation fails probably the queue has
          // recently become empty. It's expected and probably not an error.
          return;
        }
        const auto tau = tauFromDiscreteScaleDt(s, s_dot, t_delta);
        const auto tau_dot = 0.0;

        // Compute potential values.
        const auto p_value = (constraint_type == ConstraintType::HARD)
                                 ? hc_(cv::Point2d(tau, tau_dot))
                                 : sc_(cv::Point2d(tau, tau_dot));

        // Print output?
        if (params_.verbose && std::isfinite(tau)) {
          ROS_INFO_STREAM(constraint_type << " :" << frame_name << " - tau: "
                                          << tau << ", tau_dot: " << tau_dot);
          ROS_INFO_STREAM(constraint_type << " :" << frame_name
                                          << " - k0: " << p_value.x
                                          << ", k1: " << p_value.y);
        }

        // Fill ISP.
        arFillISP(p_value, image_corner_points, field);

        // Mark that at least one potential field has been computed.
        updated = true;
      });

  // Done.
  return updated;
}

void AR_ISPFieldNodeHandler::initFieldStorage(
    const cv::Size& size, const std::vector<std::string>& frame_list,
    FieldMap& field_map) {
  std::for_each(std::begin(frame_list), std::end(frame_list),
                [&](const std::string& frame_name) {
                  field_map[frame_name] = cv::Mat::zeros(size, CV_64FC2);
                });
}

void AR_ISPFieldNodeHandler::cameraCallback(
    const sensor_msgs::Image::ConstPtr& msg,
    const sensor_msgs::CameraInfoConstPtr& info_msg) {
  // Make sure field maps have storage allocated, but do it only once.
  static bool init = true;
  if (init) {
    // Initialize camera model.
    camera_model_.fromCameraInfo(info_msg);

    // Initialize ISP controller.
    params_.isp_controller_params.principal_point_x = camera_model_.cx();
    params_.isp_controller_params.focal_length_x = camera_model_.fx();
    isp_controller_ = ISP_Controller(params_.isp_controller_params);

    // Initialize storage.
    initFieldStorage(camera_model_.fullResolution(), ar_obstacle_tag_frames_,
                     obstacle_field_map_);
    initFieldStorage(camera_model_.fullResolution(), ar_target_tag_frames_,
                     target_field_map_);
    init = false;
  }

  // Compute a potential field for each tag.
  static ros::Time time_of_last_update = msg->header.stamp;
  const auto age = (msg->header.stamp - time_of_last_update).toSec();
  const auto forget_tracks = (age > params_.ar_tag_max_age);
  if (!forget_tracks) {
    const auto obstacles_updated = computePotentialFields(
        msg->header.stamp, ConstraintType::HARD, obstacle_field_map_);
    const auto targets_updated = computePotentialFields(
        msg->header.stamp, ConstraintType::SOFT, target_field_map_);
    if (!obstacles_updated && !targets_updated) {
      return;
    }
  }
  time_of_last_update = msg->header.stamp;

  // Compose fields into an ISP.
  cv::Mat ISP = computeISP();

  // Do any requested visualization.
  visualize(ISP, msg->header);

  // Get most recent desired control.
  ControlCommand u_d;
  if (const auto cmd_msg = command2d_mgr_.mostRecentMsg()) {
    u_d = command2D_Msg2ControlCommand(*cmd_msg);
  } else {
    u_d = params_.default_guidance_control;
  }

  // Compute SD control.
  const auto u_star = isp_controller_.SD_Control(ISP, u_d);
  // ROS_INFO_STREAM("u_d: " << u_d << ", u_star: " << u_star);

  // Publish control.
  control_command_output_pub_.publish(
      controlCommand2Command2D_Msg(u_star, msg->header));
}

cv::Mat AR_ISPFieldNodeHandler::computeISP() const {
  // Initialize to zeroed field.
  cv::Mat ISP = cv::Mat::zeros(camera_model_.fullResolution(), CV_64FC2);

  // Compose hard constraint field.
  std::for_each(
      std::begin(obstacle_field_map_), std::end(obstacle_field_map_),
      [&](const FieldMap::value_type& pair) { ISP = ISP + pair.second; });

  // Compose soft constraint field.
  std::for_each(
      std::begin(target_field_map_), std::end(target_field_map_),
      [&](const FieldMap::value_type& pair) { ISP = ISP + pair.second; });

  // Done.
  return ISP;
}

void AR_ISPFieldNodeHandler::visualize(const cv::Mat& ISP,
                                       const std_msgs::Header& header) const {
  return;
  // If no topic, nothing to do.
  if (params_.viz_isp_field_topic.empty()) {
    return;
  }

  // Compute visualization of ISP.
  const auto visual = computeISPFieldVisualization(
      ISP, params_.viz_potential_bounds[0], params_.viz_potential_bounds[1]);

  // Convert visualization to ROS message.
  const auto viz_msg = cv_bridge::CvImage(header, "bgr8", visual).toImageMsg();

  // Publish.
  viz_isp_field_pub_.publish(viz_msg);
}
}  // namespace maeve_automation_core
