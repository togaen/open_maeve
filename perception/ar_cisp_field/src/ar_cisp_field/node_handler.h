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

#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>

#include <array>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

#include "ar_cisp_field/params.h"
#include "maeve_automation_core/ar_cisp_field/geometry.h"
#include "maeve_automation_core/cisp_field/potential_transforms.h"
#include "maeve_automation_core/maeve_time_queue/maeve_time_queue.h"

namespace maeve_automation_core {
/**
 * @brief Interface between ROS and the expansion segmentation libraries.
 */
class AR_CISPFieldNodeHandler {
 public:
  /**
   * @brief Construct handler instance and register callbacks/subscribers.
   *
   * @param node_name The node name used to construct the ROS node handle.
   */
  explicit AR_CISPFieldNodeHandler(const std::string& node_name);

  /** @brief AR frame -> time queue. */
  typedef std::unordered_map<std::string, MaeveTimeQueue<double>>
      AR_TimeQueueMap;
  /** @brief AR frame -> scalar field. */
  typedef std::unordered_map<std::string, cv::Mat> FieldMap;

  /**
   * @brief Get and return an AR tag transform and its timestamp.
   *
   * This function retrieves the camera_T_ar_tag transform, convert it to an
   * Eigen data type and returns the Eigen transform along with the timestamp of
   * the transform. If the transform doesn't exist or violates age limits, a
   * null object is returned.
   *
   * @param ar_tag_frame The name of the transform to look up.
   * @param timestamp The time at which the function is called; used for
   * checking age.
   *
   * @return The transform along with its timestamp, or boost::none on error or
   * age violation.
   */
  boost::optional<std::tuple<Eigen::Affine3d, ros::Time>> getTransformAndStamp(
      const std::string& ar_tag_frame, const ros::Time& timestamp) const;

  /**
   * @brief Set up time queues for obstacle and target AR tags.
   *
   * In addition to setting up time queues for all AR tags, this function
   * computes and records frame names for looking up tf2 transforms.
   *
   * @param id_list A list of AR tag ids.
   *
   * @return A list of frame names corresponding to the tag IDs.
   */
  std::vector<std::string> initializeTimeQueues(
      const std::vector<int>& id_list);

  /**
   * @brief Initialize the field storage map.
   *
   * @param size The size of the field to allocate.
   * @param frame_list The list of frames to initialize fields for.
   * @param field_map The container for the initialized fields.
   */
  void initFieldStorage(const cv::Size& size,
                        const std::vector<std::string>& frame_list,
                        FieldMap& field_map);

  /**
   * @brief Callback for the camera info and image message stream.
   *
   * @param msg The ROS image message.
   */
  void cameraCallback(const sensor_msgs::Image::ConstPtr& msg,
                      const sensor_msgs::CameraInfoConstPtr& info_msg);

  /**
   * @brief Stub function to compute potential field.
   *
   * @param timestamp The timestamp for which to compute the potential field.
   * @param constraint_type The type of potential transform to use.
   * @param field_map The set of fields for which to compute potentials.
   *
   * @return True if at least one potential field has been computed; otherwise
   * false.
   */
  bool computePotentialFields(const ros::Time& timestamp,
                              const ConstraintType& constraint_type,
                              FieldMap& field_map);

  /**
   * @brief Convenience wrapper for using camera model to project points.
   *
   * This function handles translating arguments so that the projection method
   * of the camera model can be called.
   *
   * @param ar_points The set of 3D ar tag corner points in the camera frame.
   *
   * @return The set of 2D image plane points.
   */
  std::vector<cv::Point2d> projectPoints(const AR_Points& ar_points) const;

  /** @brief Node parameters. */
  AR_CISPFieldParams params_;

  /** @brief Camera image subscriber. */
  image_transport::CameraSubscriber camera_sub_;
  /** @brief CISP field visualization publisher. */
  image_transport::Publisher viz_cisp_field_pub_;
  /** @brief The ROS node handle. */
  ros::NodeHandle nh_;
  /** @brief Buffer for listening to tf2 transforms. */
  tf2_ros::Buffer tf2_buffer_;
  /** @brief Listener for tf2 transforms. */
  tf2_ros::TransformListener tf2_listener_;
  /** @brief List of obstacle AR tag frame ids. */
  std::vector<std::string> ar_obstacle_tag_frames_;
  /** @brief List of target AR tag frame ids. */
  std::vector<std::string> ar_target_tag_frames_;
  /** @brief Mapping of AR tag frame id to time queue of max extents. */
  AR_TimeQueueMap ar_max_extent_time_queue_;
  /** @brief Mapping of AR obstacle tag frame id to scalar field. */
  FieldMap obstacle_field_map_;
  /** @brief Mapping of AR target tag frame id to scalar field. */
  FieldMap target_field_map_;
  /** @brief Tag-relative set of corner points. */
  AR_Points ar_corner_points_;
  /** @brief Camera model used for projecting AR tag points into image plane. */
  image_geometry::PinholeCameraModel camera_model_;
  /** @brief The hard constraint transform. */
  PotentialTransform<ConstraintType::HARD> hc_;
  /** @brief The soft constraint transform. */
  PotentialTransform<ConstraintType::SOFT> sc_;
};  // class AR_CISPFieldNodeHandler
}  // namespace maeve_automation_core
