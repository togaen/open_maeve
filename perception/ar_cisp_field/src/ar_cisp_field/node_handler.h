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
#include <opencv2/opencv.hpp>

#include <array>
#include <string>
#include <unordered_map>
#include <vector>

#include "ar_cisp_field/params.h"
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

 private:
  /** @brief Set of points describing an AR tag. */
  typedef std::array<Eigen::Vector3d, 4> AR_Points;
  /** @brief AR frame -> time queue. */
  typedef std::unordered_map<std::string, MaeveTimeQueue<double>>
      AR_TimeQueueMap;
  /** @brief AR frame -> scalar field. */
  typedef std::unordered_map<std::string, cv::Mat> FieldMap;

  /**
   * @brief Initialize the field storage map.
   *
   * This function only performs the allocation once; subsequent calls are a
   * no-op.
   *
   * @param size The size of the field to allocate.
   */
  void initFieldStorage(const cv::Size& size);

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
   */
  void computePotentialFields(const ros::Time& timestamp);

  /**
   * @brief Convert an array of Eigen::Vector3d points to cv::Point3d points.
   *
   * @param points The array of Eigen::Vector3d points.
   *
   * @return A vector of projected cv::Point3d objects.
   */
  static std::vector<cv::Point3d> arEigenPoints2OpenCV(const AR_Points& points);

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

  /**
   * @brief Compute the maximum extent in the XY projection of the given set of
   * AR tag points.
   *
   * @param points The AR tag corner points projected onto the image plane.
   *
   * @return The maximum extent.
   */
  static double computeMaxXY_Extent(const std::vector<cv::Point2d>& points);

  /**
   * @brief From a given AR tag pose, compute its corner point.
   *
   * This method assumes the pose is centered on the tag.
   *
   * @param Tx The AR tag pose.
   *
   * @return An array of corner points in CW order (assuming right hand system,
   * looking down at XY plane).
   */
  AR_Points arTagCornerPoints(const Eigen::Affine3d& Tx) const;

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
  /** @brief List of AR tag frame ids. */
  std::vector<std::string> ar_tag_frames_;
  /** @brief Mapping of AR tag frame id to time queue of max extents. */
  AR_TimeQueueMap ar_max_extent_time_queue_;
  /** @brief Mapping of AR tag frame id to scalar field. */
  FieldMap field_map_;
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
