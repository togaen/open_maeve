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

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/TransformStamped.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <tf2_ros/transform_listener.h>
#include <Eigen/Dense>
#include <boost/optional.hpp>
#include <opencv2/opencv.hpp>

#include <array>
#include <string>
#include <unordered_map>

#include "ar_cisp_field/params.h"
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

  /**
   * @brief Callback for the image message stream.
   *
   * @param msg The ROS image message.
   */
  void callback(const sensor_msgs::Image::ConstPtr& msg);

  /**
   * @brief Stub function to compute potential field.
   *
   * @param timestamp The timestamp for which to compute the potential field.
   */
  void computePotentialField(const ros::Time& timestamp);

 private:
  /** @brief Set of points describing an AR tag. */
  typedef std::array<Eigen::Vector3d, 4> AR_Points;
  /** @brief AR frame -> transform map. */
  typedef std::unordered_map<std::string, boost::optional<Eigen::Affine3d>>
      TxMap;
  /** @brief AR frame -> time queue. */
  typedef std::unordered_map<std::string, MaeveTimeQueue<double>> AR_TimeQueue;

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

  /**
   * @brief Fill the AR tag transform map.
   *
   * For any tag that does not have a transform available, the mapping is set to
   * boost::none.
   *
   * @param timestamp The timestamp to use for transform queries.
   *
   * @return True if no errors were encountered during transform lookup;
   * otherwise false.
   */
  bool fillAR_TagTransforms(const ros::Time& timestamp);

  /** @brief Node parameters. */
  AR_CISPFieldParams params_;

  /** @brief Camera image subscriber. */
  image_transport::Subscriber camera_sub_;
  /** @brief CISP field visualization publisher. */
  image_transport::Publisher viz_cisp_field_pub_;
  /** @brief The ROS node handle. */
  ros::NodeHandle nh_;
  /** @brief Buffer for listening to tf2 transforms. */
  tf2_ros::Buffer tf2_buffer_;
  /** @brief Listener for tf2 transforms. */
  tf2_ros::TransformListener tf2_listener_;
  /** @brief Mapping of AR tag frame id to transform. */
  TxMap ar_tag_transforms_;
  /** @brief Mapping of AR tag frame id to time queue of max extents. */
  AR_TimeQueue ar_max_extent_time_queue_;
  /** @brief Tag-relative set of corner points. */
  AR_Points ar_corner_points_;
};  // class AR_CISPFieldNodeHandler
}  // namespace maeve_automation_core
