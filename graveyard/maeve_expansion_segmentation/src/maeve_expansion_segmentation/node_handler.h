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
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <opencv2/opencv.hpp>

#include <memory>

#include "open_maeve/maeve_expansion_segmentation/connected_component_tracking.h"
#include "maeve_expansion_segmentation/params.h"

namespace open_maeve {
/**
 * @brief Interface between ROS and the expansion segmentation libraries.
 */
class MaeveExpansionSegmentationNodeHandler {
 public:
  /**
   * @brief Construct handler instance and register callbacks/subscribers.
   *
   * @param nh The ROS node handle.
   */
  explicit MaeveExpansionSegmentationNodeHandler(const ros::NodeHandle& nh);

  /**
   * @brief Callback for the image message stream.
   *
   * @param msg The ROS image message.
   */
  void callback(const sensor_msgs::Image::ConstPtr& msg);

 private:
  /**
   * @brief Publish visualizations of edge detections with given header.
   *
   * @param header The ROS message header to publish with.
   * @param te_image The OpenCV temporal image detection.
   * @param se_image The OpenCV spatial image detection.
   * @param AND_image The OpenCV AND image.
   */
  void visualize(const std_msgs::Header& header, const cv::Mat& te_image,
                 const cv::Mat& se_image, const cv::Mat& AND_image);

  /** @brief Node parameters. */
  MaeveExpansionSegmentationParams params_;

  /** @brief Pointer to background subtraction operator. */
  cv::Ptr<cv::BackgroundSubtractor> bg_subtractor_ptr_;

  /** @brief Connected component tracker. */
  std::unique_ptr<ConnectedComponentTracker> cc_tracker_ptr_;

  /** @brief Camera image subscriber. */
  image_transport::Subscriber camera_sub;
  /** @brief Temporal edge visualization publisher. */
  image_transport::Publisher viz_te_pub;
  /** @brief Spatial edge visualization publisher. */
  image_transport::Publisher viz_se_pub;
  /** @brief AND visualization publisher. */
  image_transport::Publisher viz_AND_pub;
};  // class MaeveExpansionSegmentationNodeHandler
}  // namespace open_maeve
