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
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <tuple>
#include <vector>

namespace maeve_automation_core {

/**
  * @brief Perform temporal segmentation and tracking based on feature
  * correlation.
  */
class FeatureFlow {
 public:
  /**
   * @brief Callback for image frame processing.
   *
   * @param msg The ROS image message.
   */
  void callback(const sensor_msgs::Image::ConstPtr& msg);

 private:
  /** @name Segmentation and Track Information
   * These array are indexed aligned and contain object information.
   * @{
   */
  /** @brief List of homographies between previous and current frames. */
  std::vector<cv::Mat> homographies;
  /** @brief List of keypoints of previous frame. */
  std::vector<cv::KeyPoint> keypoints_prv;
  /** @brief List of keypoints of current frame. */
  std::vector<cv::KeyPoint> keypoints_cur;
  /** @brief List of descriptors of previous frame. */
  cv::Mat descriptors_prv;
  /** @brief List of descriptors of current frame. */
  cv::Mat descriptors_cur;
  /** @} */
};  // class FeaturFlow

}  // namespace maeve_automation_core
