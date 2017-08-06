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
#include "maeve_expansion_segmentation/node_handler.h"

#include <ros/ros.h>

namespace maeve_automation_core {
MaeveExpansionSegmentationNodeHandler::MaeveExpansionSegmentationNodeHandler(
    const ros::NodeHandle& nh)
    : cc_tracker_ptr_(nullptr) {
  if (!params_.load(nh)) {
    ROS_FATAL_STREAM("Failed to load parameters. Fatal error.");
    return;
  }

  // Create tracker.
  cc_tracker_ptr_ = std::unique_ptr<ConnectedComponentTracker>(
      new ConnectedComponentTracker(params_.connected_component_params));

  // Create background subtractor.
  if (params_.temporal_params.history > 0) {
    bg_subtractor_ptr_ = cv::createBackgroundSubtractorMOG2(
        params_.temporal_params.history, params_.temporal_params.threshold,
        params_.temporal_params.shadows);
  }

  // Image transport interface.
  image_transport::ImageTransport it(nh);

  // Register callback.
  camera_sub =
      it.subscribe(params_.camera_topic, 1,
                   &MaeveExpansionSegmentationNodeHandler::callback, this);

  // Visualize?
  if (params_.enable_viz) {
    viz_te_pub = it.advertise(params_.viz_te_topic, 1);
    viz_se_pub = it.advertise(params_.viz_se_topic, 1);
    viz_AND_pub = it.advertise(params_.viz_AND_topic, 1);
  }
}

void MaeveExpansionSegmentationNodeHandler::visualize(
    const std_msgs::Header& header, const cv::Mat& te_image,
    const cv::Mat& se_image, const cv::Mat& AND_image) {
  if (!params_.enable_viz) {
    return;
  }

  const auto te_msg =
      cv_bridge::CvImage(header, "mono8", te_image).toImageMsg();
  const auto se_msg =
      cv_bridge::CvImage(header, "mono8", se_image).toImageMsg();
  const auto AND_msg =
      cv_bridge::CvImage(header, "mono8", AND_image).toImageMsg();
  viz_te_pub.publish(te_msg);
  viz_se_pub.publish(se_msg);
  viz_AND_pub.publish(AND_msg);
}

namespace {
cv::Mat buildStructuringElement(
    const MaeveExpansionSegmentationParams::MorphologicalParams morpho_params) {
  const auto element_type =
      morpho_params.element_type == 0 ? cv::MORPH_RECT : cv::MORPH_ELLIPSE;
  const auto window_size =
      cv::Size(morpho_params.window_width, morpho_params.window_height);
  const auto anchor_point = cv::Point(-1, -1);  // centered
  return cv::getStructuringElement(element_type, window_size, anchor_point);
}
}  // namespace

void MaeveExpansionSegmentationNodeHandler::callback(
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

  // Generate spatial feature image.
  cv::Mat se_image;
  cv::Canny(cv_ptr->image, se_image, params_.spatial_params.edge_min,
            params_.spatial_params.edge_max,
            params_.spatial_params.edge_aperture);

  // Generate temporal feature image.
  cv::Mat te_image;
  if (bg_subtractor_ptr_) {
    bg_subtractor_ptr_->apply(cv_ptr->image, te_image);
  } else {
    se_image.copyTo(te_image);
  }

  // Build structuring elements for morphological operations.
  const auto erosion_structuring_element =
      buildStructuringElement(params_.erosion_params);
  const auto dilation_structuring_element =
      buildStructuringElement(params_.dilation_params);

  // Apply morphological operators?
  cv::Mat te_image_morph;
  cv::Mat te_image_accum;
  te_image.copyTo(te_image_accum);
  std::for_each(params_.morpho_operations.begin(),
                params_.morpho_operations.end(), [&](const int op) {
                  switch (op) {
                    case 0:
                      cv::erode(te_image_accum, te_image_morph,
                                erosion_structuring_element);
                      te_image_morph.copyTo(te_image_accum);
                      break;
                    case 1:
                      cv::dilate(te_image_accum, te_image_morph,
                                 dilation_structuring_element);
                      te_image_morph.copyTo(te_image_accum);
                      break;
                    default:
                      break;
                  }
                });

  // Apply spatial blurring to temporal features?
  cv::Mat te_image_blurred;
  if (params_.spatial_params.blur_aperture < 0) {
    te_image_blurred = te_image_morph;
  } else {
    cv::medianBlur(te_image_morph, te_image_blurred,
                   params_.spatial_params.blur_aperture);
  }

  // Do tracking.
  cv::Mat AND_image;
  if (cc_tracker_ptr_) {
    cc_tracker_ptr_->addEdgeFrame(te_image_blurred);
    // Draw connected components.
    AND_image = cv::Mat::zeros(te_image_blurred.rows, te_image_blurred.cols,
                               te_image_blurred.type());
    const auto& cc = cc_tracker_ptr_->getFrameInfoBuffer();
    std::for_each(
        cc.begin(), cc.end(),
        [&](const ConnectedComponentTracker::FrameInfo& frame_info) {
          std::for_each(
              frame_info.contours.begin(), frame_info.contours.end(),
              [&](const ConnectedComponentTracker::ContourInfo& contour) {
                cv::bitwise_or(contour.component, AND_image, AND_image);
              });
        });
  } else {
    // Just curious...
    cv::Mat AND_se;
    cv::bitwise_and(cv_ptr->image, se_image, AND_se);
    cv::bitwise_and(AND_se, te_image_blurred, AND_image);
  }

  // Publish images.
  visualize(msg->header, te_image_blurred, se_image, AND_image);
}
}  // namespace maeve_automation_core
