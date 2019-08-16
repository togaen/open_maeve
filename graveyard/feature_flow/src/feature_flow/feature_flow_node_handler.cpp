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
#include "feature_flow/feature_flow_node_handler.h"

#include <cv_bridge/cv_bridge.h>

namespace open_maeve {
FeatureFlowNodeHandler::FeatureFlowNodeHandler(const ros::NodeHandle& nh)
    : skipped_frames(0), feature_flow_ptr(nullptr) {
  if (!params.load(nh)) {
    ROS_FATAL_STREAM("Failed to load parameters. Fatal error.");
    return;
  }
  ROS_INFO_STREAM("Loaded:\n" << params);

  // Instantiate feature flow object.
  feature_flow_ptr = std::unique_ptr<FeatureFlow>(new FeatureFlow(params.ff));

  // Set up ROS graph interactions.
  image_transport::ImageTransport it(nh);
  camera_sub = it.subscribe(params.camera_topic, 1,
                            &FeatureFlowNodeHandler::callback, this);

  if (!params.viz_topic.empty()) {
    viz_pub = it.advertise(params.viz_topic, 1);
  }
}

void FeatureFlowNodeHandler::visualize(const std_msgs::Header& header) const {
  // If visualizations disabled, done.
  if (params.viz_topic.empty()) {
    return;
  }

  // Get the set of matches.
  const auto& H = feature_flow_ptr->homographyMatches();

  // 2D homogenous identity matrix; used for thresholding.
  const auto I = cv::Mat::eye(3, 3, CV_64F);

  // Draw matched keypoints in current frame.
  cv::Mat out_image;
  feature_flow_ptr->curFrame().copyTo(out_image);
  std::for_each(
      H.begin(), H.end(), [&](const FeatureFlow::HomographyMatches& matches) {
        // Get homography, check whether to ignore it.
        const auto T = std::get<0>(matches);
        const auto scale = FeatureFlow::getScaleComponents(T);
        const auto scale_mag = std::min(scale.x, scale.y);
        const auto d = cv::norm(T, I, cv::NORM_L1);
        const auto t_x = T.at<double>(0, 2);
        const auto t_y = T.at<double>(1, 2);
        const auto t_mag = std::sqrt(t_x * t_x + t_y * t_y);
        std::cout << "t_mag: " << t_mag << ", t_x: " << t_x << ", t_y: " << t_y
                  << std::endl;
        std::cout << "d: " << d << ", scale_mag: " << scale_mag
                  << ", scale.x: " << scale.x << ", scale.y: " << scale.y
                  << std::endl;
        std::cout << "===" << std::endl;
        if ((scale.x < 0) ||
            (scale.y < 0)) {  // assume reflections are bad matches
          return;
        }
        if (scale_mag < params.scale_threshold) {
          return;
        }
        if (d < params.identity_threshold) {
          return;
        }
        if (t_mag > params.translation_threshold) {
          return;
        }

        // Get the keypoints corresponding to these matches.
        const auto& dmatches = std::get<1>(matches);
        std::vector<cv::KeyPoint> keypoints;
        keypoints.reserve(dmatches.size());
        std::for_each(dmatches.begin(), dmatches.end(),
                      [&](const cv::DMatch& dmatch) {
                        keypoints.push_back(
                            feature_flow_ptr->curKeypoints()[dmatch.trainIdx]);
                      });

        // Draw the keypoints on the output image.
        const cv::Scalar rnd_clr(rand() % 255, rand() % 255, rand() % 255);
        cv::drawKeypoints(feature_flow_ptr->curFrame(), keypoints, out_image,
                          rnd_clr, cv::DrawMatchesFlags::DRAW_OVER_OUTIMG);
      });

  // Convert out_image to ROS image.
  auto viz_msg = cv_bridge::CvImage(header, "bgr8", out_image).toImageMsg();

  // Publish.
  viz_pub.publish(viz_msg);
}

void FeatureFlowNodeHandler::callback(const sensor_msgs::Image::ConstPtr& msg) {
  // ROS_INFO_STREAM("entered callback");
  // Error guard.
  if (!feature_flow_ptr) {
    return;
  }

  // Skip frames, if desired.
  if (skipped_frames < params.skip_frames) {
    ++skipped_frames;
    return;
  }
  skipped_frames = 0;

  // Convert to OpenCV.
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }

  // Add to feature flow instance, compute.
  feature_flow_ptr->addFrame(cv_ptr->image);
  ROS_INFO_STREAM(feature_flow_ptr->curKeypoints().size()
                  << " keypoints : "
                  << feature_flow_ptr->homographyMatches().size()
                  << " homography matches");

  // Publish visualization.
  visualize(msg->header);
}
}  // namespace open_maeve
