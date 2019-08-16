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
#include "open_maeve/feature_flow/feature_flow.h"

#include <opencv2/calib3d/calib3d.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace open_maeve {

FeatureFlow::ScaleComponents::ScaleComponents()
    : x(std::numeric_limits<double>::quiet_NaN()),
      y(std::numeric_limits<double>::quiet_NaN()) {}

// https://math.stackexchange.com/questions/78137/decomposition-of-a-nonsquare-affine-matrix
FeatureFlow::ScaleComponents FeatureFlow::getScaleComponents(const cv::Mat& H) {
  ScaleComponents scale;
  const auto a = H.at<double>(0, 0);
  const auto b = H.at<double>(0, 1);
  const auto d = H.at<double>(1, 0);
  const auto e = H.at<double>(1, 1);

  const auto theta = std::atan2(d, a);

  scale.x = std::sqrt(a * a + d * d);
  scale.y = (a * e - b * d) / scale.x;
  // scale.y = e * std::cos(theta) - b * std::sin(theta);

  return scale;
}

FeatureFlow::FeatureFlow(const Params& _params)
    : params(_params),
      matcher(new cv::flann::LshIndexParams(params.lsh_table_number,
                                            params.lsh_key_size,
                                            params.lsh_multi_probe_level)) {
  brisk_detector = cv::BRISK::create(params.threshold_level, params.octaves,
                                     params.pattern_scales);
}

void FeatureFlow::runDetector(const cv::Mat& image,
                              std::vector<cv::KeyPoint>& keypoints,
                              cv::Mat& descriptors) {
  // Clear old data.
  keypoints.clear();
  descriptors.release();

  // Compute new data.
  brisk_detector->detect(image, keypoints);
  brisk_detector->compute(image, keypoints, descriptors);
}

std::vector<cv::DMatch> FeatureFlow::computeGoodMatches(
    const std::vector<cv::DMatch>& matches) {
  // Initialize return value.
  std::vector<cv::DMatch> good_matches;

  // Find extrema.
  auto result = std::minmax_element(matches.begin(), matches.end());

  // Compute threshold.
  auto threshold = std::numeric_limits<float>::quiet_NaN();
  if ((result.first != matches.end()) && (result.second != matches.end())) {
    const auto min_distance = result.first->distance;
    const auto max_distance = result.second->distance;
    const auto range = max_distance - min_distance;
    threshold = min_distance + params.good_match_portion * range;
  }

  // Fill "good" matches.
  good_matches.reserve(matches.size());
  std::for_each(matches.begin(), matches.end(), [&](const cv::DMatch& match) {
    if (match.distance < threshold) {
      good_matches.push_back(match);
    }
  });

  // Done.
  return good_matches;
}

void FeatureFlow::addFrame(const cv::Mat& frame) {
  if (prv_frame.empty()) {
    prv_frame = frame;
  }

  if (cur_frame.empty()) {
    cur_frame = frame;
  } else {
    prv_frame = cur_frame;
    cur_frame = frame;
  }

  // Compute features.
  runDetector(prv_frame, keypoints_prv, descriptors_prv);
  runDetector(cur_frame, keypoints_cur, descriptors_cur);

  // Perform matching.
  std::vector<cv::DMatch> all_matches;
  matcher.match(descriptors_prv, descriptors_cur, all_matches);

  // Heuristic to determine a "good" matches.
  auto good_matches = computeGoodMatches(all_matches);

  // Find homographies.
  homographies.clear();
  const auto unlimited_homographies = params.max_homographies < 0;
  auto homographies_left = unlimited_homographies ||
                           (homographies.size() <= params.max_homographies);
  auto keypoints_left = good_matches.size() >= params.min_keypoints;
  while (keypoints_left && homographies_left) {
    // Extract keypoints.
    std::vector<cv::Point2f> prv_points;
    prv_points.reserve(good_matches.size());
    std::vector<cv::Point2f> cur_points;
    cur_points.reserve(good_matches.size());
    std::for_each(good_matches.begin(), good_matches.end(),
                  [&](const cv::DMatch& match) {
                    prv_points.push_back(keypoints_prv[match.queryIdx].pt);
                    cur_points.push_back(keypoints_cur[match.trainIdx].pt);
                  });

    // Compute homography describing largest set of matching keypoints.
    cv::Mat mask;
    const auto H =
        cv::findHomography(prv_points, cur_points, CV_RANSAC,
                           params.ransac_reprojection_error_threshold, mask);

    // Separate inliers and outliers.
    std::vector<cv::DMatch> inliers;
    inliers.reserve(good_matches.size());
    std::vector<cv::DMatch> outliers;
    outliers.reserve(good_matches.size());
    for (auto i = 0; i < mask.rows; ++i) {
      if (mask.at<uchar>(i) == 0) {
        outliers.push_back(good_matches[i]);
      } else {
        inliers.push_back(good_matches[i]);
      }
    }

    // Collect homography with its inliers.
    homographies.push_back(std::make_tuple(H, std::move(inliers)));

    // Reset "good" matches.
    good_matches.swap(outliers);

    // Still going?
    keypoints_left = good_matches.size() >= params.min_keypoints;
    homographies_left = unlimited_homographies ||
                        (homographies.size() <= params.max_homographies);
  }
}

}  // namespace open_maeve
