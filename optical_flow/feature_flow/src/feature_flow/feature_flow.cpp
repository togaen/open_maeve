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
#include "maeve_automation_core/feature_flow/feature_flow.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace maeve_automation_core {

FeatureFlow::FeatureFlow(int lsh_table_number, int lsh_key_size,
                         int lsh_multi_probe_level, int threshold_level,
                         int octaves, float pattern_scales,
                         float _good_match_portion)
    : matcher(new cv::flann::LshIndexParams(20, 10, 2)),
      good_match_portion(_good_match_portion) {
  brisk_detector = cv::BRISK::create(threshold_level, octaves, pattern_scales);
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
    threshold = min_distance + good_match_portion * range;
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

bool FeatureFlow::addFrame(const cv::Mat& frame) {
  if (prv_frame.empty()) {
    prv_frame = frame;
    return true;
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
  const auto good_matches = computeGoodMatches(all_matches);

  // Compute homography describing largest set of matching keypoints.
  // Remove (or ignore) those previous/current keypoints.
  // Repeat until some threshold is met.

  return false;
}

}  // namespace maeve_automation_core
