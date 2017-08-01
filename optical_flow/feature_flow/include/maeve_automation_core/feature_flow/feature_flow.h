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

#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

#include <vector>

namespace maeve_automation_core {

/**
  * @brief Perform temporal segmentation and tracking based on feature
  * correlation.
  */
class FeatureFlow {
 public:
  /**
   * @brief Constructor: set parameters at construction time.
   *
   * @param lsh_table_number Number of tables to use for descriptor matching
   * @param lsh_key_size Key bits to use for descriptor matching
   * @param lsh_multi_probe_level This is typically 2.
   * @param threshold_level BRISK threshold level.
   * @param octaves BRISK octaves.
   * @param pattern_scales BRISK pattern scales.
   * @param _good_match_portion Heuristic threshold for a "good" match.
   */
  FeatureFlow(int lsh_table_number, int lsh_key_size, int lsh_multi_probe_level,
              int threshold_level, int octaves, float pattern_scales,
              float _good_match_portion);

  /**
   * @brief Callback for image frame processing.
   *
   * @param frame The next frame to add for processing.
   * @return True if successful; otherwise false.
   */
  bool addFrame(const cv::Mat& frame);

 private:
  /**
   * @brief Heuristically determine "good" descriptor matches.
   *
   * @param matches The set of descriptor matches.
   *
   * @return The matches that satisfy the hueristic.
   */
  std::vector<cv::DMatch> computeGoodMatches(
      const std::vector<cv::DMatch>& matches);

  /**
   * @brief Convenience method for running the BRISK detector.
   *
   * @param image The image to compute features for.
   * @param keypoints Storage for keypoints.
   * @param descriptors Storage for descriptors.
   */
  void runDetector(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                   cv::Mat& descriptors);

  /** @brief The BRISK feature detector. */
  cv::Ptr<cv::BRISK> brisk_detector;

  /** @brief Matcher used to correlated keypoint descriptors. */
  cv::FlannBasedMatcher matcher;

  /** @brief Previous image stream frame. */
  cv::Mat prv_frame;

  /** @brief Current image stream frame. */
  cv::Mat cur_frame;

  /** @brief Descriptor matches within this portion of range are "good". */
  float good_match_portion;

  /** @name BRISK feature detection parameters
   * @{
   */
  /** @brief Threshold level. */
  int threshold_level;
  /** @brief Octaves. */
  int octaves;
  /** @brief Pattern scales. */
  float pattern_scales;
  /** @} */

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
