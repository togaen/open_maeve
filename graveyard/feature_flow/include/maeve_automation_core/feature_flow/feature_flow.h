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

#include <limits>
#include <tuple>
#include <vector>

namespace maeve_automation_core {

/**
  * @brief Perform temporal segmentation and tracking based on feature
  * correlation.
  */
class FeatureFlow {
 public:
  /** @brief Homography with corresponding matches. */
  typedef std::tuple<cv::Mat, std::vector<cv::DMatch>> HomographyMatches;

  /**
   * @brief Parameter container for Feature Flow objects.
   */
  struct Params {
    /** @brief Threshold for what constitutes a "good" match. */
    float good_match_portion;

    /** @brief Error threshold for determining inliner/outlier status. */
    int ransac_reprojection_error_threshold;

    /** @brief Minimum number of keypoints to perform detection on. */
    int min_keypoints;

    /** @brief Limit the unique homographies to compute (-1 = no limit). */
    int max_homographies;

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

    /** @name Locally Sensitive Hashing parameters
     * @{
     */
    /** @brief Number of hash tables to use. */
    int lsh_table_number;
    /** @brief Key bits to use. */
    int lsh_key_size;
    /** @brief Typically set to 2. */
    int lsh_multi_probe_level;
    /** @} */
  };  // struct Params

  /**
   * @brief Used to encapsulate the scale components of a homography.
   */
  struct ScaleComponents {
    double x;
    double y;
    ScaleComponents();
  };  // struct ScaleComponents

  /**
   * @brief Get the scale components of a homography matrix.
   *
   * @param H The 2D homography matrix.
   *
   * @return The scale components. Components are NaN if operation fails.
   */
  static ScaleComponents getScaleComponents(const cv::Mat& H);

  /**
   * @brief Constructor: set parameters at construction time.
   *
   * @param _params A filled params struct.
   */
  explicit FeatureFlow(const Params& _params);

  /**
   * @brief Callback for image frame processing.
   *
   * @param frame The next frame to add for processing.
   */
  void addFrame(const cv::Mat& frame);

  /** @name Member accessors
   * @{
   */
  const cv::Mat& prvFrame() const { return prv_frame; }
  const cv::Mat& prvDescriptors() const { return descriptors_prv; }
  const std::vector<cv::KeyPoint>& prvKeypoints() const {
    return keypoints_prv;
  }
  const cv::Mat& curFrame() const { return cur_frame; }
  const cv::Mat& curDescriptors() const { return descriptors_cur; }
  const std::vector<cv::KeyPoint>& curKeypoints() const {
    return keypoints_cur;
  }
  const std::vector<HomographyMatches>& homographyMatches() const {
    return homographies;
  }
  /** @} */

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

  /** @brief Algorithm parameters. */
  Params params;

  /** @brief The BRISK feature detector. */
  cv::Ptr<cv::BRISK> brisk_detector;

  /** @brief Matcher used to correlated keypoint descriptors. */
  cv::FlannBasedMatcher matcher;

  /** @brief Previous image stream frame. */
  cv::Mat prv_frame;

  /** @brief Current image stream frame. */
  cv::Mat cur_frame;

  /** @name Segmentation and Track Information
   * These array are indexed aligned and contain object information.
   * @{
   */
  /** @brief List of homographies between previous and current frames. */
  std::vector<HomographyMatches> homographies;
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
