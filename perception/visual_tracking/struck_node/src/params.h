/*
 * Copyright 2017 Maeve Automation
 *
 * Struck: Structured Output Tracking with Kernels
 *
 * Derived work of code to accompany the paper:
 *   Struck: Structured Output Tracking with Kernels
 *   Sam Hare, Amir Saffari, Philip H. S. Torr
 *   International Conference on Computer Vision (ICCV), 2011
 *
 * Copyright (C) 2011 Sam Hare, Oxford Brookes University, Oxford, UK
 *
 * This file is a derivative work of Struck.
 *
 * Struck is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Struck is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Struck.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#pragma once

#include <string>

#include "./bb_params.h"

#include "maeve_automation_core/ros_parameter_loading/ros_parameter_loading.h"
#include "maeve_automation_core/struck/Config.h"

namespace maeve_automation_core {

/** @brief Parameter object to load and convert STRUCK parameters from ROS.*/
struct StruckVisualTrackingParams : public ParamsBase {
  /** @brief The geometry of the initial bounding box.*/
  BoundingBoxParams bb_params;

  /** @brief Topic to listen for user-initiated tracker ready signal.*/
  std::string init_tracker_topic;

  /** @brief Queue size for user-initiated tracker ready signal topic.*/
  int init_tracker_topic_queue_size;

  /** @brief Topic to listen to for camera image stream.*/
  std::string camera_topic;

  /** @brief Queue size for camera image stream topic.*/
  int camera_topic_queue_size;

  /** @brief Topic to publish tracker visualization to.*/
  std::string tracker_image_topic;

  /** @brief Topic to publish tracker bounding boxes to.*/
  std::string tracker_bb_topic;

  /** @brief Whether to publish to the tracker visualization topic.*/
  bool enable_viz;

  /** @brief Seed for random number generator.*/
  int seed;

  /** Tracker search radius in pixels.*/
  int searchRadius;

  /** @name Frame Size
   * Frame size for use during tracking; input is scaled to this size.
   * @{
   */
  /** @brief The width of the input image for tracking.*/
  int frameWidth;
  /** @brief The height of the input image for tracking.*/
  int frameHeight;
  /** @} */

  /** @name SVM Settings
   * @{
   */
  /** @brief SVM regularization parameter.*/
  double svmC;
  /** @brief SVM budget size (0 = no budget).*/
  int svmBudgetSize;
  /**@}*/

  /**
   * @brief Image features to use.
   *
   * Format is: feature kernel [kernel-params]
   * where:
   * feature = haar/raw/histogram
   * kernel = gaussian/linear/intersection/chi2
   * for kernel=gaussian, kernel-params is sigma
   * multiple features can be specified and will be combined
   */
  std::string feature;

  /**
   * @copydoc ParamsBase::ParamsBase()
   */
  __attribute__((warn_unused_result)) bool load(
      const ros::NodeHandle& nh) override;

  /**
   * @brief Generate a STRUCK config object from the parameters in this object.
   *
   * @return The STRUCK config.
   */
  Config toStruckConfig() const;

  /**
   * @brief Perform some basic sanity checks on a STRUCK config object.
   *
   * @param c The STRUCK config object.
   *
   * @return True if sanity checks pass; otherwise false.
   */
  static bool SanityCheckStruckConfig(const Config& c);

 private:
  /**
   * @brief Check common values of Struck and MA param types.
   *
   * @param c The config object (STRUCK or ROS) to check.
   *
   * @return True if seems okay; otherwise false.
   */
  template <typename T>
  static bool SanityCheckConfig(const T& c) {
    CHECK_STRICTLY_POSITIVE(c.frameWidth);
    CHECK_STRICTLY_POSITIVE(c.frameHeight);
    CHECK_STRICTLY_POSITIVE(c.searchRadius);
    CHECK_STRICTLY_POSITIVE(c.svmC);
    CHECK_STRICTLY_POSITIVE(c.svmBudgetSize);
    return true;
  }
};  // struct StruckVisualTrackingParams

}  // namespace maeve_automation_core
