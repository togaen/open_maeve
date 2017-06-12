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

#include "maeve_automation_core/ros_parameter_loading/params_base.h"
#include "maeve_automation_core/struck/Config.h"

namespace maeve_automation_core {

struct StruckVisualTrackingParams : public ParamsBase {
  /// \brief Load parameters from parameter server.
  bool load(const ros::NodeHandle& nh) override;

  /// \brief Convert this parameter object to a STRUCK config object.
  Config toStruckConfig() const;

  /// \brief Check that a Struck config object has its members set to reasonable
  /// values.
  /// \return True if params seem okay; otherwise false.
  static bool SanityCheckStruckConfig(const Config& c);

  // params for bounding box
  BoundingBoxParams bb_params;

  // topic to listening for tracker initialized signale
  std::string init_tracker_topic;

  // init topic queue size
  int init_tracker_topic_queue_size;

  // topic name for camera images
  std::string camera_topic;

  // camera topic queue size
  int camera_topic_queue_size;

  // topic to publish tracker images to
  std::string tracker_image_topic;

  // topic to publish tracker bounding boxes to
  std::string tracker_bb_topic;

  // enable visualization topic
  bool enable_viz;

  // quiet mode disables all visual output (for experiments).
  bool quietMode;

  // debug mode enables additional drawing and visualization.
  bool debugMode;

  // frame size for use during tracking.
  // the input image will be scaled to this size.
  int frameWidth;
  int frameHeight;

  // seed for random number generator.
  int seed;

  // tracker search radius in pixels.
  int searchRadius;

  // SVM regularization parameter.
  double svmC;

  // SVM budget size (0 = no budget).
  int svmBudgetSize;

  // image features to use.
  // format is: feature kernel [kernel-params]
  // where:
  //   feature = haar/raw/histogram
  //   kernel = gaussian/linear/intersection/chi2
  //   for kernel=gaussian, kernel-params is sigma
  // multiple features can be specified and will be combined
  std::string feature;

 private:
  /// \brief Check common values of Struck and MA param types.
  /// \return True if seems okay; otherwise false.
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
