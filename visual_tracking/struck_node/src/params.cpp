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

#include "./params.h"

namespace maeve_automation_core {

bool StruckVisualTrackingParams::load(const ros::NodeHandle& nh) {
  LOAD_PARAM(camera_topic);
  LOAD_PARAM(camera_topic_queue_size);
  LOAD_PARAM(init_tracker_topic);
  LOAD_PARAM(init_tracker_topic_queue_size);
  LOAD_PARAM(tracker_image_topic);
  LOAD_PARAM(tracker_bb_topic);
  LOAD_PARAM(enable_viz);
  LOAD_PARAM(frameWidth);
  LOAD_PARAM(frameHeight);
  LOAD_PARAM(seed);
  LOAD_PARAM(searchRadius);
  LOAD_PARAM(svmC);
  LOAD_PARAM(svmBudgetSize);
  LOAD_PARAM(feature);
  CHECK_NONEMPTY(feature);
  CHECK_NONEMPTY(tracker_bb_topic);

  const auto bb_loaded = bb_params.load(nh);
  if (bb_loaded) {
    std::stringstream ss;
    ss << bb_params;
    loaded_param_set += ss.str();
  }

  const auto sanity_passed = SanityCheckConfig(*this);

  return bb_loaded && sanity_passed;
}

bool StruckVisualTrackingParams::SanityCheckStruckConfig(const Config& c) {
  CHECK_NONEMPTY(c.features);
  return SanityCheckConfig(c);
}

Config StruckVisualTrackingParams::toStruckConfig() const {
  Config config;

  config.quietMode = false;
  config.debugMode = false;

  config.sequenceBasePath = "";
  config.sequenceName = "";
  config.resultsPath = "";

  config.frameWidth = frameWidth;
  config.frameHeight = frameHeight;

  config.seed = seed;
  config.searchRadius = searchRadius;
  config.svmC = svmC;
  config.svmBudgetSize = svmBudgetSize;

  config.features.clear();
  std::istringstream iss(feature);
  config.ParseFeatureString(iss);

  return config;
}

}  // namespace maeve_automation_core
