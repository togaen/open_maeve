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

#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Bool.h>

#include <cstdint>
#include <memory>
#include <string>

#include "./params.h"
#include "maeve_automation_core/struck/struck.h"
#include "struck_node/ImageBoundingBox.h"

namespace maeve_automation_core {

struct StruckTracker {
  bool doInitialise;
  StruckVisualTrackingParams params;
  Config conf;
  std::unique_ptr<Tracker> tracker;
  FloatRect initBB;

  explicit StruckTracker(ros::NodeHandle& nh);
  bool valid() const;
  void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
  void userInitCallback(const std_msgs::Bool::ConstPtr& msg);

 private:
  void publishTrackerImage(const ros::Time& time) const;
  void publishBoundingBox(const ros::Time& time) const;

  bool is_user_initted;
  bool initialized_successfully;

  cv_bridge::CvImage result;
  ros::Publisher tracker_image_pub;
  ros::Publisher tracker_bb_pub;
};  // struct StruckTracker

}  // namespace maeve_automation_core
