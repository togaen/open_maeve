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

#include "./struck_tracker.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include <string>

namespace {
namespace ma_core = maeve_automation_core;
}  // namespace

int main(int argc, char* argv[]) {
  const auto node_name = std::string("struck_node");

  // Initialize ROS node.
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh(node_name);

  // Initialize STRUCK tracker.
  auto struck_tracker = ma_core::StruckTracker(nh);
  if (!struck_tracker.valid()) {
    ROS_INFO_STREAM("Struck config object failed sanity check.");
    return EXIT_FAILURE;
  }

  // For convenience.
  const auto& params = struck_tracker.params;

  // This topic enables user to initialize tracking.
  auto init_sub = nh.subscribe(
      params.init_tracker_topic, params.init_tracker_topic_queue_size,
      &ma_core::StruckTracker::userInitCallback, &struck_tracker);

  // Set camera image stream topic.
  auto camera_sub =
      nh.subscribe(params.camera_topic, params.camera_topic_queue_size,
                   &ma_core::StruckTracker::cameraCallback, &struck_tracker);

  // Kick it off.
  ros::spin();

  // Done.
  return EXIT_SUCCESS;
}
