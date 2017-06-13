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

/**
 * @brief Convenience encapsulaton of functions/variable for running tracker.
 */
class StruckTracker {
	public:
	/** @brief Flag for whether to initialize the tracker.*/
  bool doInitialise;

	/** @brief ROS parameter object.*/
  StruckVisualTrackingParams params;

	/** @brief STRUCK parameter object.*/
  Config conf;

	/** @brief The STRUCK tracker object.*/
  std::unique_ptr<Tracker> tracker;

	/** @brief The gemoetry of the initial bounding box.*/
	FloatRect initBB;

	/**
	 * @brief Construct an instance of this class using a ROS node handle.
	 *
	 * @param nh The handle of the node owning this object.
	 */
  explicit StruckTracker(ros::NodeHandle& nh);

	/**
	 * @brief Whether this object is probably validly initialized.
	 *
	 * @note The tests in this function do not guarantee parameterization is
	 * correct; they only check for obvious problems.
	 *
	 * @return True if the sanity checks pass; otherwise false.
	 */
  bool valid() const;

	/**
	 * @brief Callback to run the tracker on each input image frame.
	 *
	 * @param msg The image frame.
	 */
  void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);

	/**
	 * @brief Callback to initialize the tracker based on the user trigger.
	 *
	 * @param msg The boolean flag indicating the user trigger.
	 */
  void userInitCallback(const std_msgs::Bool::ConstPtr& msg);

 private:
	/** @brief Whether the user has triggered initialization yet.*/
  bool is_user_initted;

	/** @brief Whether the tracker has been successfully initialized.*/
  bool initialized_successfully;

	/** @brief Storage for the tracker visualization.*/
  cv_bridge::CvImage result;

	/** @brief ROS publisher for tracker visualization.*/
  ros::Publisher tracker_image_pub;

	/** @brief ROS publisher for tracker bounding box output.*/
  ros::Publisher tracker_bb_pub;

	/**
	 * @brief Publish a visualization of tracker output.
	 *
	 * @param time The desired stamp that the visualization image should have.
	 */
  void publishTrackerImage(const ros::Time& time) const;

	/**
	 * @brief Publish the bounding box output of the tracker.
	 *
	 * @param time The desired stamp that the bounding box output should have.
	 */
  void publishBoundingBox(const ros::Time& time) const;
};  // class StruckTracker

}  // namespace maeve_automation_core
