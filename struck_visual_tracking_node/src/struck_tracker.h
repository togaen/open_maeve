#pragma once

#include "params.h"

#include <cstdint>
#include <string>
#include <memory>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "struck_visual_tracking_node/ImageBoundingBox.h"
#include "maeve_automation_core/struck_visual_tracking/struck_visual_tracking.h"

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
  void publishTrackerImage(const ros::Time& time);
  void publishBoundingBox(const ros::Time& time);

	bool is_user_initted;
	bool initialized_successfully;

	cv_bridge::CvImage result;
	ros::Publisher tracker_image_pub;
  ros::Publisher tracker_bb_pub;
};  // struct StruckTracker
