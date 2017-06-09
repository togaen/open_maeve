#pragma once

#include <string>

#include <sensor_msgs/Image.h>

#include "maeve_automation_core/struck_visual_tracking/struck_visual_tracking.h"

struct StruckTracker {
	bool runFromCameraTopic;
	Config conf;
	Tracker tracker;
	TrackerInit tracker_init;

	StruckTracker(const Config& c, const std::string& camera_topic) : runFromCameraTopic(!camera_topic.empty()), conf(c), tracker(c), tracker_init(buildTrackerInit(c, camera_topic)) {}
	bool runTracker();
	void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
};  // struct StruckTracker
