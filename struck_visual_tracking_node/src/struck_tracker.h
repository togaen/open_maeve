#pragma once

#include "params.h"

#include <string>

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

#include "maeve_automation_core/struck_visual_tracking/struck_visual_tracking.h"

struct StruckTracker {
  StruckVisualTrackingParams params;
	bool runFromCameraTopic;
	Config conf;
	Tracker tracker;
	TrackerInit tracker_init;
  cv::Mat result;
	ros::Publisher tracker_image_pub;

	StruckTracker(const StruckVisualTrackingParams& p, ros::NodeHandle& nh) : params(p), runFromCameraTopic(!params.camera_topic.empty()), conf(params.toStruckConfig()), tracker(conf), tracker_init(buildTrackerInit(conf, params.camera_topic)), result(conf.frameHeight, conf.frameWidth, CV_8UC3), tracker_image_pub(nh.advertise<sensor_msgs::Image>(params.tracker_image_topic, 1000)) {}
	bool runTracker();
	void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
};  // struct StruckTracker
