#pragma once

#include "params.h"

#include <cstdint>
#include <string>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>

#include "struck_visual_tracking_node/ImageBoundingBox.h"
#include "maeve_automation_core/struck_visual_tracking/struck_visual_tracking.h"

struct StruckTracker {
  StruckVisualTrackingParams params;
	bool runFromCameraTopic;
	Config conf;
	Tracker tracker;
	TrackerInit tracker_init;


	StruckTracker(const StruckVisualTrackingParams& p, ros::NodeHandle& nh) : params(p), runFromCameraTopic(!params.camera_topic.empty()), conf(params.toStruckConfig()), tracker(conf), tracker_init(buildTrackerInit(conf, params.camera_topic)), is_user_initted(false), tracker_image_pub(nh.advertise<sensor_msgs::Image>(params.tracker_image_topic, 1000)), tracker_bb_pub(nh.advertise<struck_visual_tracking_node::ImageBoundingBox>(params.tracker_bb_topic, 1000)) {
	result.image = cv::Mat(conf.frameHeight, conf.frameWidth, CV_8UC3);
result.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
	}
	void cameraCallback(const sensor_msgs::Image::ConstPtr& msg);
	void userInitCallback(const std_msgs::Bool::ConstPtr& msg);

	private:
  void publishTrackerImage(const ros::Time& time);
  void publishBoundingBox(const ros::Time& time);

	bool is_user_initted;

	cv_bridge::CvImage result;
	ros::Publisher tracker_image_pub;
  ros::Publisher tracker_bb_pub;
};  // struct StruckTracker
