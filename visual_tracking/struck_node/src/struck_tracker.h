// Copyright 2017 Maeve Automation
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
