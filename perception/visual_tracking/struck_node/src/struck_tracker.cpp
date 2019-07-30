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

#include <ros/console.h>

namespace maeve_core {

StruckTracker::StruckTracker(ros::NodeHandle& nh)
    : doInitialise(false),
      is_user_initted(false),
      initialized_successfully(false),
      it(nh) {
  if (params.load(nh)) {
    conf = params.toStruckConfig();
    if (StruckVisualTrackingParams::SanityCheckStruckConfig(conf)) {
      srand(conf.seed);
      tracker = std::move(std::unique_ptr<Tracker>(new Tracker(conf)));
      result.image = cv::Mat(conf.frameHeight, conf.frameWidth, CV_8UC3);
      result.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
      initBB = IntRect(params.bb_params.bb_x_min, params.bb_params.bb_y_min,
                       params.bb_params.width, params.bb_params.height);

      // Advertise topics.
      tracker_image_pub = it.advertise(params.tracker_image_topic, 1);
      tracker_bb_pub = nh.advertise<struck_node::ImageBoundingBox>(
          params.tracker_bb_topic, 1000);

      // Register callbacks.
      user_init_sub = nh.subscribe(params.init_tracker_topic,
                                   params.init_tracker_topic_queue_size,
                                   &StruckTracker::userInitCallback, this);
      camera_sub =
          it.subscribe(params.camera_topic, params.camera_topic_queue_size,
                       &StruckTracker::cameraCallback, this);

      initialized_successfully = true;
    } else {
      ROS_ERROR_STREAM("STRUCK conf failed sanity check.");
    }
  } else {
    ROS_ERROR_STREAM("Failed to load parameters.");
  }
}

bool StruckTracker::valid() const { return initialized_successfully; }

void StruckTracker::publishBoundingBox(const ros::Time& time) const {
  const FloatRect& bb = tracker->GetBB();
  struck_node::ImageBoundingBox bb_msg;
  bb_msg.header.stamp = time;
  bb_msg.image_width = conf.frameWidth;
  bb_msg.image_height = conf.frameHeight;
  bb_msg.x_min = bb.XMin();
  bb_msg.x_max = bb.XMax();
  bb_msg.y_min = bb.YMin();
  bb_msg.y_max = bb.YMax();
  tracker_bb_pub.publish(bb_msg);
}

void StruckTracker::userInitCallback(const std_msgs::Bool::ConstPtr& msg) {
  if (!is_user_initted) {
    doInitialise = msg->data;
  }
}

void StruckTracker::publishTrackerImage(const ros::Time& time) const {
  if (params.enable_viz) {
    auto msg = result.toImageMsg();
    msg->header.stamp = time;
    tracker_image_pub.publish(msg);
  }
}

void StruckTracker::cameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
  // Convert ROS message to opencv image.
  cv_bridge::CvImageConstPtr frameOrig;
  try {
    frameOrig =
        cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC3);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
    return;
  }

  // Prepare frame for tracking.
  cv::Mat frame;
  cv::resize(frameOrig->image, frame,
             cv::Size(conf.frameWidth, conf.frameHeight));
  flip(frame, frame, 1);
  frame.copyTo(result.image);
  if (doInitialise) {
    if (tracker->IsInitialised()) {
      tracker->Reset();
    } else {
      tracker->Initialise(frame, initBB);
    }
    doInitialise = false;
  } else if (!tracker->IsInitialised()) {
    rectangle(result.image, initBB, CV_RGB(255, 255, 255));
  }

  // Perform tracking.
  if (tracker->IsInitialised()) {
    tracker->Track(frame);

    if (!conf.quietMode && conf.debugMode) {
      tracker->Debug();
    }

    rectangle(result.image, tracker->GetBB(), CV_RGB(0, 255, 0));
  }

  // Set result header and publish image (optionally).
  publishTrackerImage(msg->header.stamp);

  // Publish track.
  publishBoundingBox(msg->header.stamp);
}

}  // namespace maeve_core
