#include "struck_tracker.h"

#include <ros/console.h>

StruckTracker::StruckTracker(ros::NodeHandle& nh) : doInitialise(false), is_user_initted(false), initialized_successfully(false) {
  if (params.load(nh)) {
		ROS_INFO_STREAM("Loaded params:\n" << params);
		conf = params.toStruckConfig();
		if (StruckVisualTrackingParams::SanityCheckStruckConfig(conf)) {
	    srand(conf.seed);
			tracker = std::move(std::unique_ptr<Tracker>(new Tracker(conf)));
		  tracker_image_pub = nh.advertise<sensor_msgs::Image>(params.tracker_image_topic, 1);
		  tracker_bb_pub = nh.advertise<struck_node::ImageBoundingBox>(params.tracker_bb_topic, 1000);
  	  result.image = cv::Mat(conf.frameHeight, conf.frameWidth, CV_8UC3);
	    result.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
      initBB = IntRect(params.bb_params.bb_x_min, params.bb_params.bb_y_min, params.bb_params.width, params.bb_params.height);
      initialized_successfully = true;
		} else {
			ROS_ERROR_STREAM("STRUCK conf failed sanity check.");
		}
	} else {
    ROS_ERROR_STREAM("Failed to load parameters.");
	}
}

bool StruckTracker::valid() const {
	return initialized_successfully;
}

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
    try
		{
		  frameOrig = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC3);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
		  return;
		}
	
		// Prepare frame for tracking.
		cv::Mat frame;
		cv::resize(frameOrig->image, frame, cv::Size(conf.frameWidth, conf.frameHeight));
		flip(frame, frame, 1);
		frame.copyTo(result.image);
		if (doInitialise)
		{
			if (tracker->IsInitialised())
			{
				tracker->Reset();
			}
			else
			{
				tracker->Initialise(frame, initBB);
			}
			doInitialise = false;
		}
		else if (!tracker->IsInitialised())
		{
			rectangle(result.image, initBB, CV_RGB(255, 255, 255));
		}

    // Perform tracking.
		if (tracker->IsInitialised())
		{
			tracker->Track(frame);
			
			if (!conf.quietMode && conf.debugMode)
			{
				tracker->Debug();
			}
			
			rectangle(result.image, tracker->GetBB(), CV_RGB(0, 255, 0));
		}

		// Set result header and publish image (optionally).
		publishTrackerImage(msg->header.stamp);

		// Publish track.
		publishBoundingBox(msg->header.stamp);
}

