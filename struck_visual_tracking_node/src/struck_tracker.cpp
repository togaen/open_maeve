#include "struck_tracker.h"

#include <ros/console.h>

TrackerInit::TrackerInit(const StruckVisualTrackingParams& params) : doInitialise(false) {
	initBB = IntRect(params.bb_params.bb_x_min, params.bb_params.bb_y_min, params.bb_params.width, params.bb_params.height);
}

void StruckTracker::publishBoundingBox(const ros::Time& time) {
		const FloatRect& bb = tracker.GetBB();
    struck_visual_tracking_node::ImageBoundingBox bb_msg;
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
				if (msg->data)
				{
					tracker_init.doInitialise = true;
				}
	}
}

void StruckTracker::publishTrackerImage(const ros::Time& time) {
    result.header.stamp = time;
	  if (params.enable_viz) {
			tracker_image_pub.publish(result.toImageMsg());
		}
}

void StruckTracker::cameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
	srand(conf.seed);
	const auto paused = false;	

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
		if (tracker_init.doInitialise)
		{
			if (tracker.IsInitialised())
			{
				tracker.Reset();
			}
			else
			{
				tracker.Initialise(frame, tracker_init.initBB);
			}
			tracker_init.doInitialise = false;
		}
		else if (!tracker.IsInitialised())
		{
			rectangle(result.image, tracker_init.initBB, CV_RGB(255, 255, 255));
		}

    // Perform tracking.
		if (tracker.IsInitialised())
		{
			tracker.Track(frame);
			
			if (!conf.quietMode && conf.debugMode)
			{
				tracker.Debug();
			}
			
			rectangle(result.image, tracker.GetBB(), CV_RGB(0, 255, 0));
		}

		// Set result header and publish image (optionally).
		publishTrackerImage(msg->header.stamp);

		// Publish track.
		publishBoundingBox(msg->header.stamp);
}

