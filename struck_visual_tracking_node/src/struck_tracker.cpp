#include "struck_tracker.h"

#include <ros/console.h>

void StruckTracker::publishBoundingBox(const ros::Time& time) {
		const FloatRect& bb = tracker.GetBB();
    struck_visual_tracking_interface::ImageBoundingBox bb_msg;
		bb_msg.header.stamp = time;
		bb_msg.image_width = conf.frameWidth;
	  bb_msg.image_height = conf.frameHeight;	
		bb_msg.x_min = bb.XMin() / tracker_init.scaleW;
		bb_msg.x_max = bb.XMax() / tracker_init.scaleW;
		bb_msg.y_min = bb.YMax() / tracker_init.scaleH;
		bb_msg.y_max = bb.YMax() / tracker_init.scaleH;
    tracker_bb_pub.publish(bb_msg);
}

void StruckTracker::userInitCallback(const std_msgs::Bool::ConstPtr& msg) {
	if (!is_user_initted) {
				if (msg->data && tracker_init.useCamera)
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

		// Perform tracking.
		cv::Mat frame;
    prepareCameraTrackingFrame(frameOrig->image, conf, tracker_init, tracker, frame, result.image);
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

bool StruckTracker::runTracker() {
	if (!conf.quietMode)
	{
		cv::namedWindow("result");
	}
	
	auto paused = false;
	srand(conf.seed);
	for (int frameInd = tracker_init.startFrame; frameInd <= tracker_init.endFrame; ++frameInd)
	{
		cv::Mat frame;
		if (!prepareTrackingFrame(conf, tracker_init, tracker, frame, result.image, frameInd)) {
			return false;
		}
		
		if (tracker.IsInitialised())
		{
			tracker.Track(frame);
			
			if (!conf.quietMode && conf.debugMode)
			{
				tracker.Debug();
			}
			
			rectangle(result.image, tracker.GetBB(), CV_RGB(0, 255, 0));
		}

		publishTrackerImage(ros::Time::now());
	}
	return true;
}
