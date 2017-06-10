#include "struck_tracker.h"

#include <ros/console.h>

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
}

bool StruckTracker::runTracker() {
  std::ofstream outFile;
	if (conf.resultsPath != "")
	{
		outFile.open(conf.resultsPath.c_str(), std::ios::out);
		if (!outFile)
		{
			ROS_ERROR_STREAM("error: could not open results file: " << conf.resultsPath);
			return false;
		}
	}

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

      writeOutput(tracker, tracker_init, outFile);
		}

		publishTrackerImage(ros::Time::now());
	}
	
	if (outFile.is_open())
	{
		outFile.close();
	}

	return true;
}
