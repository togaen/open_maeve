#include "struck_tracker.h"

#include <ros/console.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

void StruckTracker::cameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
	srand(conf.seed);
	const auto paused = false;	

		// Convert ROS message to opencv image.
		cv_bridge::CvImageConstPtr cv_ptr;
    try
		{
		  cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::TYPE_8UC3);
		}
		catch (cv_bridge::Exception& e)
		{
			ROS_ERROR_STREAM("cv_bridge exception: " << e.what());
		  return;
		}

		// Perform tracking.
		cv_bridge::CvImage frame;
		frame.header = msg->header;
		frame.encoding = sensor_msgs::image_encodings::TYPE_8UC3;
    prepareCameraTrackingFrame(cv_ptr->image, conf, tracker_init, tracker, frame.image, result);
		if (tracker.IsInitialised())
		{
			tracker.Track(frame.image);
			
			if (!conf.quietMode && conf.debugMode)
			{
				tracker.Debug();
			}
			
			rectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));
		}

	
	  if (params.enable_viz) {
			ROS_INFO_STREAM("Publishing tracker image...");
			tracker_image_pub.publish(frame.toImageMsg());
		}	
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
	
	cv::Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);
	auto paused = false;
	srand(conf.seed);
	for (int frameInd = tracker_init.startFrame; frameInd <= tracker_init.endFrame; ++frameInd)
	{
		cv::Mat frame;
		if (!prepareTrackingFrame(conf, tracker_init, tracker, frame, result, frameInd)) {
			return false;
		}
		
		if (tracker.IsInitialised())
		{
			tracker.Track(frame);
			
			if (!conf.quietMode && conf.debugMode)
			{
				tracker.Debug();
			}
			
			rectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));

      writeOutput(tracker, tracker_init, outFile);
		}

		if (!showOutput(conf, result, tracker_init, frameInd, paused)) {
			break;
		}
	}
	
	if (outFile.is_open())
	{
		outFile.close();
	}

	return true;
}
