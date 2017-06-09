#include "struck_tracker.h"

#include <ros/console.h>

void StruckTracker::cameraCallback(const sensor_msgs::Image::ConstPtr& msg) {

	srand(conf.seed);
	const auto paused = false;
	cv::Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);
	for (int frameInd = tracker_init.startFrame; frameInd <= tracker_init.endFrame; ++frameInd)
	{

		/// BEGIN 
		cv::Mat frame;
	  cv::Mat frameOrig; // <-- set this to incoming image message
    cv::Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);
    prepareCameraTrackingFrame(frameOrig, conf, tracker_init, tracker, frame, result);
		/// END
		
		if (tracker.IsInitialised())
		{
			tracker.Track(frame);
			
			if (!conf.quietMode && conf.debugMode)
			{
				tracker.Debug();
			}
			
			rectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));
		}

		// publish to output viz topic
		if (!showOutput(conf, result, tracker_init, frameInd, paused)) {
			break;
		}
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
