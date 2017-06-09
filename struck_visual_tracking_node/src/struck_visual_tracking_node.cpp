#include "params.h"

#include <iostream>
#include <fstream>

#include "ros/ros.h"
#include "ros/console.h"

#include "maeve_automation_core/struck_visual_tracking/struck_visual_tracking.h"

int main(int argc, char* argv[]) {
	const auto node_name = std::string("struck_visual_tracking_node");

	// Initialize ROS node.
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh(node_name);

	// Load parameters.
  auto params = StruckVisualTrackingParams();
	if (!params.load(nh)) {
		ROS_ERROR_STREAM("Failed to load parameters. Aborting.");
		return EXIT_FAILURE;
	}

	// Debug params?
	if (params.debugMode) {	
		ROS_INFO_STREAM("Loaded params:\n" << params);
	}

	// Initialize STRUCK tracker.
	auto conf = params.toStruckConfig();
	if (!StruckVisualTrackingParams::SanityCheckStruckConfig(conf)) {
    ROS_INFO_STREAM("Struck config object failed sanity check.");
		return EXIT_FAILURE;
	}
	Tracker tracker(conf);

	std::ofstream outFile;
	if (conf.resultsPath != "")
	{
		outFile.open(conf.resultsPath.c_str(), std::ios::out);
		if (!outFile)
		{
			ROS_ERROR_STREAM("error: could not open results file: " << conf.resultsPath);
			return EXIT_FAILURE;
		}
	}

  auto tracker_init = buildTrackerInit(conf);
	if (!tracker_init.valid) {
		return EXIT_FAILURE;
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
		if (!initializeTracker(conf, tracker_init, tracker, frame, result, frameInd)) {
			return EXIT_FAILURE;
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
	
	return EXIT_SUCCESS;
}
