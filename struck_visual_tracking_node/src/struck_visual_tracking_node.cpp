#include "vot.hpp"
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

  auto tracker_init = BuildTrackerInit(conf);
	if (!tracker_init.valid) {
		return EXIT_FAILURE;
	}
	
	if (!conf.quietMode)
	{
		cv::namedWindow("result");
	}
	
	cv::Mat result(conf.frameHeight, conf.frameWidth, CV_8UC3);
	auto paused = false;
	auto doInitialise = false;
	srand(conf.seed);
	for (int frameInd = tracker_init.startFrame; frameInd <= tracker_init.endFrame; ++frameInd)
	{
		cv::Mat frame;
		if (tracker_init.useCamera)
		{
			cv::Mat frameOrig;
			tracker_init.cap >> frameOrig;
			resize(frameOrig, frame, cv::Size(conf.frameWidth, conf.frameHeight));
			flip(frame, frame, 1);
			frame.copyTo(result);
			if (doInitialise)
			{
				if (tracker.IsInitialised())
				{
					tracker.Reset();
				}
				else
				{
					tracker.Initialise(frame, tracker_init.initBB);
				}
				doInitialise = false;
			}
			else if (!tracker.IsInitialised())
			{
				rectangle(result, tracker_init.initBB, CV_RGB(255, 255, 255));
			}
		}
		else
		{			
			char imgPath[256];
			sprintf(imgPath, tracker_init.imgFormat.c_str(), frameInd);
			cv::Mat frameOrig = cv::imread(imgPath, 0);
			if (frameOrig.empty())
			{
				ROS_ERROR_STREAM("error: could not read frame: " << imgPath);
				return EXIT_FAILURE;
			}
			resize(frameOrig, frame, cv::Size(conf.frameWidth, conf.frameHeight));
			cvtColor(frame, result, CV_GRAY2RGB);
		
			if (frameInd == tracker_init.startFrame)
			{
				tracker.Initialise(frame, tracker_init.initBB);
			}
		}
		
		if (tracker.IsInitialised())
		{
			tracker.Track(frame);
			
			if (!conf.quietMode && conf.debugMode)
			{
				tracker.Debug();
			}
			
			rectangle(result, tracker.GetBB(), CV_RGB(0, 255, 0));
			
			if (outFile)
			{
				const FloatRect& bb = tracker.GetBB();
				outFile << bb.XMin()/tracker_init.scaleW << "," << bb.YMin()/tracker_init.scaleH << "," << bb.Width()/tracker_init.scaleW << "," << bb.Height()/tracker_init.scaleH << std::endl;
			}
		}
		
		if (!conf.quietMode)
		{
			imshow("result", result);
			int key = cv::waitKey(paused ? 0 : 1);
			if (key != -1)
			{
				if (key == 27 || key == 113) // esc q
				{
					break;
				}
				else if (key == 112) // p
				{
					paused = !paused;
				}
				else if (key == 105 && tracker_init.useCamera)
				{
					doInitialise = true;
				}
			}
			if (conf.debugMode && frameInd == tracker_init.endFrame)
			{
				ROS_INFO_STREAM("\n\nend of sequence, press any key to exit");
				cv::waitKey();
			}
		}
	}
	
	if (outFile.is_open())
	{
		outFile.close();
	}
	
	return EXIT_SUCCESS;
}
