#include "params.h"

#include <iostream>
#include <fstream>

#include <boost/lockfree/spsc_queue.hpp>
#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/Image.h>

#include "maeve_automation_core/struck_visual_tracking/struck_visual_tracking.h"

void cameraCallback(const sensor_msgs::Image::ConstPtr& msg) {
	// invoke tracker here
}

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

  auto tracker_init = buildTrackerInit(conf, params.camera_topic);
	if (!tracker_init.valid) {
		return EXIT_FAILURE;
	}

  // If camera topic is not set, run tracker from config file.
	if (tracker_init.camera_topic.empty()) {
		// Loop here.
		// Exit.
	}

	if (!runTracker(conf, tracker_init, tracker)) {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
