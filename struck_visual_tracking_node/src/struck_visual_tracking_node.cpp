#include "params.h"
#include "struck_tracker.h"

#include <ros/ros.h>

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
	auto struck_tracker = StruckTracker(params.toStruckConfig(), params.camera_topic);
	if (!StruckVisualTrackingParams::SanityCheckStruckConfig(struck_tracker.conf)) {
    ROS_INFO_STREAM("Struck config object failed sanity check.");
		return EXIT_FAILURE;
	}

	if (!struck_tracker.tracker_init.valid) {
		return EXIT_FAILURE;
	}

  // If camera topic is not set, run tracker from config file.
	if (struck_tracker.runFromCameraTopic) {
		// Loop here.
		// Exit.
	}

	if (!struck_tracker.runTracker()) {
		return EXIT_FAILURE;
	}
	return EXIT_SUCCESS;
}
