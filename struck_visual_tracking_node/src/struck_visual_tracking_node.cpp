#include "params.h"
#include "struck_tracker.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

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
	auto struck_tracker = StruckTracker(params, nh);
	if (!StruckVisualTrackingParams::SanityCheckStruckConfig(struck_tracker.conf)) {
    ROS_INFO_STREAM("Struck config object failed sanity check.");
		return EXIT_FAILURE;
	}

	if (!struck_tracker.tracker_init.valid) {
		return EXIT_FAILURE;
	}

	// This topic enables user to initialize tracking.
	auto init_sub = nh.subscribe("init_tracker", 1000, &StruckTracker::userInitCallback, &struck_tracker);

  // If camera topic is set, run from camera topic.
	if (struck_tracker.runFromCameraTopic) {
		ROS_INFO_STREAM("Running from camera topic: " << params.camera_topic);
		auto camera_sub = nh.subscribe(params.camera_topic, 1000, &StruckTracker::cameraCallback, &struck_tracker);
		ros::spin();
		return EXIT_SUCCESS;
	}

	ros::AsyncSpinner spinner(1);
	spinner.start();

	// Uncommenting this leads to segfaults when not built in debug mode. Weird.
	//ROS_INFO_STREAM("Running from direct input.");

	// Otherwise, run directly from file/webcam input.
	while (struck_tracker.tracker_init.useCamera) {
		cv_bridge::CvImage frameOrig;
		frameOrig.header.stamp = ros::Time::now();
		frameOrig.encoding = sensor_msgs::image_encodings::TYPE_8UC3; 
		struck_tracker.tracker_init.cap >> frameOrig.image;
		auto img_msg = frameOrig.toImageMsg();
		struck_tracker.cameraCallback(img_msg);
	}

	ros::waitForShutdown();
	return EXIT_SUCCESS;
}
