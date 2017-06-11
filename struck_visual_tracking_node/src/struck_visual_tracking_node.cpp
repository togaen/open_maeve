#include "struck_tracker.h"

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

int main(int argc, char* argv[]) {
	const auto node_name = std::string("struck_visual_tracking_node");

	// Initialize ROS node.
	ros::init(argc, argv, node_name);
	ros::NodeHandle nh(node_name);

	// Initialize STRUCK tracker.
	auto struck_tracker = StruckTracker(nh);
	if (!struck_tracker.valid()) {
    ROS_INFO_STREAM("Struck config object failed sanity check.");
		return EXIT_FAILURE;
	} 

	// This topic enables user to initialize tracking.
	auto init_sub = nh.subscribe(struck_tracker.params.init_tracker_topic, 1000, &StruckTracker::userInitCallback, &struck_tracker);

	// Set camera image stream topic.
	auto camera_sub = nh.subscribe(struck_tracker.params.camera_topic, 1000, &StruckTracker::cameraCallback, &struck_tracker);

  // Kick it off.
	ros::spin();

	// Done.
	return EXIT_SUCCESS;
}
