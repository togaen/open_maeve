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

  // For convenience.
	const auto& params = struck_tracker.params;

	// This topic enables user to initialize tracking.
	auto init_sub = nh.subscribe(params.init_tracker_topic, params.init_tracker_topic_queue_size, &StruckTracker::userInitCallback, &struck_tracker);

	// Set camera image stream topic.
	auto camera_sub = nh.subscribe(params.camera_topic, params.camera_topic_queue_size, &StruckTracker::cameraCallback, &struck_tracker);

  // Kick it off.
	ros::spin();

	// Done.
	return EXIT_SUCCESS;
}
