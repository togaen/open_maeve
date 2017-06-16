#include "./params.h"

#include <ros/ros.h>

int main(int argc, char* argv[]) {
  const auto node_name = std::string("donkey_vehicle_controller");

  // Initialize ROS node.
  ros::init(argc, argv, node_name);
  ros::NodeHandle nh(node_name);

  auto params = maeve_automation_core::DonkeyVehicleControllerParams();
  if (params.load(nh)) {
    ROS_INFO_STREAM("Loaded params:\n" << params);
  } else {
    ROS_ERROR_STREAM("Failed to load params!");
  }

  // Done.
  return EXIT_SUCCESS;
}
