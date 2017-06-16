#include "./params.h"

namespace maeve_automation_core {
  bool DonkeyVehicleControllerParams::load(const ros::NodeHandle& nh) {
    LOAD_NS_PARAM(throttle_actuator, channel);
    LOAD_NS_PARAM(throttle_actuator, max_pulse);
    LOAD_NS_PARAM(throttle_actuator, zero_pulse);
    LOAD_NS_PARAM(throttle_actuator, min_pulse);

    LOAD_NS_PARAM(steering_actuator, channel);
    LOAD_NS_PARAM(steering_actuator, left_pulse);
    LOAD_NS_PARAM(steering_actuator, right_pulse);

    return true;
  }
}  // namespace maeve_automation_core
