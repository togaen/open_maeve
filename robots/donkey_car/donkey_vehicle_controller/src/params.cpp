#include "./params.h"

namespace maeve_automation_core {
  bool DonkeyVehicleControllerParams::load(const ros::NodeHandle& nh) {
    LOAD_PARAM(lat_lon_command_topic);

    CHECK_NONEMPTY(lat_lon_command_topic);

    LOAD_NS_PARAM(throttle_actuator, channel);
    LOAD_NS_PARAM(throttle_actuator, max_pulse);
    LOAD_NS_PARAM(throttle_actuator, zero_pulse);
    LOAD_NS_PARAM(throttle_actuator, min_pulse);

    CHECK_GE(throttle_actuator.max_pulse, throttle_actuator.zero_pulse);
    CHECK_LE(throttle_actuator.min_pulse, throttle_actuator.zero_pulse);

    LOAD_NS_PARAM(steering_actuator, channel);
    LOAD_NS_PARAM(steering_actuator, max_pulse);
    LOAD_NS_PARAM(steering_actuator, zero_pulse);
    LOAD_NS_PARAM(steering_actuator, min_pulse);

    CHECK_GE(steering_actuator.max_pulse, steering_actuator.zero_pulse);
    CHECK_LE(steering_actuator.min_pulse, steering_actuator.zero_pulse);

    return true;
  }
}  // namespace maeve_automation_core
