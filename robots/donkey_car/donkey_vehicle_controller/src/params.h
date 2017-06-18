#pragma once

#include "maeve_automation_core/ros_parameter_loading/params_base.h"

namespace maeve_automation_core {

struct DonkeyVehicleControllerParams : public ParamsBase {
  struct ActuatorParams {
    int channel;
    int max_pulse;
    int zero_pulse;
    int min_pulse;
  };

  std::string lat_lon_command_topic;
  ActuatorParams throttle_actuator;
  ActuatorParams steering_actuator;

  /**
   * @copydoc ParamsBase::ParamsBase()
   */
  __attribute__((warn_unused_result)) bool load(
      const ros::NodeHandle& nh) override;

};  // struct DonkeyVehicleControllerParams

}  // namespace maeve_automation_core
