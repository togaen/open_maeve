#pragma once

#include "maeve_automation_core/ros_parameter_loading/params_base.h"

namespace maeve_automation_core {

struct DonkeyVehicleControllerParams : public ParamsBase {
  struct ThrottleActuatorParams {
    int channel;
    int max_pulse;
    int zero_pulse;
    int min_pulse;
  };

  struct SteeringActuatorParams {
    int channel;
    int left_pulse;
    int right_pulse;
  };

  ThrottleActuatorParams throttle_actuator;
  SteeringActuatorParams steering_actuator;

  /**
   * @copydoc ParamsBase::ParamsBase()
   */
  __attribute__((warn_unused_result)) bool load(
      const ros::NodeHandle& nh) override;

};  // struct DonkeyVehicleControllerParams

}  // namespace maeve_automation_core
