# README #

This package contains parameters and launch files for starting the Donkey Car
vehicle control node.

This node is intended to only be run locally on the Donkey Car.

## Controlling the Donkey Car with a game controller ##

This node can be used in conjunction with the joystick\_controller\_2d node to
control the Donkey Car with a USB game controller:

1. Probably you will want to plug the game controller into a remote machine
(e.g. a desktop or laptop). Before you do, make sure your ROS networking
settings are configured properly on both the Donkey Car and on the remote
machine. See: http://wiki.ros.org/ROS/NetworkSetup
1. On the Donkey Car:
    1. Run `roscore`
    1. Check that `debug_only` for throttle and steering are set to `false` in
the `actuator_params.yaml` of the `donkey_vehicle_controller` package (setting
the values to `true` prints controls to console rather than sending them to the
motors)
    1. Run `roslaunch donkey_vehicle_controller launch.launch`
1. On the remote machine:
    1. Plug in the controller
    1. Configure the `joystick_controller_2d` node for your controller
(see the README in that package for details)
    1. Run `roslaunch joystick_controller_2d joystick_controller_2d.launch`

You should now be able to control the donkey car with the controller.
