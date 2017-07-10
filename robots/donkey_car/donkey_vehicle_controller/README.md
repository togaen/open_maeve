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

## Subscribe ##

* \[user-defined Command2D topic\] (see 'command2D\_topic' param): Input commands for the controller.

## Parameters: params/actuator\_params.yaml ##

* command2D\_topic: The topic to listen to for command messages (default '/joystick\_controller\_2d/command')
* throttle\_actuator: Channel and PWM settings for the throttle (x control axis) actuator (defaults below)
    * debug\_only: false
    * channel: 0
    * max\_pulse: 500
    * zero\_pulse: 370
    * min\_pulse: 220
* steering\_actuator: Channel and PWM settings for the steering (y control axis) actuator (defaults below)
    * debug\_only: false
    * channel: 1
    * max\_pulse: 460
    * zero\_pulse: 360
    * min\_pulse: 260

## Launch Files ##

* launch.launch: Launches the control node and listens to the command2D\_topic.
