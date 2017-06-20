# README #

This is package uses the [joy](http://wiki.ros.org/joy) package to publish 2D
control control command messages that are generated with joystick input. The
intent is to be able to control mobile agents with a joystick.

## Configuration ##

To configure this node: 

* The `/joy_node/dev` parameter needs to be set in the launch file (the default value `/dev/input/js0` is often okay).
* In `joytsick_controller_2d.py` the `x_axis` and `y_axis` default values need to be set to appropriate values for your controller. The current default values `x_axis=3` and `y_axis=1` map up/down left hat motions to +/- x axis, and left/right hat motions to +/- y axis values for a generic USB game controller. 

For more detailed configutation about the joy package, see: http://wiki.ros.org/joy/Tutorials/ConfiguringALinuxJoystick
