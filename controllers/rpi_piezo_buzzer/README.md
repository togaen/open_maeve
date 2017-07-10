# README #

This package implements a simple node that enables activation of a Piezo buzzer
on a Raspberry Pi. The buzz that is issued is configurable via parameters: the
buzzer is set to be active for a given duration of time, and during that time
it cycles between 'on' and 'off' states.

## Subscribe ##

* \[user-defined activation topic\] (see 'buzzer\_topic' param): Listen for 'true' on this topic to issue a buzz.

## Parameters ##

* buzzer\_topic: The topic to listent to for buzzer activations (default '/rpi\_piezo\_buzzer/activate')
* buzzer\_gpio\_pin: The GPIO pin that the Piezo buzzer is connected to (default 27)
* buzzer\_duration: The length of time in seconds the buzzer should be active (default 0.5)
* buzzer\_on\_duration: The period of time in seconds that the buzzer should be 'on' while it is active (default 0.05)
* buzzer\_off\_duration: The period of time in seconds that the buzzer should be 'off' while it is active (default 0.05)

## Launch Files ##

* launch.launch: Load parameters onto ROS parameter server and start the node.
