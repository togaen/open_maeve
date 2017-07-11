#!/usr/bin/env python
# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy
import ros_parameter_loading
import adafruit_pca9685
from controller_interface_msgs.msg import Command2D

## @package donkey_vehicle_controller
# Vehicle controller for the donkey car.


##
# @brief Map an index `idx` of range 1 onto range 2.
#
# @note Behavior undefined if idx \notin [range1_min, range1_max] or if either
# range is invalid.
#
# @param idx An index into range 1.
# @param range1_min Minimum of range 1.
# @param range1_max Maximum of range 1.
# @param range2_min Minimum of range 2.
# @param range2_max Maximum of range 2.
#
# @return The index into range 2 whose offset is proportional to that of idx into range 1.
def map_index(idx, range1_min, range1_max, range2_min, range2_max):
    alpha = (idx - range1_min) / (range1_max - range1_min)
    range2_offset = alpha * (range2_max - range2_min)
    return int(range2_min + range2_offset)


##
# @brief Convenience wrapper for loading a servo controller and performing command scaling.
class Controller:
    RANGE_MIN = -1
    RANGE_MAX = 1

    ##
    # @brief Instantiate a servo controller and keep a local copy of the params.
    #
    # @param p The param set for the servo controller.
    def __init__(self, p):
        self.controller = adafruit_pca9685.PCA9685_Controller(p['channel'])
        self.params = p

    ##
    # @brief Map a command `cmd` \in [-1,1] onto [min_pulse,max_pulse] centered on `zero_pulse`.
    #
    # @param cmd The input control command.
    def update(self, cmd):
        if cmd < 0:
            pulse = map_index(
                cmd,
                self.RANGE_MIN,
                0,
                self.params['min_pulse'],
                self.params['zero_pulse'])
        else:
            pulse = map_index(
                cmd,
                0,
                self.RANGE_MAX,
                self.params['zero_pulse'],
                self.params['max_pulse'])

        if self.params['debug_only']:
            rospy.loginfo('set ' + str(pulse) + ' to channel ' + str(self.params['channel']))
        else:
            self.controller.set_pulse(pulse)


##
# @brief Handler class for control command callback.
class Handler:

    ##
    # @brief Instantiate controllers for throttle and steering.
    #
    # @param p Servo params.

    def __init__(self, p):
        self.throttle_controller = Controller(p.throttle_actuator)
        self.steering_controller = Controller(p.steering_actuator)

    ##
    # @brief Read control command from message, send to controller.
    #
    # @param msg The control command message.
    def callback(self, msg):
        self.throttle_controller.update(msg.x)
        self.steering_controller.update(msg.y)

if __name__ == '__main__':
    rospy.init_node('donkey_vehicle_controller')
    node_params = ros_parameter_loading.NodeParams()

    handler = Handler(node_params)
    rospy.Subscriber(
        node_params.command2D_topic,
        Command2D,
     handler.callback)
    rospy.spin()
