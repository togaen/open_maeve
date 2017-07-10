#!/usr/bin/env python
# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy
from std_msgs.msg import Bool

import ros_parameter_loading
import rpi_piezo_buzzer

## @package rpi_piezo_buzzer
# ROS node for activating the Raspberry Pi Piezo buzzer.


##
# @brief Handler class for Piezo buzzer callback.
class Handler:

    ##
    # @brief Instantiate the piezo buzzer control.
    #
    # @param p Buzzer params.

    def __init__(self, p):
        self.buzzer = rpi_piezo_buzzer.Buzzer(
            p.buzzer_gpio_pin,
            p.buzzer_duration,
            p.buzzer_on_duration,
            p.buzzer_off_duration)

    ##
    # @brief Activate the buzzer for each 'True' signal on the topic.
    #
    # @param msg The activation message.
    def callback(self, msg):
        if not msg.data:
            return

        self.buzzer.buzz()

if __name__ == '__main__':
    rospy.init_node('rpi_piezo_buzzer')
    node_params = ros_parameter_loading.NodeParams()

    handler = Handler(node_params)
    rospy.Subscriber(
        node_params.buzzer_topic,
        Bool,
     handler.callback)
    rospy.spin()
