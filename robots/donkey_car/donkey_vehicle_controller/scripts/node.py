#!/usr/bin/env python
# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy
import ros_parameter_loading
import adafruit_pca9685
from controller_interface_msgs.msg import Command2D

## @package donkey_vehicle_controller
# Vehicle controller for the donkey car.


def map_index(idx, r1_min, r1_max, r2_min, r2_max):
    alpha = (idx - r1_min) / (r1_max - r1_min)
    r2_offset = alpha * (r2_max - r2_min)
    return int(r2_min + r2_offset)


class Controller:
    RANGE_MIN = -1
    RANGE_MAX = 1

    def __init__(self, p):
        self.controller = adafruit_pca9685.PCA9685_Controller(p['channel'])
        self.params = p

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
            print '[debug] set ' + str(pulse) + ' to channel ' + str(self.params['channel'])
        else:
            self.controller.set_pulse(pulse)


class Handler:

    def __init__(self, p):
        self.throttle_controller = Controller(p.throttle_actuator)
        self.steering_controller = Controller(p.steering_actuator)

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
