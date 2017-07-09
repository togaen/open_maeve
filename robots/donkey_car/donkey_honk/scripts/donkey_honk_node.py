#!/usr/bin/env python
# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy

import ros_parameter_loading
import donkey_honk

## @package donkey_vehicle_controller
# Vehicle controller for the donkey car.


if __name__ == '__main__':
    rospy.init_node('donkey_honk')
    node_params = ros_parameter_loading.NodeParams()

    buzzer = donkey_honk.Buzzer(
        node_params.buzzer_gpio_pin,
        node_params.buzzer_duration,
     node_params.buzzer_on_duration,
     node_params.buzzer_off_duration)
    buzzer.buzz()
