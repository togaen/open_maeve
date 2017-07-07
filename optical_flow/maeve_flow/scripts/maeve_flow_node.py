#!/usr/bin/env python
# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy
from sensor_msgs.msg import Image
import ros_parameter_loading
import maeve_flow

if __name__ == '__main__':
    rospy.init_node('maeve_flow')
    node_params = ros_parameter_loading.NodeParams()

    handler = maeve_flow.Handler(node_params)
    rospy.Subscriber(
        '/raspicam_node/image_republished_raw',
        Image,
     handler.callback)
    rospy.spin()
