#!/usr/bin/env python
# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy
from sensor_msgs.msg import Image
import maeve_flow

if __name__ == '__main__':
    rospy.init_node('optical_flow')

    handler = maeve_flow.Handler()
    rospy.Subscriber(
        '/raspicam_node/image_republished_raw',
        Image,
     handler.callback)
    rospy.spin()

