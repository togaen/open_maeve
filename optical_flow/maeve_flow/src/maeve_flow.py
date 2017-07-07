# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy
import rosbag
import ros_parameter_loading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from collections import deque

import sys
import cv2
import numpy as np


def Resize(img, scale):
    res = cv2.resize(
        img,
        None,
     fx=scale,
     fy=scale,
     interpolation=cv2.INTER_CUBIC)
    half_width_diff = (res.shape[1] - img.shape[1]) / 2
    half_height_diff = (res.shape[0] - img.shape[0]) / 2
    start_y = half_height_diff
    end_y = res.shape[0] - half_height_diff
    start_x = half_width_diff
    end_x = res.shape[1] - half_width_diff
    return cv2.resize(res[start_y:end_y, start_x:end_x], (img.shape[1], img.shape[0]))


class Handler:

    def __init__(self, p):
        self.p = p
        self.skip_count = 0
        self.frames = deque([None, None])
        self.bridge = CvBridge()
        self.publishers = {}
        for scale in p.scale_pyramid:
            topic_name = self.ScaleTopic(scale)
            self.publishers[topic_name] = rospy.Publisher(
                topic_name, Image, queue_size=10)

    def ScaleTopic(self, scale):
        return rospy.get_name() + '/' + self.p.scaled_image_topic_prefix + str(scale).replace('.', '_')

    def callback(self, msg):
        # Convert ROS image to 8-bit grayscale OpenCV image.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            print(e)
            return

        # Set first queue position; this should only execute once.
        if self.frames[0] is None:
            self.frames[0] = cv_image
            return

        # Skip any frames?
        if self.skip_count < self.p.skip_frames:
            self.skip_count = self.skip_count + 1
            return

        # All frames have been skipped, reset counter.
        self.skip_count = 0

        # If deque is full, rotate out the first image.
        if self.frames[1] is not None:
            self.frames.rotate(-1)

        # Add current frame to end of deque.
        self.frames[1] = cv_image

        # Generate resize pyramid.
        scaled_images = {}
        for scale in self.p.scale_pyramid:
            topic_name = self.ScaleTopic(scale)
            scaled_image = Resize(self.frames[0], scale)
            delta = cv2.norm(self.frames[0], scaled_image, cv2.NORM_L1)
            print topic_name + ' delta: ' + str(delta)
            scaled_images[topic_name] = self.bridge.cv2_to_imgmsg(
                scaled_image, encoding="passthrough")

        print 'END'

        # Publish scaled images.
        for key, value in scaled_images.items():
            self.publishers[key].publish(value)
