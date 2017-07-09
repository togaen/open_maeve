#!/usr/bin/env python
# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

from collections import deque

import rospy
import ros_parameter_loading
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import cv2

import maeve_flow

## @package maeve_flow
# Simple, robust encroachment detection.


##
# @brief Handler class for containing the message callback.
class Handler:

    ##
    # @brief Construct a callback handler.
    #
    # @param p Node parameters from ros_parameter_loading.

    def __init__(self, p):
        self.p = p
        self.skip_count = 0
        self.frames = deque([None, None])
        self.bridge = CvBridge()
        self.publishers = {}
        for scale in p.scales:
            topic_name = self.ScaleTopic(scale)
            self.publishers[topic_name] = rospy.Publisher(
                topic_name, Image, queue_size=10)

    ##
    # @brief Compute a standard ROS topic name to publish a scaled image.
    #
    # @param scale The image dilation scale.
    #
    # @return The string topic name.
    def ScaleTopic(self, scale):
        return rospy.get_name() + '/' + self.p.scaled_image_topic_prefix + str(scale).replace('.', '_')

    ##
    # @brief The camera message callback. For each message, generate a scale pyramid and perform a matching.
    #
    # @param msg The image message.
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
        scale_pyramid = maeve_flow.BuildScalePyramid(
            self.frames[0], self.p.scales)
        for key, value in scale_pyramid.items():
            topic_name = self.ScaleTopic(key)
            delta = cv2.norm(self.frames[0], value, cv2.NORM_L1)
            print topic_name + ' delta: ' + str(delta)
            scaled_images[topic_name] = self.bridge.cv2_to_imgmsg(
                value, encoding="passthrough")

        print 'END'

        # Publish scaled images.
        for key, value in scaled_images.items():
            self.publishers[key].publish(value)


if __name__ == '__main__':
    rospy.init_node('maeve_flow')
    node_params = ros_parameter_loading.NodeParams()

    handler = Handler(node_params)
    rospy.Subscriber(
        node_params.camera_topic,
        Image,
     handler.callback)
    rospy.spin()
