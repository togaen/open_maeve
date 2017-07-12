#!/usr/bin/env python
# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

from collections import deque

import rospy
import ros_parameter_loading
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from cv_bridge import CvBridge, CvBridgeError

import cv2

import encroachment_detection

## @package encroachment_detection
# Encroachment detection using simple image dilation detection.


##
# @brief Handler class for containing the message callback.
class Handler:

    ##
    # @brief Construct a callback handler.
    #
    # @param p Node parameters from ros_parameter_loading.

    def __init__(self, p):
        self.p = p
        self.low_pass_filter = 0
        self.skip_count = 0
        self.frames = deque([None, None])
        self.bridge = CvBridge()
        self.pub = rospy.Publisher(
            rospy.get_name() + '/' + self.p.output_topic,
            Bool,
            queue_size=10)

    ##
    # @brief Convenience method for invoking the dilation metric.
    #
    # @param img1 The first argument to the metric.
    # @param img2 The second argument to the metric.
    #
    # @return The measure.
    def getMetric(self, img1, img2):
        return encroachment_detection.DilationMetric(
            img1, img2,
                self.p.enable_median_filter,
                self.p.median_filter_window,
                self.p.enable_blur_filter,
                self.p.blur_filter_window)

    ##
    # @brief Detect encroachment by performing a match to a scale pyramid
    #
    # @return True if encroachment detected; otherwise false.
    def detectEncroachment(self):
        encroachment_detected = False
        scale_pyramid = encroachment_detection.BuildScalePyramid(
            self.frames[0], self.p.scales)
        for key, value in scale_pyramid.items():
            bg_m = self.getMetric(self.frames[0], self.frames[1])
            m = self.getMetric(self.frames[1], value)
            if (m - bg_m) < 0:
                # Could return here, but let's keep run times deterministic.
                encroachment_detected = True

            # rospy.loginfo(str(key) + ' bg_m: ' + str(bg_m) + ', m: ' + str(m)
            # + ', delta: ' + str(m-bg_m) + ', E: ' +
            # str(encroachment_detected))

        return encroachment_detected

    ##
    # @brief Perform a low-pass filter on a detection signal.
    #
    # @param detection The detection signal (True|False)
    #
    # @return True if the detection
    def filterDetection(self, detection):
        if detection:
            self.low_pass_filter = self.low_pass_filter + 1
            if self.low_pass_filter >= self.p.low_pass_filter:
                self.low_pass_filter = 0
                return True

        return False

    ##
    # @brief Set up the image queue for detection processing.
    #
    # @param cv_image The input image.
    #
    # @return True if the queue has exactly two images; otherwise False.
    def imageQueueReady(self, cv_image):
        # Set first queue position; this should only execute once.
        if self.frames[0] is None:
            self.frames[0] = cv_image
            return False

        # Skip any frames?
        if self.skip_count < self.p.skip_frames:
            self.skip_count = self.skip_count + 1
            return False

        # All frames have been skipped, reset counter.
        self.skip_count = 0

        # If deque is full, rotate out the first image.
        if self.frames[1] is not None:
            self.frames.rotate(-1)

        # Add current frame to end of deque.
        self.frames[1] = cv_image

        return True

    def preprocessImage(self, cv_image):
        # Resize the incoming image.
        cv_image = cv2.resize(
            cv_image,
            None,
            fx=self.p.scale_input,
            fy=self.p.scale_input,
            interpolation=cv2.INTER_AREA)

        # Median filter to reduce noise
        if self.p.enable_median_filter:
            cv_image = cv2.medianBlur(cv_image, self.p.median_filter_window)

        # Blur to smooth the metric function
        if self.p.enable_blur_filter:
            cv_image = cv2.blur(
                cv_image,
                (self.p.blur_filter_window,
                 self.p.blur_filter_window))

        return cv_image

    ##
    # @brief The camera message callback. For each message, generate a scale pyramid and perform a matching.
    #
    # @param msg The image message.
    def callback(self, msg):
        # Convert ROS image to 8-bit grayscale OpenCV image.
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            rospy.logerr(e)
            return

        # Do any pre-processing.
        cv_image = self.preprocessImage(cv_image)

        # Set up image queue.
        if not self.imageQueueReady(cv_image):
            return

        # Perform detection.
        detection = self.detectEncroachment()

        # Filter detection.
        detection = self.filterDetection(detection)

        if self.p.verbose and detection:
            rospy.loginfo('ENCROACHMENT DETECTED')

        # Publish findings.
        msg = Bool()
        msg.data = detection
        self.pub.publish(msg)


if __name__ == '__main__':
    rospy.init_node('encroachment_detection')
    node_params = ros_parameter_loading.NodeParams()

    handler = Handler(node_params)
    rospy.Subscriber(
        node_params.camera_topic,
        Image,
     handler.callback)
    rospy.loginfo('Running encroachment detection...')
    rospy.spin()
