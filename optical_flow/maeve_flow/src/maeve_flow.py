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
    res = cv2.resize(img,None,fx=scale, fy=scale, interpolation = cv2.INTER_CUBIC)
    half_width_diff = (res.shape[1] - img.shape[1]) / 2
    half_height_diff = (res.shape[0] - img.shape[0]) / 2
    start_y = half_height_diff
    end_y = res.shape[0] - half_height_diff
    start_x = half_width_diff
    end_x = res.shape[1] - half_width_diff
    return cv2.resize(res[start_y:end_y, start_x:end_x], (img.shape[1], img.shape[0]))

def Norm(frame_prv, frame_nxt, frame):
    l = []
    l.append(cv2.norm(frame_prv, frame, cv2.NORM_L1))
    l.append(cv2.norm(frame_nxt, frame, cv2.NORM_L1))
    return l

class Handler:
    def __init__(self, frame_buffer=4):
        self.l = frame_buffer
        self.frames = deque([None] * self.l)
        self.hsv = None
        self.bridge = CvBridge()
        self.pub_gs = rospy.Publisher('/maeve_gs', Image, queue_size=10)
        self.pub_flow = rospy.Publisher('/maeve_flow', Image, queue_size=10)
        self.pub_image = rospy.Publisher('/maeve_image', Image, queue_size=10)
        self.pub_diff = rospy.Publisher('/maeve_diff', Image, queue_size=10)

    def vizFlow(self, flow):
        mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
        self.hsv[...,0] = ang*180/np.pi/2
        self.hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
        return cv2.cvtColor(self.hsv,cv2.COLOR_HSV2BGR)

    def callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            print(e)
            return

        if self.hsv is None:
            self.hsv = np.zeros_like(cv_image)
            self.hsv[...,1] = 255

        cv_image_gs = cv2.cvtColor(cv_image,cv2.COLOR_BGR2GRAY)

        for i in range(0,self.l-1):
            if self.frames[i] is None:
                self.frames[i] = cv_image_gs
                return

        if self.frames[self.l-1] is not None:
            self.frames.rotate(-1)

        self.frames[self.l-1] = cv_image_gs

        # Averaging
        frame_prv = cv2.addWeighted(self.frames[0], 0, self.frames[0], 1, 0)
        frame_nxt = cv2.addWeighted(self.frames[1], 0, self.frames[3], 1, 0)

        # Temporal Diff
        cv_diff = cv2.subtract(frame_nxt, frame_prv)
        cv_diff = cv2.normalize(cv_diff, cv_diff, 0, 255, cv2.NORM_MINMAX) 

        # Thresholding
        ret, cv_diff = cv2.threshold(cv_diff, 0, 255, cv2.THRESH_TOZERO)
        diff_output = self.bridge.cv2_to_imgmsg(cv_diff, encoding="passthrough")

        # Optical Flow
        flow = cv2.calcOpticalFlowFarneback(frame_prv, frame_nxt, None, 0.5, 5, 15, 3, 7, 1.5, 0)
        cv_output = self.vizFlow(flow)
        flow_output = self.bridge.cv2_to_imgmsg(cv_output, encoding="passthrough")

        # Edges
        edges = cv2.Canny(frame_nxt,50,200)
        edge_img = self.bridge.cv2_to_imgmsg(edges, encoding="passthrough")

        # Resize pyramid (generate in terms of time, not frames)
        res_quarter = Resize(frame_prv, 1.25)
        r125 = self.bridge.cv2_to_imgmsg(res_quarter, encoding="passthrough")
        res_half = Resize(frame_prv, 1.5)
        r15 = self.bridge.cv2_to_imgmsg(res_half, encoding="passthrough")
        res_three_quarter = Resize(frame_prv, 1.75)
        r175 = self.bridge.cv2_to_imgmsg(res_three_quarter, encoding="passthrough")
        res_two = Resize(frame_prv, 2.0)
        r2 = self.bridge.cv2_to_imgmsg(res_two, encoding="passthrough")
        diffs = []
        diffs.append(Norm(frame_prv, frame_nxt, res_quarter))
        diffs.append(Norm(frame_prv, frame_nxt, res_half))
        diffs.append(Norm(frame_prv, frame_nxt, res_three_quarter))
        diffs.append(Norm(frame_prv, frame_nxt, res_two))
        for d in diffs:
            print 'delta: ' + str(d[1])
            #delta = d[1] - d[0]
            #break
            #print 'prv: ' + str(d[0]) + ', nxt: ' + str(d[1]) + ', delta: ' + str(delta)

        print 'END'

        # Publish
        #self.pub_flow.publish(flow_output)
        self.pub_flow.publish(r125)
        
        #self.pub_image.publish(msg)
        #self.pub_image.publish(edge_img)
        self.pub_image.publish(r15)

        #self.pub_gs.publish(self.bridge.cv2_to_imgmsg(cv_image_gs, encoding="passthrough"))
        self.pub_gs.publish(r175)

        #self.pub_diff.publish(diff_output)
        self.pub_diff.publish(self.bridge.cv2_to_imgmsg(cv_image_gs, encoding="passthrough"))
        #self.pub_diff.publish(r2)

