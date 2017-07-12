# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

from collections import deque

import cv2

## @package encroachment_detection
# Simple image dilation detection.


##
# @brief An image space metric for dilation detection.
#
# @param img1 The first argument to the metric.
# @param img2 The second argument to the metric.
#
# @return
def DilationMetric(img1, img2):
    return cv2.norm(img1, img2, cv2.NORM_L1)

##
# @brief Dilate an image by the given scale.
#
# @param img The input image for dilation.
# @param scale The scale by which to dilate.
#
# @return An image with the same dimensions as img, dilated by scale.


def Dilate(img, scale, x=0, y=0):
    scale_x = scale
    if x != 0:
        scale_x = x

    scale_y = scale
    if y != 0:
        scale_y = y

    res = cv2.resize(
        img,
        None,
     fx=scale_x,
     fy=scale_y,
     interpolation=cv2.INTER_LINEAR)
    start_row = (res.shape[0] - img.shape[0]) // 2
    start_col = (res.shape[1] - img.shape[1]) // 2
    return cv2.resize(res[start_row:img.shape[0], start_col:img.shape[1]], (img.shape[1], img.shape[0]))


##
# @brief For an image and ordered (ASC) list of scales, build a scale pyramid.
#
# @param img The base image of the pyramid.
# @param scales The ordered list of scales.
#
# @return The scale pyramid: a dictionary of scale dialted images with scale values as keys.
def BuildScalePyramid(img, scales, x=0, y=0):
    scale_pyramid = {}
    for scale in scales:
        scale_pyramid[scale] = Dilate(img, scale, x, y)

    return scale_pyramid
