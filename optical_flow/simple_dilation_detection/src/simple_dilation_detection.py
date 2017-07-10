# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

from collections import deque

import cv2

## @package simple_dilation_detection
# Simple image dilation detection.


##
# @brief An image space metric specialized for dilation detection.
#
# @param img1 The first argument to the metric.
# @param img2 The second argument to the metric.
# @param enable_median_filter Whether to preprocess img1 and img2 with a median filter.
# @param median_filter_window If preprocessing with a median filter, use this window size (square).
# @param enable_blur_filter Whether to preprocess img1 and img2 with a blur filter.
# @param blur_filter_window If preprocessing with a blur filter, use this window size (square).
#
# @return
def DilationMetric(
    img1,
     img2,
     enable_median_filter,
     median_filter_window,
     enable_blur_filter,
     blur_filter_window):
    # Median filter to reduce noise
    if enable_median_filter:
        img1 = cv2.medianBlur(img1, median_filter_window)
        img2 = cv2.medianBlur(img2, median_filter_window)

    # Blur to smooth the metric function
    if enable_blur_filter:
        img1 = cv2.blur(img1, (blur_filter_window, blur_filter_window))
        img2 = cv2.blur(img2, (blur_filter_window, blur_filter_window))

    return cv2.norm(img1, img2, cv2.NORM_L1)

##
# @brief Dilate an image by the given scale.
#
# @param img The input image for dilation.
# @param scale The scale by which to dilate.
#
# @return An image with the same dimensions as img, dilated by scale.


def Dilate(img, scale):
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


##
# @brief For an image and ordered (ASC) list of scales, build a scale pyramid.
#
# @param img The base image of the pyramid.
# @param scales The ordered list of scales.
#
# @return The scale pyramid: a dictionary of scale dialted images with scale values as keys.
def BuildScalePyramid(img, scales):
    scale_pyramid = {}
    for scale in scales:
        scale_pyramid[scale] = Dilate(img, scale)

    return scale_pyramid

##
# @brief Match an image to a scale pyramid.
#
# @param img The image to match.
# @param scale_pyramid The scale pyramid.
#
# @return The interpolated matching index.
#def MatchImage(img, scale_pyramid):

    # TODO introduce some state here.
