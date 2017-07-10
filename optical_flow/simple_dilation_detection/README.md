# README #

This package implements simple image dilation detection in a streaming sequence
of camera images. The detection works by comparing a current frame to dilations
of the previous frame at various scales.

This kind of detection can provide a coarse estimate of motion flow in a scene
that is directed either toward or away from the camera. The emphasis is on
reactivity rather than accuracy, so the library is designed to be as fast and
lightweight as is reasonable.

## Subscribe ##

* \[user-defined camera topic\] (see 'camera\_topic' param): The camera image stream over which to perform dilation detection.

## Parameters ##

* camera\_topic: The topic to listen to for camera images (default '/raspicam\_node/image\_republished\_raw')
* skip\_frames: Number of frames to skip between previous and current frames for dilation computation (default 0)
* dilated\_image\_topic\_prefix: Scaled images are published to a node-relative topic with this prefix (default 'dilated\_image\_')
* scales: The set of dilation scales to use for detection (default [1.01,1.015,1.03,1.07])

