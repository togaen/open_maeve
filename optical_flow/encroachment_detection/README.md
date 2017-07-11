# README #

This package implements encroachment detection using a very simple image
dilation detection technique in a streaming sequence of camera images. The
detection works by comparing a current frame to dilations of the previous frame
at various scales and looking for below-expected image space distance.

This kind of detection can provide a coarse estimate of motion flow that is
directed either toward or away from the camera. The emphasis is on reactivity
rather than accuracy, so the library is designed to be as fast and lightweight
as is reasonable.

The parameter default values are tuned for camera output from:

    roslaunch maeve_cameras raspicam_republish_rectify.launch

If running from different camera input, you'll probably need to make
adjustments, especially to the scales and filter windows.

## Subscribe ##

* \[user-defined camera topic\] (see 'camera\_topic' param): The camera image stream over which to perform dilation detection.

## Publish ##

* \[user-defined output topic\] (see 'output\_topic' param): Publish detection signals to this topic.

## Parameters ##

* output\_topic: Publish detection signals to this topic: this will be relative to the node name (default '~/detection')
* camera\_topic: The topic to listen to for camera images (default '/raspicam\_node/image\_rect')
* skip\_frames: Number of frames to skip between previous and current frames for dilation computation (default 3 assuming 90fps)
* low\_pass\_filter: Require this many sequential frames with encroachment detection before indicating a positive (default 2)
* scales: The set of dilation scales to use for detection 
* enable\_median\_filter: Whether to pre-process images with a median filter before computing dilation metric (default false)
* median\_filter\_window: Window size for median filter (default 15)
* enable\_blur\_filter: Whether to pre-process iamges with a blur filter before computing dilation metric (default false)
* blur\_filter\_window: Window size for blue filter (default 15)

## Launch Files ##

* launch.launch: Load parameters to ROS parameter server and launch the node.
