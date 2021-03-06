# README #

This package implements encroachment detection using a very simple image
dilation detection technique in a streaming sequence of camera images. The
detection works by comparing a current frame to dilations of the previous frame
at various scales and looking for below-expected image space distance.

This kind of detection can provide a coarse estimate of motion flow that is
directed either toward or away from the camera. The library is intended to
serve as a baseline for use in the development of more sophisticated
algorithms.

## Use ##

Use of this node requires some experimentation to set the parameters
appropriately. Notably, the scales are sensitive to relative speed and camera
frame rate. For example, if your robot tends to travel very fast, the scale
should probably have larger steps. Conversely, if your camera runs at a high
frame rate, the scale should probably have lower steps. The bg\_noise\_threshold
determines how sensitive the detector is; a lower threshold hold induces a
higher false positive rate.

For an example use (in conjunction with the rpi\_piezo\_buzzer package), see: [https://youtu.be/QDZJRk6OJZQ](https://youtu.be/QDZJRk6OJZQ)

## Subscribe ##

* \[user-defined camera topic\] (see 'camera\_topic' param): The camera image stream over which to perform dilation detection.

## Publish ##

* \[user-defined output topic\] (see 'output\_topic' param): Publish detection signals to this topic.

## Parameters ##

* output\_topic: Publish detection signals to this topic: this will be relative to the node name (default '~/detection')
* camera\_topic: The topic to listen to for camera images (default '/raspicam\_node/image\_rect')
* verbose: Print information to console during node run time (default true)
* skip\_frames: Number of frames to skip between previous and current frames for dilation computation
* low\_pass\_filter: Require this many sequential frames with encroachment detection before indicating a positive (default 2)
* bg\_noise\_threshold: Assume frames with greater than this distance between them have too much noise for meaningful results
* input\_scale: Scale the incoming image by this scale factor (default 1.0)
* scales: The set of dilation scales to use for detection 
* enable\_median\_filter: Whether to pre-process images with a median filter before computing dilation metric (default false)
* median\_filter\_window: Window size for median filter (default 15)
* enable\_blur\_filter: Whether to pre-process iamges with a blur filter before computing dilation metric (default false)
* blur\_filter\_window: Window size for blue filter (default 15)

## Launch Files ##

* launch.launch: Load parameters to ROS parameter server and launch the node.
