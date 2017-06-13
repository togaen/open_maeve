# README #

This package provides a simple node that performs visual tracking on a camera
stream using the STRUCK visual tracker. The node publishes the bounding box
from the tracker, and, optionally, an image with the bounding box overlaid.

# struck\_node #

The node defines a simple class that invokes the STRUCK tracker on a image
stream. All relevant parameters can be set in the `params.yaml` file. Topics
for publishing/subscribing can also be changed there.

## Usage ##

Using the node is straightforward. Below outlines a typical use case: it assumes you are using the included RViz config, and that the enable\_viz parameter is set true.

1. Make sure images are being published to the camera\_topic set in the params.yaml file. (For instance, if you have the cv\_camera package and a webcam, use the included cv\_camera.launch file to publish images from your webcam.)
1. Open RViz with the included RViz config. You should see a camera feed in the Webcam window.
1. Launch the tracker node with the launch.launch launch file.
1. You now should see tracker output in the Tracker Viz window with a white bounding box.
1. Center the object you wish to track in the white bounding box.
1. When you are ready to track, trigger the initialization with the init\_tracker.launch launch file.
1. The bounding box should turn green, and it should follow the object in the image. The node publishes the bounding box to the tracker\_bb\_topic topic.

## Subscribe ##

* /cv\_camera\_node/image\_raw (sensor\_msgs/Image): The input camera stream
* ~/init\_tracker (std\_msgs/Bool): When `true` is published to this topic, the tracker initializes

## Publish ##

* ~/track\_image (sensor\_msgs/Image): Visualization of tracker output
* ~/track\_bounding\_box (struck\_node/ImageBoundingBox): The tracked bounding box

## Parameters: params/params.yaml ##

The below parameters are specific to the struck\_node wrapper:
* camera\_topic (default /cv\_camera\_node/image\_raw): Listen to this topic for camera image stream.
* camera\_topic\_queue\_size (default 1): Queue size for camera topic.
* init\_tracker\_topic (default ~/init\_tracker): Listen to this topic for user initialization trigger.
* init\_tracker\_topic\_queue\_size (default 1): Queue size for init\_tracker\_topic.
* tracker\_image\_topic (default ~/track\_image): Topic to publish tracker output visualization.
* tracker\_bb\_topic (default ~/track\_bounding\_box): Topic to publish tracker bounding box output.
* enable\_viz (default true): Flag to enable tracker output visualization.

The below parameters are specific to the STRUCK tracker:
* bb\_(x|y)\_(min|max) (default [120 200]x[80 160]): Bounding box used to initialize tracker.
* frame(Width|Height) (default 320x240): Size of image to perform tracking on; input is scaled to these dimensions.
* seed (default 0): Seed for random number generator.
* searchRadius (default 30): Tracker search radius in pixels.
* svmC (default 100.0): SVM regularization parameter.
* svmBudgetSize (default 100): SVM budget size (0 = no budget).
* feature (default 'haar gaussian 0.2'): Image features to use; see params/params.yaml for more details.

## RViz Configs ##

* struck\_node.rviz: Included for convenience. Displays input image and tracker visualization in RViz.

## Messages ##

* ImageBoundingBox.msg: A message that describes a bounding box on an image plane.

## Launch Files ##

* launch.launch: This loads the `params/params.yaml` parameter file and launches the node.
* init\_tracker.launch: This publishes a `true` to the init\_tracker topic to signal that tracking should begin.
* cv\_camera.launch: Included for convenience, this launch file loads default camera parameters `params/cv_camera.yaml` and launches the cv\_camera node that publishes images to `/cv\_camera\_node/image\_raw` using a webcam or a local video file. Note that this requires the cv\_camera package to be installed.
