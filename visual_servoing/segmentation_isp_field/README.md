# README #

This package consumes images with per-pixel semantic segmentations and computes
ISP fields according to user-defined semantic label weights. The ISP fields are
fed into a controller to then compute Command2D messages for controlling a
ground vehicle.

This packages uses the segmentation\_taxonomy package for loading segmentation
taxonomies and interpreting image segmentations. See the README in that package
for more details.

NOTE: This controller does not support taxonomy IDs with more than one class
association.

## segmentation\_isp\_field ##

The node defined by this package does the following:

1. Loads a user-specified segmentation taxonomy and user-defined weights for segmentation labels
1. Listens to a camera topic for segmentation images
1. For each segmentation image, an ISP field is computed based on segmentation label weights
1. An isp\_controller\_2d instance consumes the ISP field and a desired control to output a control command

## Usage ##

In order to use the node:

1. Define a segmentation taxonomy according to the specification in the segmentation\_taxonomy package and point to it in `params/node\_params.yaml`.
1. Publish segmentation images to a known topic along with a [CameraInfo](http://docs.ros.org/api/sensor_msgs/html/msg/CameraInfo.html) message
1. Use the launch.launch file to start this node and launch the [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar) node, which provides AR tag tracking information.
1. Use the provided rviz.launch launch file to start rviz with a window camera and ISP field visualization output (you may need to modify the camera topic to whatever you are using).
1. Output control commands are published to the node's 'command' topic

## Subscribe ##

* /segmentation-camera-name/image (sensor\_msgs/Image): Segmentation image
* /segmentation-camera-name/camera\_info (sensor\_msgs/CameraInfo): Segmentation camera parameters

## Publish ##

* ~/viz\_\* (sensor\_msgs/Image): The node publishes multiple visualization topics, all prefixed with "viz\_" and all are published as images
* ~/command (controller\_interface\_msgs/Command2D): The output control command

## Parameters ##

This node has several parameter files, described below. For information about
the individual parameters in the files, see the comments in those files.

* params/node\_params.yaml: These parameters specify topics that are published to and subscribed, as well as visualization and node parameters and path to taxonomy
* params/controller\_params.yaml: These parameters specify behavior for the ISP controller (see the README in the isp\_controller\_2d package for more information about these parameters)
* params/isp\_field\_params.yaml: These parameters defined the potential transforms used to project guidance weights into ISP space (see the README in the isp\_field package for more information about these parameters)
* params/guidance\_weights.yaml: These are the user-defined weights for segmentation labels; these weights inform the controller about which labels are "attractive" and which are "repulsive"

## RViz Configs ##

* rviz\_configs/config.rviz: Included for convenience. Displays data set and visualizations of intermediate control computation data structures

## Launch Files ##

* launch.launch: This loads parameter files and launches the node
* rviz.launch: This loads RViz configured to display camera output and visualizations
