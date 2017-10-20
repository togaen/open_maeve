# README #

This package uses the [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar) package to generate and visualize Image Space Potential (ISP) fields from AR tags. It is intended to serve as an example of how to compute ISP fields.

## ar\_isp\_field ##

The node defined by this package computes performs the following for each detected AR tag:

1. An AR tag pose is projected onto the camera image plane.
1. The max extent for the AR tag is computed and recorded.
1. The time derivative of the max extent for the tag is estimated using previous measurements.
1. The time-to-contact measure, tau, is computed for the AR tag.
1. An Image Space Potential field is computed such that each pixel in the area of the image occuped by the AR tag takes the potential value f(tau) and all other pixels take the value 0.

Once ISPs have been computed for each detected tag, they are composed into a ISP field.

## Usage ##

In order to use the node:

1. Prepare a set of AR tag markers for use. See [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar).
1. Add marker ids to the params.yaml file.
1. A camera must be used that has a calibration available. If the camera does not have a calibration available, the node cannot work.
1. Assuming a calibrated camera is available, configure the ar\_isp\_field params to listen to the proper camera topic
1. Configure launch/markers.launch to listen to the correct camera topic.
1. Use the launch.launch file to start this node and launch the [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar) node, which provides AR tag tracking information.
1. Use the provided rviz.launch launch file to start rviz with a window camera and ISP field visualization output (you may need to modify the camera topic to whatever you are using).
1. Launch the joystick\_controller\_2d node to provide guidance control commands to the node
1. Output control commands are published to the node's 'command' topic

## Subscribe ##

* /cv\_camera\_node/image\_raw (sensor\_msgs/Image): The input camera stream
* /joystick\_controller\_2d/command (controller\_interface\_msgs/Command2D): The input desired control

## Publish ##

* ~/viz\_isp\_field (sensor\_msgs/Image): The ISP field visualization
* ~/command (controller\_interface\_msgs/Command2D): The output control command

## Parameters ##

This node has several parameter files, described below. For information about
the individual parameters in the files, see the comments in those files.

* params/node\_params.yaml: These parameters specify topics that are published to and subscribed, as well as visualization and node parameters and path to taxonomy
* params/controller\_params.yaml: These parameters specify behavior for the ISP controller (see the README in the isp\_controller\_2d package for more information about these parameters)
* params/isp\_field\_params.yaml: These parameters defined the potential transforms used to project guidance weights into ISP space (see the README in the isp\_field package for more information about these parameters)

Note that there are also camera parameters specified in the markers.launch launch file. These need to correspond to the camera parameters in params.yaml. At some point parameters should be unified between nodes.

## RViz Configs ##

* rviz\_configs/config.rviz: Included for convenience. Displays input image and ISP field visualization in RViz.

## Launch Files ##

* launch.launch: This loads the `params/params.yaml` parameter file and launches all nodes.
* markers.launch: This loads the ar\_track\_alvar node to perform AR tag tracker. Note: the camera parameters in this launch file need to correspond to those specified in params.yaml
* rviz.launch: This loads RViz configured to display camera output and ISP field visualizations.
