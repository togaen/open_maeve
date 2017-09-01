# README #

This package uses the [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar) package to generate and visualize Composite Image Space Potential (CISP) fields from AR tags. It is intended to serve as an example of how to compute CISP fields.

## ar\_isp\_field ##

The node defined by this package computes performs the following for each detected AR tag:

* An AR tag pose is projected onto the camera image plane.
* The max extent for the AR tag is computed and recorded.
* The time derivative of the max extent for the tag is estimated using previous measurements.
* The time-to-contact measure, tau, is computed for the AR tag.
* An Image Space Potential field is computed such that each pixel in the area of the image occuped by the AR tag takes the potential value f(tau) and all other pixels take the value 0.

Once ISPs have been computed for each detected tag, they are composed into a CISP field.

## Usage ##

In order to use the node:

* Prepare a set of AR tag markers for use. See [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar).
* Add marker ids to the params.yaml file.
* A camera must be used that has a calibration available. If the camera does not have a calibration available, the node cannot work.
* Assuming a calibrated camera is available, configure the ar\_isp\_field params to listen to the proper camera topic
* Configure launch/markers.launch to listen to the correct camera topic.
* Use the launch.launch file to start this node and launch the [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar) node, which provides AR tag tracking information.
* Use the provided rviz.launch launch file to start rviz with a window camera and CISP field visualization output (you may need to modify the camera topic to whatever you are using).

## Subscribe ##

* /cv\_camera\_node/image\_raw (sensor\_msgs/Image): The input camera stream

## Publish ##

* /viz\_isp\_field (sensor\_msgs/Image): The CISP field visualization

## Parameters: params/params.yaml ##

The below parameters govern node behavior:

* camera\_topic (default '/cv\_camera\_node/image\_raw'): The input camera topic.
* viz\_isp\_field\_topic (default 'viz\_isp\_field'): Node-relative topic for visualizing output, leave empty to disable visualization
* viz\_potential\_bounds: Bounds for scaling potential values for visualization; must be two elements with first less than zero and second greater than zero.
* verbose (default 'false'): Print output to terminal during exectution?

The below values govern potential transform behavior:

* hard\_constraint\_transform: Shape and constraint parameters for the hard constraint potential transform:
    * alpha (default 1.0)
    * beta (default 1.)
    * range\_min: Values greater than this and less than range\_max map to infinity.
    * range\_max: Values less than this and greater than range\_min map to infinity.
* soft\_constraint\_transform: Shape and constraint parameters for the soft constraint potential transform:
    * alpha (default 1.0)
    * beta (default 1.0)
    * range\_min: Infimum of potential value mapping
    * range\_max: Supremum of potential value mapping

The below values govern AR tag tracking behavior:

* ar\_tag\_max\_age (default 0.5): Maximum age of an AR tag transform to consider the transform valid (seconds).
* ar\_time\_queue\_size (default 10): Number of measurements for each tag to maintain
* ar\_time\_queue\_max\_gap (default 0.5): Maximum age to consider a measurement for a given tag to be connected to a previous measurement of the same tag.
* output\_frame\_param\_name (default '/ar\_track\_alvar/output\_frame'): Get the AR pose frame parent from this parameter.
* marker\_size\_param\_name (default '/ar\_track\_alvar/marker\_size'): Get the size of the AR tags from this parameter.
* ar\_frame\_prefix (default 'ar\_marker\_'): Prefix shared by frame names for AR tags.
* ar\_tag\_ids: A list of AR tag IDs that are to be tracked.

## RViz Configs ##

* rviz\_configs/config.rviz: Included for convenience. Displays input image and CISP field visualization in RViz.

## Launch Files ##

* launch.launch: This loads the `params/params.yaml` parameter file and launches all nodes.
* markers.launch: This loads the ar\_track\_alvar node to perform AR tag tracker.
* rviz.launch: This loads RViz configured to display camera output and CISP field visualizations.
