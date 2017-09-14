# README #

This package uses the [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar) package to generate and visualize Image Space Potential (ISP) fields from AR tags. It is intended to serve as an example of how to compute ISP fields.

## ar\_isp\_field ##

The node defined by this package computes performs the following for each detected AR tag:

* An AR tag pose is projected onto the camera image plane.
* The max extent for the AR tag is computed and recorded.
* The time derivative of the max extent for the tag is estimated using previous measurements.
* The time-to-contact measure, tau, is computed for the AR tag.
* An Image Space Potential field is computed such that each pixel in the area of the image occuped by the AR tag takes the potential value f(tau) and all other pixels take the value 0.

Once ISPs have been computed for each detected tag, they are composed into a ISP field.

## Usage ##

In order to use the node:

* Prepare a set of AR tag markers for use. See [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar).
* Add marker ids to the params.yaml file.
* A camera must be used that has a calibration available. If the camera does not have a calibration available, the node cannot work.
* Assuming a calibrated camera is available, configure the ar\_isp\_field params to listen to the proper camera topic
* Configure launch/markers.launch to listen to the correct camera topic.
* Use the launch.launch file to start this node and launch the [ar\_track\_alvar](http://wiki.ros.org/ar_track_alvar) node, which provides AR tag tracking information.
* Use the provided rviz.launch launch file to start rviz with a window camera and ISP field visualization output (you may need to modify the camera topic to whatever you are using).
* Launch the joystick\_controller\_2d node to provide guidance control commands to the node
* Output control commands are published to the node's 'command' topic

## Subscribe ##

* /cv\_camera\_node/image\_raw (sensor\_msgs/Image): The input camera stream
* /joystick\_controller\_2d/command (controller\_interface\_msgs/Command2D): The input desired control

## Publish ##

* ~/viz\_isp\_field (sensor\_msgs/Image): The ISP field visualization
* ~/command (controller\_interface\_msgs/Command2D): The output control command

## Parameters: params/params.yaml ##

Note that there are also camera parameters specified in the markers.launch launch file. These need to correspond to the camera parameters in params.yaml. At some point parameters should be unified between nodes.

The below parameters govern node behavior:

* camera\_topic (default '/cv\_camera\_node/image\_raw'): The input camera topic.
* viz\_isp\_field\_topic (default 'viz\_isp\_field'): Node-relative topic for visualizing output, leave empty to disable visualization.
* viz\_potential\_bounds: Bounds for scaling potential values for visualization; must be two elements with first less than zero and second greater than zero.
* verbose (default 'false'): Print output to terminal during exectution.
* control\_command\_input\_topic (default: '/joystick\_controller\_2d/command'): Absolute topic name for desired command input.
* control\_command\_output\_topic (default: 'command'): Node-relative topic for control command output.

The below values govern potential transform behavior:

* hard\_constraint\_transform: Shape and constraint parameters for the hard constraint potential transform:
    * translation (default 0.0)
    * alpha (default 1.0)
    * beta (default 1.)
    * range\_min: Values greater than this and less than range\_max map to infinity.
    * range\_max: Values less than this and greater than range\_min map to infinity.
* soft\_constraint\_transform: Shape and constraint parameters for the soft constraint potential transform:
    * translation (default 0.0)
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

The below govern the behavior of the ISP controller:

* isp\_controller\_params: Namespace for all ISP controller parameters
    * K\_P: Proportional gain for computing control from potential value
    * K\_D: Derivative gain for computing control from potential value
    * potential\_inertia: The amount of potential change the controller ignores before moving
    * erosion\_kernel: Kernel parameters for min reduction during control horizon computation
        * width: In [0, 1] a proportion of image width
        * height: In [0, 1] a proportion of image height
        * horizon: In [0, 1] with 0 - top and 1 - bottom of image
    * yaw\_decay: Parameters for biasing yaw around the desired value
        * left: Normally should be in [0, 1]
        * right: Normally should be in [0, 1]
    * shape\_parameters: These shape parameters define the output control space.
        * translation: Probably should be zero.
        * range_min: This is taken as the minimum of the control range for output
        * range_max: This is taken as the maximum of the control range for output
        * alpha
        * beta
    * guidance\_gains: Proportional gains for input guidance commands.
        * throttle
        * yaw
        * control_set

## RViz Configs ##

* rviz\_configs/config.rviz: Included for convenience. Displays input image and ISP field visualization in RViz.

## Launch Files ##

* launch.launch: This loads the `params/params.yaml` parameter file and launches all nodes.
* markers.launch: This loads the ar\_track\_alvar node to perform AR tag tracker. Note: the camera parameters in this launch file need to correspond to those specified in params.yaml
* rviz.launch: This loads RViz configured to display camera output and ISP field visualizations.
