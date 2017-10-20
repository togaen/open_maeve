# README #

This package defines a subsumption-based visual servoing controller that
computes Command2D messages given input guidance command and ISP fields. This
controller is designed to provide the local control component of a vision-based
[Selective Determinism](http://hdl.handle.net/2022/21670) architecture.

This controller is implemented as a library that is to be included in a node
that recieves and processes camera input into ISP fields. See the ar\_isp\_field
or segmentation\_isp\_field nodes for examples.

## Parameters ##

Users of this controller are responsible for setting the controller parameters.
For convenience, a ROS interface library is provided that can load the
parameters from the ROS parameter server. The parameter set is described below:

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

