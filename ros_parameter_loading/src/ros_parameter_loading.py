# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy

## @package ros_parameter_loading
# Convenience class for loading a set of parameters to an object instance.

## Params class constructs an object whose attributes are the ROS parameters.
#
#  The constructed object has an attribute (member) for each parameter in the
#  given list. The list of passed parameter names is assumed to be relative  to
#  the node name.


class NodeParams:

    def __init__(self):
        node_name = rospy.get_name()
        node_params = rospy.get_param(node_name)
        self.__dict__.update(node_params)
