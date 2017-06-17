#!/usr/bin/env python
# license removed for brevity

import rospy
import ros_parameter_loading

if __name__ == '__main__':
    rospy.init_node('donkey_vehicle_controller')
    node_params = ros_parameter_loading.NodeParams(['throttle_actuator', 'steering_actuator'])
    print node_params.throttle_actuator
    print node_params.steering_actuator
