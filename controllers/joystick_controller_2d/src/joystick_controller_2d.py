# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy
from controller_interface_msgs.msg import Command2D

# @package joystick_controller_2d
# Mapping functionality from joystick message to Command2D message


##
# @brief Map a joystick message to a Command2D message.
#
# The function map up/down motions of the left joystick hat to x axis
# commands, and the left/right motions of the right joystick hat to y axis
# commands.
#
# @return The equivalent Command2D message.


def JoyMsgToCommand2DMsg(
        m,
        x_axis,
        x_gain,
        y_axis,
        y_gain):
    cmd_msg = Command2D()
    cmd_msg.header = m.header
    cmd_msg.sticky_control = True
    cmd_msg.x = x_gain * m.axes[x_axis]
    cmd_msg.y = y_gain * m.axes[y_axis]
    return cmd_msg
