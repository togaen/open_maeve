# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy
from controller_interface_msgs.msg import Command2D

## @package joystick_controller_2d
# Mapping functionality from joystick message to Command2D message


##
# @brief Parameters used by the script to transform from joystick to Command2D.
JOYSTICK_X_AXIS = 1
JOYSTICK_Y_AXIS = 0
JOYSTICK_X_GAIN = 1.0
JOYSTICK_Y_GAIN = 1.0

##
# @brief Map a joystick message to a Command2D message.
#
# The function map up/down motions of the left joystick hat to x axis
# commands, and the left/right motions of the right joystick hat to y axis
# commands.
#
# @param m The input joystick message.
#
# @return The equivalent Command2D message.


def JoyMsgToCommand2DMsg(m, x_axis=3, y_axis=1):
    cmd_msg = Command2D()
    cmd_msg.header = m.header
    cmd_msg.sticky_control = True
    cmd_msg.x = JOYSTICK_X_GAIN * m.axes[JOYSTICK_X_AXIS]
    cmd_msg.y = JOYSTICK_Y_GAIN * m.axes[JOYSTICK_Y_AXIS]
    return cmd_msg
