# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy
from controller_interface_msgs.msg import Command2D

## @package joystick_controller_2d
# Mapping functionality from joystick message to Command2D message

##
# @brief Map a joystick message to a Command2D message.
#
# The function map up/down motions of the left joystick hat to x axis
# commands, and the left/right motions of the right joystick hat to y axis
# commands. The default axis mapping is tested on one controller, adjust as
# needed.
#
# @param m The input joystick message.
# @param x_axis The raw joystick axis index for x axis control commands.
# @param y_axis The raw joystick axis index for y axis control commands.
#
# @return The equivalent Command2D message.


def JoyMsgToCommand2DMsg(m, x_axis=3, y_axis=1):
    cmd_msg = Command2D()
    cmd_msg.header = m.header
    cmd_msg.lazy_publishing = True
    cmd_msg.x = m.axes[x_axis]
    cmd_msg.y = m.axes[y_axis]
    return cmd_msg
