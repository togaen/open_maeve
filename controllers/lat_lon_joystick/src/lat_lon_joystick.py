# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy
from controller_interface_msgs.msg import LatLonCommand

## @package lat_lon_joystick
# Mapping functionality from joystick message to LatLonCommand message

##
# @brief Map a joystick message to a LatLonCommand message.
#
# The function map up/down motions of the left joystick hat to longitudinal
# commands, and the left/right motions of the right joystick hat to lateral
# commands. The default axis mapping is tested on one controller, adjust as
# needed.
#
# @param m The input joystick message.
# @param lat_cmd_axis The axis index for lateral commands.
# @param lon_cmd_axis The axis index for longitudinal commands.
#
# @return The equivalent LatLonCommand message.
def JoyMsg2LatLonCommand(m, lat_cmd_axis=1, lon_cmd_axis=3):
    ll_msg = LatLonCommand()
    ll_msg.lazy_publishing = True
    ll_msg.lat_cmd = m.axes[lat_cmd_axis]
    ll_msg.lon_cmd = m.axes[lon_cmd_axis]
    return ll_msg
