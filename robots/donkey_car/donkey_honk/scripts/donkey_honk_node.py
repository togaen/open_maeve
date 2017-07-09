#!/usr/bin/env python
# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import rospy

import donkey_honk

if __name__ == '__main__':
    rospy.init_node('donkey_honk')
    buzzer = donkey_honk.Buzzer(27, 0.25)
    buzzer.beep()

