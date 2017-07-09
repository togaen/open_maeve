# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import getpass
import os
import RPi.GPIO as GPIO
import time
import re
import locale

class Buzzer:
    def __init__(self, buzzer_pin, buzzer_duration):
        GPIO.setmode(GPIO.BCM)
        self.buzzer_pin = buzzer_pin
        self.buzzer_duration = buzzer_duration
        GPIO.setup(self.buzzer_pin, GPIO.IN)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)

    def __del__(self):
        class_name = self.__class__.__name__
        GPIO.cleanup()

    def beep(self):
        GPIO.output(self.buzzer_pin, True)
        time.sleep(self.buzzer_duration)
        GPIO.output(self.buzzer_pin, False)
