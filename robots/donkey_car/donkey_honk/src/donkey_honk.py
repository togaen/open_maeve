# Copyright (C) 2017 Maeve Automation - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import RPi.GPIO as GPIO
import time

## @package donkey_honk
# Library for activating the piezo buzzer on the Donkey Car.


##
# @brief Set up and maintain necessary interface to GPIO piezo buzzer.
class Buzzer:

    ##
    # @brief Construct a Buzzer instance.
    #
    # @param buzzer_pin The GPIO pin that the buzzer is connected to.
    # @param buzzer_duration The total duration that the buzzer should operate.
    # @param buzzer_on_duration The period of time the buzzer should be 'on' while buzzing.
    # @param buzzer_off_duration The period of time the buzzer should be 'off' while buzzing.

    def __init__(
        self,
        buzzer_pin,
     buzzer_duration,
     buzzer_on_duration,
     buzzer_off_duration):
        GPIO.setmode(GPIO.BCM)
        self.buzzer_pin = buzzer_pin
        self.buzzer_duration = buzzer_duration
        self.buzzer_on_duration = buzzer_on_duration
        self.buzzer_off_duration = buzzer_off_duration
        GPIO.setup(self.buzzer_pin, GPIO.IN)
        GPIO.setup(self.buzzer_pin, GPIO.OUT)

    ##
    # @brief Destruct a Buzzer instance. This method performs GPIO cleanup.
    def __del__(self):
        class_name = self.__class__.__name__
        GPIO.cleanup()

    ##
    # @brief Activate the buzzer according to the parameters this object was constructed with.
    def buzz(self):
        current_time = time.time()
        while (time.time() - current_time) < self.buzzer_duration:
            GPIO.output(self.buzzer_pin, True)
            time.sleep(self.buzzer_on_duration)
            GPIO.output(self.buzzer_pin, False)
            time.sleep(self.buzzer_off_duration)
