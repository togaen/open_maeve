# Copyright (C) 2017 Will Roscoe - All Rights Reserved
# Permission to copy and modify is granted under the MIT license

import Adafruit_PCA9685

## @package adafruit_pca9685
# Interface for Adafruit PCA9685 controller.

class PCA9685_Controller:
    ''' 
    Adafruit PWM controler. 
    This is used for most RC Cars
    '''
    def __init__(self, channel, frequency=60): 
        # Initialise the PCA9685 using the default address (0x40).
        self.pwm = Adafruit_PCA9685.PCA9685()

        self.pwm.set_pwm_freq(frequency)
        self.channel = channel

    def set_pulse(self, pulse):
        self.pwm.set_pwm(self.channel, 0, pulse)

