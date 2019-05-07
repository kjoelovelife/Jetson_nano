#!/usr/bin/python

# Wrapping the Adafruit API to talk to DC motors with a simpler interface
#
# date:    11/17/2015
#
# authors: Valerio Varricchio <valerio@mit.edu>
#          Luca Carlone <lcarlone@mit.edu>
#          Dmitry Yershov <dmitry.s.yershov@gmail.com>
#


# from math import fabs, floor

class RGB_LED():
#
#     LED_REAR_LEFT   = 0
#     LED_REAR_RIGH   = 3
#     LED_TOP         = 6
#     LED_FRONT_LEFT  = 9
#     LED_FRONT_RIGHT = 12

    OFFSET_RED   = 0
    OFFSET_GREEN = 1
    OFFSET_BLUE  = 2

    def __init__(self, debug=False):
        from Adafruit_PWM_Servo_Driver import PWM  # @UnresolvedImport
        self.pwm = PWM(address=0x40, debug=debug)
        for i in range(15):
            self.pwm.setPWM(i, 0, 4095)

    def setLEDBrightness(self, led, offset, brightness):
        self.pwm.setPWM(3 * led + offset, brightness << 4, 4095)

    def setRGBint24(self, led, color):
        r = color >> 16 & 0xFF
        g = color >> 8 & 0xFF
        b = color >> 0 & 0xFF
        self.setRGBvint8(led, [r, g, b])
        
    def setRGBvint8(self, led, color):
        self.setLEDBrightness(led, self.OFFSET_RED  , color[0])
        self.setLEDBrightness(led, self.OFFSET_GREEN, color[1])
        self.setLEDBrightness(led, self.OFFSET_BLUE , color[2])

    def setRGB(self, led, color):
        self.setRGBvint8(led, map(lambda f: int(f * 255), color))

    def __del__(self):
        for i in range(15):
            self.pwm.setPWM(i, 0, 4095)
        del self.pwm
