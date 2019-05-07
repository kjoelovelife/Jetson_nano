#!/usr/bin/python

# Python library for MB1242 sonar

from Adafruit_I2C import Adafruit_I2C


class Sonar_MB1242(Adafruit_I2C):

    # Minimal constants carried over from Arduino library
    MB1242_ADDRESS = 0x70            # 1110000x
                                     # Default    Type
    MB1242_REGISTER_RANGE_OUT = 0x51 # 01010001   rw


    def __init__(self, busnum=-1, debug=False):
        # initiate sonar i2c interface
        self.sonar = Adafruit_I2C(self.MB1242_ADDRESS, busnum, debug)

    def read(self):
        # Read the accelerometer
        sonar_range = self.sonar.readList(self.MB1242_REGISTER_RANGE_OUT, 2)
        return sonar_range


# Simple example prints accel/mag data once per second:
if __name__ == '__main__':

    from time import sleep

    sonar = Sonar_MB1242()

    print 'Range'
    while True:
        print sonar.read()
        sleep(1) # Output is fun to watch if this is commented out
