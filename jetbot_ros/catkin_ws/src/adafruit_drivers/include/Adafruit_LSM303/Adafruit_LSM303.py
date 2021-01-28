#!/usr/bin/python

# Python library for Adafruit Flora Accelerometer/Compass Sensor (LSM303).
# This is pretty much a direct port of the current Arduino library and is
# similarly incomplete (e.g. no orientation value returned from read()
# method).  This does add optional high resolution mode to accelerometer
# though.

# Copyright 2013 Adafruit Industries

# Permission is hereby granted, free of charge, to any person obtaining a
# copy of this software and associated documentation files (the "Software"),
# to deal in the Software without restriction, including without limitation
# the rights to use, copy, modify, merge, publish, distribute, sublicense,
# and/or sell copies of the Software, and to permit persons to whom the
# Software is furnished to do so, subject to the following conditions:

# The above copyright notice and this permission notice shall be included
# in all copies or substantial portions of the Software.

# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
# THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
# FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
# DEALINGS IN THE SOFTWARE.

from Adafruit_I2C import Adafruit_I2C


class Adafruit_LSM303(Adafruit_I2C):

    # Minimal constants carried over from Arduino library
    LSM303_ADDRESS_ACCEL = (0x32 >> 1)  # 0011001x
    LSM303_ADDRESS_MAG   = (0x3C >> 1)  # 0011110x
                                             # Default    Type
    LSM303_REGISTER_ACCEL_CTRL_REG1_A = 0x20 # 00000111   rw
    LSM303_REGISTER_ACCEL_CTRL_REG4_A = 0x23 # 00000000   rw
    LSM303_REGISTER_ACCEL_OUT_X_L_A   = 0x28
    LSM303_REGISTER_MAG_CRB_REG_M     = 0x01
    LSM303_REGISTER_MAG_MR_REG_M      = 0x02
    LSM303_REGISTER_MAG_OUT_X_H_M     = 0x03

    # Gain settings for setMagGain()
    LSM303_MAG_GAIN_1_3 = 0x20 # +/- 1.3
    LSM303_MAG_GAIN_1_9 = 0x40 # +/- 1.9
    LSM303_MAG_GAIN_2_5 = 0x60 # +/- 2.5
    LSM303_MAG_GAIN_4_0 = 0x80 # +/- 4.0
    LSM303_MAG_GAIN_4_7 = 0xA0 # +/- 4.7
    LSM303_MAG_GAIN_5_6 = 0xC0 # +/- 5.6
    LSM303_MAG_GAIN_8_1 = 0xE0 # +/- 8.1

    # Sensitivity settings for getAccelSens()
    LSM303_MAG_XY_SENSITIVITY_1_3 = 0.000909091 # 1100 LSB/Gauss
    LSM303_MAG_XY_SENSITIVITY_1_9 = 0.001169591 # 855 LSB/Gauss
    LSM303_MAG_XY_SENSITIVITY_2_5 = 0.001492537 # 670 LSB/Gauss
    LSM303_MAG_XY_SENSITIVITY_4_0 = 0.002222222 # 450 LSB/Gauss
    LSM303_MAG_XY_SENSITIVITY_4_7 = 0.002500000 # 400 LSB/Gauss
    LSM303_MAG_XY_SENSITIVITY_5_6 = 0.003030303 # 330 LSB/Gauss
    LSM303_MAG_XY_SENSITIVITY_8_1 = 0.004347826 # 230 LSB/Gauss
    # Z gain is different from X and Y gain
    LSM303_MAG_Z_SENSITIVITY_1_3 = 0.001020408 # 980 LSB/Gauss
    LSM303_MAG_Z_SENSITIVITY_1_9 = 0.001315789 # 760 LSB/Gauss
    LSM303_MAG_Z_SENSITIVITY_2_5 = 0.001666667 # 600 LSB/Gauss
    LSM303_MAG_Z_SENSITIVITY_4_0 = 0.002500000 # 400 LSB/Gauss
    LSM303_MAG_Z_SENSITIVITY_4_7 = 0.002816901 # 355 LSB/Gauss
    LSM303_MAG_Z_SENSITIVITY_5_6 = 0.003389831 # 295 LSB/Gauss
    LSM303_MAG_Z_SENSITIVITY_8_1 = 0.004878049 # 205 LSB/Gauss

    # Gain settings setAccelGain()
    LSM303_ACCEL_GAIN_2  = 0x00 # +/- 2  g
    LSM303_ACCEL_GAIN_4  = 0x10 # +/- 4  g
    LSM303_ACCEL_GAIN_8  = 0x20 # +/- 8  g
    LSM303_ACCEL_GAIN_16 = 0x30 # +/- 16 g

    # Sensitivity settings for getAccelSens()
    LSM303_ACCEL_SENSITIVITY_2  = 0.001 # 1  mg/LSB
    LSM303_ACCEL_SENSITIVITY_4  = 0.002 # 2  mg/LSB
    LSM303_ACCEL_SENSITIVITY_8  = 0.004 # 4  mg/LSB
    LSM303_ACCEL_SENSITIVITY_16 = 0.012 # 12 mg/LSB As in the datasheet, but why?

    def __init__(self, accel_gain=LSM303_ACCEL_GAIN_2,
                 mag_gain=LSM303_MAG_GAIN_1_3,
                 busnum=-1, debug=False, hires=False):

        # Accelerometer and magnetometer are at different I2C
        # addresses, so invoke a separate I2C instance for each
        self.accel = Adafruit_I2C(self.LSM303_ADDRESS_ACCEL, busnum, debug)
        self.mag   = Adafruit_I2C(self.LSM303_ADDRESS_MAG  , busnum, debug)

        # Enable the accelerometer
        self.accel.write8(self.LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27)
        # Select hi-res (12-bit) or low-res (10-bit) output mode.
        # Low-res mode uses less power and sustains a higher update rate,
        # output is padded to compatible 12-bit units.
        self.hires = hires
  
        # Enable the magnetometer
        self.mag.write8(self.LSM303_REGISTER_MAG_MR_REG_M, 0x00)

        self.setAccelGain(accel_gain)

        self.setMagGain(mag_gain)


    # Interpret signed 12-bit acceleration component from list
    def accel12(self, list, idx):
        n = list[idx] | (list[idx+1] << 8) # Low, high bytes
        if n > 32767: n -= 65536           # 2's complement signed
        return n >> 4                      # 12-bit resolution


    # Interpret signed 16-bit magnetometer component from list
    def mag16(self, list, idx):
        n = (list[idx] << 8) | list[idx+1]   # High, low bytes
        return n if n < 32768 else n - 65536 # 2's complement signed

    def getAccelSens(self, value_int):
        value_f = 0.0
        if self.accel_gain == self.LSM303_ACCEL_GAIN_2:
            value_f = value_int * self.LSM303_ACCEL_SENSITIVITY_2
        elif self.accel_gain == self.LSM303_ACCEL_GAIN_4:
            value_f = value_int * self.LSM303_ACCEL_SENSITIVITY_4
        elif self.accel_gain == self.LSM303_ACCEL_GAIN_8:
            value_f = value_int * self.LSM303_ACCEL_SENSITIVITY_8
        elif self.accel_gain == self.LSM303_ACCEL_GAIN_16:
            value_f = value_int * self.LSM303_ACCEL_SENSITIVITY_16
        return value_f

    def getMagXYSens(self, value_int):
        value_f = 0.0
        if self.mag_gain == self.LSM303_MAG_GAIN_1_3:
            value_f = value_int * self.LSM303_MAG_XY_SENSITIVITY_1_3
        elif self.mag_gain == self.LSM303_MAG_GAIN_1_9:
            value_f = value_int * self.LSM303_MAG_XY_SENSITIVITY_1_9
        elif self.mag_gain == self.LSM303_MAG_GAIN_2_5:
            value_f = value_int * self.LSM303_MAG_XY_SENSITIVITY_2_5
        elif self.mag_gain == self.LSM303_MAG_GAIN_4_0:
            value_f = value_int * self.LSM303_MAG_XY_SENSITIVITY_4_0
        elif self.mag_gain == self.LSM303_MAG_GAIN_4_7:
            value_f = value_int * self.LSM303_MAG_XY_SENSITIVITY_4_7
        elif self.mag_gain == self.LSM303_MAG_GAIN_5_6:
            value_f = value_int * self.LSM303_MAG_XY_SENSITIVITY_5_6
        elif self.mag_gain == self.LSM303_MAG_GAIN_8_1:
            value_f = value_int * self.LSM303_MAG_XY_SENSITIVITY_8_1
        return value_f

    def getMagZSens(self, value_int):
        value_f = 0.0
        if self.mag_gain == self.LSM303_MAG_GAIN_1_3:
            value_f = value_int * self.LSM303_MAG_Z_SENSITIVITY_1_3
        elif self.mag_gain == self.LSM303_MAG_GAIN_1_9:
            value_f = value_int * self.LSM303_MAG_Z_SENSITIVITY_1_9
        elif self.mag_gain == self.LSM303_MAG_GAIN_2_5:
            value_f = value_int * self.LSM303_MAG_Z_SENSITIVITY_2_5
        elif self.mag_gain == self.LSM303_MAG_GAIN_4_0:
            value_f = value_int * self.LSM303_MAG_Z_SENSITIVITY_4_0
        elif self.mag_gain == self.LSM303_MAG_GAIN_4_7:
            value_f = value_int * self.LSM303_MAG_Z_SENSITIVITY_4_7
        elif self.mag_gain == self.LSM303_MAG_GAIN_5_6:
            value_f = value_int * self.LSM303_MAG_Z_SENSITIVITY_5_6
        elif self.mag_gain == self.LSM303_MAG_GAIN_8_1:
            value_f = value_int * self.LSM303_MAG_Z_SENSITIVITY_8_1
        return value_f

    def read(self):
        # Read the accelerometer
        list = self.accel.readList(
          self.LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80, 6)
        res = [( self.getAccelSens(self.accel12(list, 0)),
                 self.getAccelSens(self.accel12(list, 2)),
                 self.getAccelSens(self.accel12(list, 4)) )]

        # Read the magnetometer
        list = self.mag.readList(self.LSM303_REGISTER_MAG_OUT_X_H_M, 6)
        res.append((self.getMagXYSens(self.mag16(list, 0)),
                    self.getMagXYSens(self.mag16(list, 2)),
                    self.getMagZSens(self.mag16(list, 4)) ))
        # ToDo: Calculate orientation

        return res


    def setAccelGain(self, accel_gain):
        reg = accel_gain
        if self.hires:
            reg |= 0b00001000
        self.accel.write8(self.LSM303_REGISTER_ACCEL_CTRL_REG4_A, reg)
        self.accel_gain = accel_gain

    def setMagGain(self, mag_gain):
        self.mag.write8(self.LSM303_REGISTER_MAG_CRB_REG_M, mag_gain)
        self.mag_gain = mag_gain


# Simple example prints accel/mag data once per second:
if __name__ == '__main__':

    from time import sleep

    lsm = Adafruit_LSM303()

    print '[(Accelerometer X, Y, Z), (Magnetometer X, Y, Z, orientation)]'
    while True:
        print lsm.read()
        sleep(1) # Output is fun to watch if this is commented out
