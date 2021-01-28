#!/usr/bin/python

# Python interface driver for L3GD20 gyroscope, inspired by
# Adafruit_LSM303 drivers.
#
# date:    12/11/2015
#
# author: Dmitry Yershov <dmitry.s.yershov@gmail.com>
#

from Adafruit_I2C import Adafruit_I2C


class Gyro_L3GD20(Adafruit_I2C):

    # Minimal constants carried over from Arduino library
    L3GD20_ADDRESS = (0xD6 >> 1)             # 1101011x

    # L3GD20 registers
                                             # Default    Type
    L3GD20_REGISTER_WHO_AM_I       = 0x0F    # 11010100   r
    L3GD20_REGISTER_CTRL_REG1      = 0x20    # 00000111   rw
    L3GD20_REGISTER_CTRL_REG2      = 0x21    # 00000000   rw
    L3GD20_REGISTER_CTRL_REG3      = 0x22    # 00000000   rw
    L3GD20_REGISTER_CTRL_REG4      = 0x23    # 00000000   rw
    L3GD20_REGISTER_CTRL_REG5      = 0x24    # 00000000   rw
    L3GD20_REGISTER_REFERENCE      = 0x25    # 00000000   r
    L3GD20_REGISTER_OUT_TEMP       = 0x26    #            r
    L3GD20_REGISTER_STATUS_REG     = 0x27    #            r
    L3GD20_REGISTER_OUT_X_L        = 0x28    #            r
    L3GD20_REGISTER_OUT_X_H        = 0x29    #            r
    L3GD20_REGISTER_OUT_Y_L        = 0x2A    #            r
    L3GD20_REGISTER_OUT_Y_H        = 0x2B    #            r
    L3GD20_REGISTER_OUT_Z_L        = 0x2C    #            r
    L3GD20_REGISTER_OUT_Z_H        = 0x2D    #            r
    L3GD20_REGISTER_FIFO_CTRL_REG  = 0x2E    # 00000000   rw
    L3GD20_REGISTER_FIFO_SRC_REG   = 0x2F    #            r
    L3GD20_REGISTER_INT1_CFG       = 0x30    # 00000000   rw
    L3GD20_REGISTER_INT1_SRC       = 0x31    #            r
    L3GD20_REGISTER_TSH_XH         = 0x32    # 00000000   rw
    L3GD20_REGISTER_TSH_XL         = 0x33    # 00000000   rw
    L3GD20_REGISTER_TSH_YH         = 0x34    # 00000000   rw
    L3GD20_REGISTER_TSH_YL         = 0x35    # 00000000   rw
    L3GD20_REGISTER_TSH_ZH         = 0x36    # 00000000   rw
    L3GD20_REGISTER_TSH_ZL         = 0x37    # 00000000   rw
    L3GD20_REGISTER_INT1_DURATION  = 0x38    # 00000000   rw


    # Chipsed IDs
    L3GD20_ID  = 0xD4
    L3GD20H_ID = 0xD7

    # Range settings setRange()
    L3GD20_RANGE_250DPS  = 250
    L3GD20_RANGE_500DPS  = 500
    L3GD20_RANGE_2000DPS = 2000

    # Range sensitivity getSens()
    L3GD20_SENSITIVITY_250DPS  = 0.00875
    L3GD20_SENSITIVITY_500DPS  = 0.0175
    L3GD20_SENSITIVITY_2000DPS = 0.070

    def __init__(self, gyro_range=250, busnum=-1, debug=False, hires=False):

        # Accelerometer and magnetometer are at different I2C
        # addresses, so invoke a separate I2C instance for each
        self.gyro = Adafruit_I2C(self.L3GD20_ADDRESS, busnum, debug)

        chipset_id = self.gyro.readU8(self.L3GD20_REGISTER_WHO_AM_I)

        if chipset_id != self.L3GD20_ID and chipset_id != self.L3GD20H_ID:
            raise Warning("L3GD20 error: wrong id %4x received at address  %4x" % (chipset_id, self.L3GD20_ADDRESS))

        # Enable all three axis on the gyro 
        self.gyro.write8(self.L3GD20_REGISTER_CTRL_REG1, 0x00)
        self.gyro.write8(self.L3GD20_REGISTER_CTRL_REG1, 0x0F)

        self.gyro_range = self.L3GD20_SENSITIVITY_250DPS
        self.setRange(gyro_range)

    # Interpret signed 16-bit angular rotation from list
    def gyro16(self, values_uint8, idx):
        value_int16 = values_uint8[idx] | (values_uint8[idx+1] << 8)
        if value_int16 > 32767: value_int16 -= 65536
        return value_int16

    def getSens(self, value_int16):
        value_f = 0.0
        if self.gyro_range == self.L3GD20_RANGE_250DPS:
            value_f = value_int16 * self.L3GD20_SENSITIVITY_250DPS
        elif self.gyro_range == self.L3GD20_RANGE_500DPS:
            value_f = value_int16 * self.L3GD20_SENSITIVITY_500DPS
        elif self.gyro_range == self.L3GD20_RANGE_2000DPS:
            value_f = value_int16 * self.L3GD20_SENSITIVITY_2000DPS
        return value_f
    
    def read(self):
        # Read the gyro
        values_uint8 = self.gyro.readList(self.L3GD20_REGISTER_OUT_X_L | 0x80, 6)
        # Convert readings
        result = [( self.getSens(self.gyro16(values_uint8, 0)),
                    self.getSens(self.gyro16(values_uint8, 2)),
                    self.getSens(self.gyro16(values_uint8, 4)) )]
        return result


    def setRange(self, gyro_range):
        if gyro_range == self.gyro_range:
            return;
        elif gyro_range == self.L3GD20_RANGE_250DPS:
            self.gyro.write8(self.L3GD20_REGISTER_CTRL_REG4, 0x00)
        elif gyro_range == self.L3GD20_RANGE_500DPS:
            self.gyro.write8(self.L3GD20_REGISTER_CTRL_REG4, 0x10)
        elif gyro_range == self.L3GD20_RANGE_2000DPS:
            self.gyro.write8(self.L3GD20_REGISTER_CTRL_REG4, 0x20)
        else:
            raise ValueError("L3GD20 range value can only be 250, 500, or 2000")
            
        self.gyro_range = gyro_range;


# Simple example prints accel/mag data once per second:
if __name__ == '__main__':

    from time import sleep

    gyro = Gyro_L3GD20()

    print '[(Gyro w_X, w_Y, w_Z)]'
    while True:
        print gyro.read()
        sleep(1) # Output is fun to watch if this is commented out
