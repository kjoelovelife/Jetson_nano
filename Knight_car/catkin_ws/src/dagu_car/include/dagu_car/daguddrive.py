#!/usr/bin/python

# Wrapping the Adafruit API to talk to DC motors with a simpler interface
#
# date:    11/17/2015
#
# authors: Valerio Varricchio <valerio@mit.edu>
#          Luca Carlone <lcarlone@mit.edu>
#          Dmitry Yershov <dmitry.s.yershov@gmail.com>
#

from Adafruit_MotorHAT import Adafruit_MotorHAT
from math import fabs, floor

class DAGU_Differential_Drive:

    LEFT_MOTOR_MIN_PWM = 60        # Minimum speed for left motor  
    LEFT_MOTOR_MAX_PWM = 255       # Maximum speed for left motor  

    RIGHT_MOTOR_MIN_PWM = 60       # Minimum speed for right motor  
    RIGHT_MOTOR_MAX_PWM = 255      # Maximum speed for right motor  

    AXEL_TO_RADIUS_RATIO = 1.0     # The axel length and turning radius ratio
    SPEED_TOLERANCE = 1.e-2;       # speed tolerance level

    def __init__(self, verbose=False, debug=False, left_flip=False, right_flip=False, car_like_mode=True):
        self.car_like_mode = car_like_mode
        self.motorhat = Adafruit_MotorHAT(addr=0x60)
        self.leftMotor = self.motorhat.getMotor(1)
        self.rightMotor = self.motorhat.getMotor(2)


        self.verbose = verbose or debug
        self.debug = debug
        
        self.left_sgn = 1.0;
        if left_flip:
            self.left_sgn = -1.0;

        self.right_sgn = 1.0;
        if right_flip:
            self.right_sgn = -1.0;

        self.speed = 0.0
        self.angle = 0.0

        self.leftSpeed = 0.0
        self.rightSpeed = 0.0

        self.vmax = 1.0 / (1.0 + 0.5 * self.AXEL_TO_RADIUS_RATIO)

        self.updatePWM()

    def PWMvalue(self, v, minPWM, maxPWM):
        pwm = 0;
        if fabs(v) > self.SPEED_TOLERANCE:
            pwm = int(floor(fabs(v) * (maxPWM - minPWM) + minPWM))
        return pwm;

    def updatePWM(self):

        v = self.speed * self.vmax;
        u = self.angle

        if self.car_like_mode:
            vl = v * (1.0 - u * 0.5 * self.AXEL_TO_RADIUS_RATIO) * self.left_sgn
            vr = v * (1.0 + u * 0.5 * self.AXEL_TO_RADIUS_RATIO) * self.right_sgn
        else:
            vl = v + self.vmax * ( - u * 0.5 * self.AXEL_TO_RADIUS_RATIO) * self.left_sgn
            vr = v + self.vmax * ( + u * 0.5 * self.AXEL_TO_RADIUS_RATIO) * self.right_sgn

        pwml = self.PWMvalue(vl, self.LEFT_MOTOR_MIN_PWM, self.LEFT_MOTOR_MAX_PWM)
        pwmr = self.PWMvalue(vr, self.RIGHT_MOTOR_MIN_PWM, self.RIGHT_MOTOR_MAX_PWM)

        if self.debug:
            print "v = %5.3f, u = %5.3f, vl = %5.3f, vr = %5.3f, pwml = %3d, pwmr = %3d" % (v, u, vl, vr, pwml, pwmr)

        if fabs(vl) < self.SPEED_TOLERANCE:
            leftMotorMode = Adafruit_MotorHAT.RELEASE
        elif vl > 0:
            leftMotorMode = Adafruit_MotorHAT.FORWARD
        elif vl < 0: 
            leftMotorMode = Adafruit_MotorHAT.BACKWARD

        if fabs(vr) < self.SPEED_TOLERANCE:
            rightMotorMode = Adafruit_MotorHAT.RELEASE
            pwmr = 0;
        elif vr > 0:
            rightMotorMode = Adafruit_MotorHAT.FORWARD
        elif vr < 0: 
            rightMotorMode = Adafruit_MotorHAT.BACKWARD

        self.leftMotor.setSpeed(pwml)
        self.leftMotor.run(leftMotorMode);
        self.rightMotor.setSpeed(pwmr)
        self.rightMotor.run(rightMotorMode);

    # Speed has to be in [-1, 1], sign determines bwd/fwd
    def setSpeed(self, speed):
        # Warning the user
        if fabs(speed)>1:
            raise ValueError('Speed must be between -1 and 1')

        self.speed = speed

        if self.verbose:
            print "Speed is set to %5.3f" % self.speed

        self.updatePWM()
        


    def setSteerAngle(self, angle): 

        if fabs(angle)>1:
            raise ValueError('Steer angle must be between -1 and 1')

        self.angle = angle

        if self.verbose:
            print "Steer angle is set to %5.3f" % self.angle

        self.updatePWM()


    def __del__(self):
        self.leftMotor.run(Adafruit_MotorHAT.RELEASE)
        self.rightMotor.run(Adafruit_MotorHAT.RELEASE)
        del self.motorhat

# Simple example to test motors
if __name__ == '__main__':

    from time import sleep

    N = 10;
    delay = 100. / 1000.

    dagu = DAGU_Differential_Drive()

    # turn left
    dagu.setSteerAngle(1.0)
    # accelerate forward
    for i in range(N):
        dagu.setSpeed((1.0 + i) / N)
        sleep(delay)
    # decelerate forward
    for i in range(N):
        dagu.setSpeed((-1.0 - i + N) / N)
        sleep(delay)

    # turn right
    dagu.setSteerAngle(-1.0)
    # accelerate backward
    for i in range(N):
        dagu.setSpeed(-(1.0 + i) / N)
        sleep(delay)
    # decelerate backward
    for i in range(N):
        dagu.setSpeed(-(-1.0 - i + N) / N)
        sleep(delay)

    # turn left
    dagu.setSteerAngle(1.0)
    # accelerate forward
    for i in range(N):
        dagu.setSpeed((1.0 + i) / N)
        sleep(delay)
    # decelerate forward
    for i in range(N):
        dagu.setSpeed((-1.0 - i + N) / N)
        sleep(delay)

    del dagu
