# Wrapping the Adafruit API to talk to DC motors with a simpler interface
#
# date:    11/17/2015
#
# authors: Valerio Varricchio <valerio@mit.edu>
#          Luca Carlone <lcarlone@mit.edu>
#

# ~~~~~ IMPORTANT !!! ~~~~~
#
# Make sure that the front motor is connected in such a way that a positive
# speed  causes an increase in the potentiometer reading!
from Adafruit_MotorHAT import Adafruit_MotorHAT, Adafruit_DCMotor
import time
import atexit
import numpy
import warnings
from threading import Thread
from dagu_car import pot
import math

class Controller:
    def __init__(self):

        # First eye-metric tuning. At least the signs are ok!
        self.P = 1.2
        self.I = 4
        self.D = 50
        self.reference = 0
        self.integral = 0
        self.oldValue = 0
        self.dt = 0.02 # the higher this frequency the more often the potentiometer fails
        self.numIt = 0
        self.totalTime = 0

class DAGU_Car:

    __driveDeadZone = 10       # Minimum speed to win static friction  
    __driveMaxSpeed = 255      # Maximum speed that can be commanded

    __steerDeadZone = 10       # Minimum steer DC motor speed to win static friction  
    __steerMaxSpeed = 255
    __steerTolerance = 0.1     # Positioning threshold below which we send release to the DC motor

    def __init__(self, verbose=False):

        self.mh = Adafruit_MotorHAT(addr=0x60)
        self.rearMotor = self.mh.getMotor(1)
        self.frontMotor = self.mh.getMotor(2)
        self.rearMotorMode = Adafruit_MotorHAT.RELEASE

        # create Potentiometer object
        self.potentiometer = pot.Pot()
        self.referenceSteerAngle = 0
        self.verbose = verbose
        # 
        self.steerController = Controller()

        # Separate thred
        self.steerThread = Thread(target = self.steerServo)
        self.steerThreadStop = False;
        self.steerThread.start()


    # Speed has to be in [-1, 1], sign determines bwd/fwd
    def setSpeed(self, speed):
        # Warning the user
        if abs(speed)>1:
            warnings.warn("Input speed has to be in [-1, 1]. Clamping it.")

        # Clamping speed value to [-1, 1]
        speed = numpy.clip(speed, -1, 1)

        if abs(speed) < 1e-4:
            self.rearMotorMode = Adafruit_MotorHAT.RELEASE
        elif speed > 0:
            self.rearMotorMode = Adafruit_MotorHAT.FORWARD
        elif speed < 0: 
            self.rearMotorMode = Adafruit_MotorHAT.BACKWARD


        self.rearMotor.setSpeed(int(round(\
            abs(speed)*(DAGU_Car.__driveMaxSpeed-DAGU_Car.__driveDeadZone)\
            +DAGU_Car.__driveDeadZone)));
        
        self.rearMotor.run(self.rearMotorMode);

    def steerServo(self):
        while not self.steerThreadStop:
            #print("Separate Servo thread running")
            self.steerController.numIt += 1
            tin = time.time()
            
            reading = self.potentiometer.getPot()
            
            if(math.isnan(reading)):
                # If the sensor turns out "not attached" (PhidgetException downstream)
                # then...wait for a while. Apparently if I keep issuing requests...
                # it doesn't manage to reconnect again.
                if(self.verbose):
                    print("potentiometer not ready, waiting...")
                
                time.sleep(5*self.steerController.dt)
                continue

            delta = self.steerController.reference-reading
            steerMotorMode = Adafruit_MotorHAT.RELEASE

            derivative = (delta-self.steerController.oldValue)/self.steerController.dt

            # worst case Delta is pm 2
            cmd = self.steerController.P*delta
            cmd += self.steerController.I*self.steerController.integral
            cmd *= DAGU_Car.__steerMaxSpeed
            if(self.verbose):
                print("x: "+str(delta)+" xdot: "+str(derivative)+" int: "+str(self.steerController.integral))
                print("rawcmd: "+str(int(cmd)))

            if(abs(delta)>DAGU_Car.__steerTolerance):
                if(cmd>0):
                    steerMotorMode = Adafruit_MotorHAT.BACKWARD
                else:
                    steerMotorMode = Adafruit_MotorHAT.FORWARD
                cmd = abs(cmd)
                cmd += self.steerController.D*abs(derivative)
                cmd = numpy.clip(cmd, DAGU_Car.__steerDeadZone, DAGU_Car.__steerMaxSpeed)
            else:
                self.steerController.integral = 0 #resetting integral term
            
            if(self.verbose):
                #print("x: "+str(delta)+" xdot: "+str(derivative)+" int: "+str(self.steerController.integral))
                print("cmd: "+str(int(cmd)))

            self.frontMotor.setSpeed(int(cmd))
            self.frontMotor.run(steerMotorMode)

            self.steerController.oldValue = delta
            self.steerController.integral += delta*self.steerController.dt
            time.sleep(self.steerController.dt)
            self.steerController.totalTime += time.time()-tin;

    def stopSteerControl(self):
        print("Trying to stop the steer control thread and release motor.")
        self.steerThreadStop = True
        #self.steerThread.join()
        self.frontMotor.run(Adafruit_MotorHAT.RELEASE)
        
    def startSteerControl(self):
        self.steerThreadStop = False;
        self.steerThread.start()
        

    def setSteerAngle(self, angle, P=float('nan'), D=float('nan'), I=float('nan')): 
    # TODO this has to be implemented (maybe using a separate control thread)
    # once the potentiometer data is available
        if not math.isnan(P):
            self.steerController.P = P
        
        if not math.isnan(P):
            self.steerController.D = D
        
        if not math.isnan(P):
            self.steerController.I = I

        if abs(angle)>1:
            warnings.warn("Input angle has to be in [-1, 1]. Clamping it.")

        self.steerController.integral = 0 #resetting integral term
        self.steerController.reference = numpy.clip(angle, -1, 1);

    def printHz(self):
        print(self.steerController.numIt/self.steerController.totalTime)

    # recommended for auto-disabling motors on shutdown!
    def turnOffMotors(self):
        self.rearMotor.run(Adafruit_MotorHAT.RELEASE)
        self.frontMotor.run(Adafruit_MotorHAT.RELEASE)
        self.setSteerAngle(0)
        
    def __del__(self):

        self.stopSteerControl()
        del self.potentiometer
        del self.mh
        self.turnOffMotors()
