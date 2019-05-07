#Basic imports
from ctypes import *
#Phidget specific imports
from Phidgets.PhidgetException import PhidgetErrorCodes, PhidgetException
from Phidgets.Events.Events import AttachEventArgs, DetachEventArgs, ErrorEventArgs, InputChangeEventArgs, OutputChangeEventArgs, SensorChangeEventArgs
from Phidgets.Devices.InterfaceKit import InterfaceKit
from Phidgets.Phidget import PhidgetLogLevel

class Pot:

    __sensorPort = 1
    __sampleRate = 4 # Maybe too high?

    __potMin = 242
    __potZero = 490
    __potMax = 668


    def __init__(self):
        #Create an interfacekit object
        try:
            self.interfaceKit = InterfaceKit()
        except RuntimeError as e:
            print("Runtime Exception: %s" % e.details)
            print("Exiting....")
            exit(1)

        try:
            self.interfaceKit.openPhidget()
            self.interfaceKit.waitForAttach(10000)
            self.interfaceKit.setDataRate(self.__sensorPort, self.__sampleRate)
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            try:
                self.interfaceKit.closePhidget()
            except PhidgetException as e:
                print("Phidget Exception %i: %s" % (e.code, e.details))
                print("Exiting....")
                exit(1)


    def __del__(self):
        try:
            self.interfaceKit.closePhidget()
        except PhidgetException as e:
            print("Phidget Exception %i: %s" % (e.code, e.details))
            print("Exiting....")
            exit(1)

    def getRawPot(self):
        try:
            resp = self.interfaceKit.getSensorValue(self.__sensorPort)
        except PhidgetException as e:
            # suppress print (makes testing harder) #TODO restore!!
            resp = float('nan')
            #print("Skipping reading - phidget Exception %i: %s" % (e.code, e.details))
        return resp

    #def recconect(self):


    def getPot(self):
        rawSensor = self.getRawPot()
        value = 0.0

        if rawSensor > self.__potMax:
            #warnings.warn("Pot measuremens out of bounds.") 
            return 1.0
        if rawSensor < self.__potMin: 
            #warnings.warn("Pot measuremens out of bounds.")
            return -1.0

        if rawSensor >= self.__potZero:
            value = float(rawSensor - self.__potZero) / (self.__potMax - self.__potZero)
        else:
            value = float(rawSensor - self.__potZero) / (self.__potZero - self.__potMin)

        return value

    
	
