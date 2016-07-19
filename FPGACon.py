'''GPIO pin library'''
import RPi.GPIO as GPIO
import time
import numpy as np

'''Pin connections'''
CLK = 8
DIN = 25
DOUT = 11
SEN = 9

buffer = 200  # Adds a buffer to ensure the no of samples
overread = 300


class FPGACon():
    def __init__(self):
        '''Set up the pins to the digital board'''
        self.setupPins()
        time.sleep(0.1)
        GPIO.output(SEN, GPIO.HIGH)
        time.sleep(0.1)
        GPIO.output(SEN, GPIO.LOW)
        time.sleep(0.1)

        self.sampleLength = 1024

    def setupPins(self):
        '''Sets up the GPIO pins'''
        '''Configure the behaviour of the GPIO pins'''
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(CLK, GPIO.OUT)
        GPIO.setup(DIN, GPIO.OUT)
        GPIO.setup(DOUT, GPIO.IN)
        GPIO.setup(SEN, GPIO.OUT)

        '''Set default states'''
        GPIO.output(CLK, GPIO.LOW)
        GPIO.output(DIN, GPIO.LOW)
        GPIO.output(SEN, GPIO.LOW)

    def readData(self, customLength=-1):
        '''Reads all of the data from the fifo'''
        if not customLength == -1:
            length = customLength
        else:
            length = self.sampleLength

        data = []
        GPIO.output(DIN, GPIO.LOW)
        GPIO.output(SEN, GPIO.HIGH)
        for x in np.arange(length + buffer + overread):
            byte = 0
            for i in np.arange(8):
                self.pulseCLK()
                if(GPIO.input(DOUT) == 1):
                    byte = byte|(0b1<<i)
            if x < length + buffer/2 and x >= buffer/2:
                data.append(byte - 128)
        GPIO.output(SEN, GPIO.LOW)
        return data

    def setSampleLength(self, length):
        '''Sets the no of samples to store in the fifo buffer
        max = 65000'''
        '''Send set sample length command'''
        self.sendCommand(0b00000000000010)
        '''Send the sample length'''
        self.sendCommand(length + buffer)
        self.sampleLength = length

    def setTriggerLevel(self, level):
        '''Sets the trigger level'''
        '''Send set trigger level command'''
        self.sendCommand(0b00000000000001)
        '''Send the trigger level'''
        self.sendCommand(level)

    def setTrigger(self):
        '''Sets the device in trigger mode'''
        '''Send set trigger command'''
        self.sendCommand(0b00000000000011)

    def readDOUT(self):
        '''Returns the level of dout'''
        return GPIO.input(DOUT)

    def sendCommand(self, command):
        '''Send a 16 bit command'''
        GPIO.output(DIN, GPIO.LOW)
        GPIO.output(CLK, GPIO.LOW)
        GPIO.output(SEN, GPIO.HIGH)
        for i in np.arange(16):
            if command&(0b1<<i) > 0:
                GPIO.output(DIN, GPIO.HIGH)
            else:
                GPIO.output(DIN, GPIO.LOW)
            self.pulseCLK()
        GPIO.output(SEN, GPIO.LOW)
        GPIO.output(DIN, GPIO.LOW)
        GPIO.output(CLK, GPIO.LOW)

    def pulseCLK(self):
        '''Pulse the clock pin'''
        GPIO.output(CLK, GPIO.LOW)
        GPIO.output(CLK, GPIO.HIGH)

