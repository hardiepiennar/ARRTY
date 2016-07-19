'''GPIO pin library'''
import RPi.GPIO as GPIO
import time
import numpy as np

'''Pin connections'''
PWR_EN = 2
CSN = 3
SDATA = 4
SCLK = 17
RESETN = 27
PWR_DWN = 22


class ADCCon():
    def __init__(self):
        '''Setup pin configuration'''
        self.setupPins()

        self.power(False)
        time.sleep(0.001)

        print("#####Initiating ADC#####")
        print("Applying power to ADC")
        self.power(True)

        print("Resetting ADC")
        self.reset()
        print("Setting the ADC into power down")
        self.powerDown(True)

        print("Configuring ADC"),
        # Modes of operation and clock divide factor
        reg31 = 0b0000000000000000;
        reg31 |= (1 << 0)  # Set single channel mode
        reg31 |= (0 << 8)  # Set clock div to 1
        self.writeReg(0x31, reg31)

        # Input Select
        reg3A = 0b0000000000000000;
        reg3B = 0b0000000000000000;
        # Connect all 4 ADC's to a single port 1
        reg3A |= (1 << 1)
        reg3A |= (1 << 9)
        reg3B |= (1 << 1)
        reg3B |= (1 << 9)
        self.writeReg(0x3A, reg3A)
        self.writeReg(0x3B, reg3B)

        # Full-scale control (Default)
        # Current control (Default)
        # External Common Mode Voltage Buffer Driving Strength (Default)
        reg50 = 0b0000000000000000;
        reg50 |= (0b10 << 4)
        self.writeReg(0x50, reg50)

        # Start-up and Clock Jitter Control
        reg56 = 0b0000000000000000;
        reg30 = 0b0000000000000000;
        reg56 |= (0b011 << 0)  # Set start up time to 13 - 17.3us
        reg30 |= (1 << 0)  # Clock jitter at 160 fsrms (Lowest power)
        self.writeReg(0x56, reg56)
        self.writeReg(0x30, reg30)

        # LVDS Output Configuration and Control
        # Low clock frequency, andvance and delay all inactive
        # LCLK phase shifted 90 degrees
        # LSB first
        # Straight offset binary

        # LVDS Drive Strength
        # 3.5mA drive (Standard)

        # LVDS Internal Termination Programmability
        # Disabled

        # Power mode control
        # Inactive

        # Programmable gain
        # 1x gain
        # Branch gains - 0dB

        # Analog input invert
        # Inactive

        # LVDS Test Patterns
        self.setFramingPattern(True)

        time.sleep(0.001)
        print("Setting the ADC into power up mode")
        self.powerDown(False)

        print("[DONE]")

    def setGain(self, gain=0b1101):
        '''Set the programmable gain of the adc in dB 0b1101 - 50x'''
        # Enable course gain
        reg33 = 0b0000000000000000;
        reg33 |= (0b1 << 0)
        self.writeReg(0x33, reg33)

        # Set gain for single channel setup
        reg2B = 0b0000000000000000;
        reg2B |= (gain << 8)
        self.writeReg(0x2B, reg2B)

    def setTestValuePattern(self, value):
        '''Turn on a test pattern'''
        reg25 = 0b0000000000000000;
        reg26 = 0b0000000000000000;
        # Enable single code
        reg25 |= (0b001 << 4)
        reg26 |= (value << 8)
        self.writeReg(0x25, reg25)
        self.writeReg(0x26, reg26)

    def setFramingPattern(self, enabled):
        '''Turn on a framing pattern'''
        reg25 = 0b0000000000000000;
        reg26 = 0b0000000000000000;
        # Enable single code
        if enabled:
            reg25 |= (0b001 << 4)
        else:
            reg25 |= (0b000 << 4)
        reg26 |= (0b00000001 << 8)
        self.writeReg(0x25, reg25)
        self.writeReg(0x26, reg26)

    def setRampPattern(self, enabled):
        '''Turn on a ramping pattern'''
        reg25 = 0b0000000000000000;
        # Enable single code
        if enabled:
            reg25 |= (0b100 << 4)
        else:
            reg25 |= (0b000 << 4)
        self.writeReg(0x25, reg25)

    def setupPins(self):
        '''Configures the behavior of the GPIO pins'''
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)  # Sets numbering scheme
        GPIO.setup(PWR_EN, GPIO.OUT)
        GPIO.setup(RESETN, GPIO.OUT)
        GPIO.setup(PWR_DWN, GPIO.OUT)
        GPIO.setup(CSN, GPIO.OUT)
        GPIO.setup(SDATA, GPIO.OUT)
        GPIO.setup(SCLK, GPIO.OUT)

        '''Set default states'''
        GPIO.output(PWR_DWN, GPIO.HIGH)
        GPIO.output(PWR_DWN, GPIO.HIGH)
        GPIO.output(RESETN, GPIO.HIGH)
        GPIO.output(PWR_EN, GPIO.LOW)

    def power(self, powerOn=True):
        '''Turns on ARRTY regulators to ADC module'''
        if powerOn:
            GPIO.output(PWR_EN, GPIO.HIGH)
        else:
            GPIO.output(PWR_EN, GPIO.LOW)

    def reset(self):
        '''Applies a reset signal to the module'''
        GPIO.output(RESETN, GPIO.LOW)
        time.sleep(0.001)
        GPIO.output(RESETN, GPIO.HIGH)

    def powerDown(self, state=True):
        '''Sets the module into power down mode'''
        if state:
            GPIO.output(PWR_DWN, GPIO.HIGH)
        else:
            GPIO.output(PWR_DWN, GPIO.LOW)

    def writeReg(self, addr, data):
        '''Writes a register with the specified data at an given address'''

        '''Bring Clock low'''
        GPIO.output(SCLK, GPIO.LOW)
        '''Bring CSN low to initiate write'''
        GPIO.output(CSN, GPIO.LOW)

        '''Clock in 8 bits of address data'''
        for i in np.arange(8):
            '''Set bit'''
            if ((1 << (7 - i)) & addr) > 0:
                GPIO.output(SDATA, GPIO.HIGH)
            else:
                GPIO.output(SDATA, GPIO.LOW)
            '''Clock bit'''
            GPIO.output(SCLK, GPIO.HIGH)
            GPIO.output(SCLK, GPIO.LOW)

        '''Clock in 16 bits of payload data'''
        for i in np.arange(16):
            '''Set bit'''
            if ((1 << (15 - i)) & data) > 0:
                GPIO.output(SDATA, GPIO.HIGH)
            else:
                GPIO.output(SDATA, GPIO.LOW)
            '''Clock bit'''
            GPIO.output(SCLK, GPIO.HIGH)
            GPIO.output(SCLK, GPIO.LOW)

        '''Finish write by bringing CSN high again'''
        GPIO.output(CSN, GPIO.HIGH)
