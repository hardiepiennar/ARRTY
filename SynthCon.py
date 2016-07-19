"""GPIO pin library"""
import RPi.GPIO as GPIO
import time
import numpy as np
from fractions import Fraction

'''Pin connections'''
PWR_EN = 14
CEN = 15
SEN = 18
SDO = 23
SCK = 24
SDI = 10

'''Verbose'''
verbose = True


class SynthCon():
    def __init__(self):
        """Set up the pins to the digital board"""
        self.setupPins()

        self.power(False)
        time.sleep(0.001)

    def setFrequency(self, freq):
        """Sets the registers to the desired frequency"""
        ref = 50e6
        if verbose:
            print("Calculating registers for: " + str(freq) + " Hz")
        '''Calculate NOUT'''
        '''VCO runs from 1.5 GHz up to 3 GHz, NOUT sets the VCO output divider'''
        NOUT = 2 * np.floor(3e9 / (2 * freq))

        '''Calculate the reference divider'''
        '''PFD needs to run below 70 MHz'''
        RDIV = np.ceil(ref / 70e6)
        FPFD = ref / RDIV

        '''Calculating NDIV'''
        ratio = freq / ref
        if (NOUT == 0):
            NDIV = ratio * RDIV
        else:
            NDIV = ratio * RDIV * NOUT
        NINT = np.floor(NDIV)
        NFRAC = np.ceil((2 ** 24) * (NDIV - NINT))

        '''Calculate the exact frequency mode register'''
        fraction = Fraction(float(NDIV) - float(NINT))
        EXACT = 1 / (fraction.limit_denominator())

        '''Calculate charge pump settings'''
        if (NOUT == 0):
            FVCO = freq
        else:
            FVCO = freq * NOUT
        ICP = 1.1e-3 + (2.3e-3 - 1.1e-3) * (FVCO - 1.5e9) / (3e9 - 1.5e9)
        ICPOffset = ((2.5e-9) + 4 * (1 / FVCO)) * (ICP / (1 / (FPFD)))

        '''Lock detection window settings'''
        LDWindow = (((ICPOffset) / (FPFD * ICP)) + (2.66e-9) + (1 / FPFD)) / 2

        '''Print out settings'''
        if verbose and False:
            print("-------PLL Configuration-------")
            print("Freq   : " + str(freq * 1e-9) + " GHz")
            print("Ref    : " + str(ref * 1e-6) + " MHz")
            print("Ratio  : " + str(ratio))
            print("NOUT   : " + str(NOUT))
            print("RDIV   : " + str(RDIV))
            print("FPFD   : " + str(FPFD * 1e-6) + " MHz")
            print("NDIV   : " + str(NDIV))
            print("NINT   : " + str(NINT))
            print("NFRAC  : " + str(NFRAC))
            print("DEN    : " + str(EXACT))
            print("FVCO   : " + str(FVCO * 1e-9) + " GHz")
            print("ICP    : " + str(ICP * 1e6) + " uA")
            print("ICPOff : " + str(ICPOffset * 1e6) + " uA")
            print("LD Wind: " + str(LDWindow * 1e9) + " ns")
            print("------------------------------")
        '''Check minimum divide ratio (20 for fractional mode)'''
        if (NINT < 20):
            print("ERROR: Minimum divide ratio not met")
            return

        if verbose:
            print("Configuring device..."),

        '''Configure reference divider'''
        RDIV = int(RDIV)
        self.writeReg(0x02, RDIV)

        '''Configure delta-sigma modulator'''
        DCM = 2
        DCM = DCM | 2 << 2
        DCM = DCM | 4 << 4
        DCM = DCM | 0 << 7
        DCM = DCM | 1 << 8
        DCM = DCM | 1 << 9
        DCM = DCM | 1 << 10
        DCM = DCM | 1 << 11
        self.writeReg(0x06, DCM)

        '''Configure autocal'''
        AC = 0x2046
        self.writeReg(0x0A, AC)

        '''Configure charge pump current'''
        '''Calculate register'''
        # CPC = 0x30E5CB
        CPC = 1 << 21  # Enable up offset
        CP = int(np.round(ICP / 20e-6))
        CPC = CPC | CP << 0
        CPC = CPC | CP << 7
        CPO = int(np.round(ICPOffset / 5e-6))
        CPC = CPC | CPO << 14
        self.writeReg(0x09, CPC)

        '''Configure lock detect'''
        # LD = 0x94D
        LD = 5 << 0
        LD = LD | 1 << 3
        LD = LD | 1 << 6
        divide, speed = self.LDLookup(LDWindow)
        LD = LD | int(divide) << 7
        LD = LD | int(speed) << 10
        self.writeReg(0x07, LD)

        '''Configure VCO subsystem, VSPI clock = 12.5MHz'''
        # self.writeReg(0x05,0xFF88) # Enable outputstage
        switchSettings = self.readReg(0x10)
        switchSettings = switchSettings & 0b011111111
        self.writeReg(0x05, 0xF98)
        # Low phase noise mode
        # Enable RF_N and RF_P
        # Internal termination
        time.sleep(0.001)
        self.writeReg(0x05, 0x4B38)  # Set output gain to 1.4V
        time.sleep(0.001)
        # self.writeReg(0x01,0xFF88) #Set output on
        VCOSS = (0x0 | switchSettings << 1) << 7
        self.writeReg(0x05, VCOSS)
        time.sleep(0.001)

        '''Programming the frequency of operation'''
        self.writeReg(0x03, int(NINT))
        # self.writeReg(0x0C,int(EXACT))
        self.writeReg(0x04, long(NFRAC))

        '''Setting the output divider'''
        switchSettings = self.readReg(0x10)
        switchSettings = switchSettings & 0b011111111
        NOUTReg = 0 << 0
        NOUTReg = NOUTReg | 2 << 3
        NOUTReg = NOUTReg | int(NOUT) << 7
        self.writeReg(0x05, NOUTReg)  # 0x90 Set VCO div
        time.sleep(0.001)
        VCOSS = (0x0 | switchSettings << 1) << 7
        self.writeReg(0x05, VCOSS)
        time.sleep(0.001)

        print("[DONE]")
        print("Verifying configuration..."),
        if verbose and False:
            print("0x02: " + hex(RDIV))
            print("0x03: " + hex(NINT))
            print("0x04: " + hex(NFRAC))
            print("0x05: " + hex(VCOSS))
            print("0x06: " + hex(DCM))
            print("0x07: " + hex(self.readReg(0x07)))
            print("0x08: " + hex(self.readReg(0x08)))
            print("0x09: " + hex(CPC))
            print("0x0A: " + hex(self.readReg(0x0A)))
            print("0x0B: " + hex(self.readReg(0x0B)))
            print("0x0C: " + hex(self.readReg(0x0C)))
        if (RDIV == self.readReg(0x02) and
                    DCM == self.readReg(0x06) and
                    CPC == self.readReg(0x09) and
                    LD == self.readReg(0x07) and
                    VCOSS == self.readReg(0x05) and
                    NINT == self.readReg(0x03) and
                    NFRAC == self.readReg(0x04)):
            print("[SUCCESS]")
        else:
            print("[Fail]")

        time.sleep(0.001)
        return self.checkLock()

    def checkLock(self):
        '''Polls the lock register'''
        if verbose:
            print("Checking Lock..."),
        GPO2Reg = self.readReg(0x12)
        locked = GPO2Reg & 0b10
        if locked > 0:
            if verbose:
                print("[LOCKED]")
            return True
        else:
            if verbose:
                print("[NOT LOCKED]")
            return False

    def readVCOTuneRegister(self):
        '''Reads the status of the VCO tune register'''
        if verbose:
            print("VCO Subsystem Status:")
        VCOTune = self.readReg(0x10)
        VCOSS = VCOTune & 0b011111111
        if VCOTune & 0b100000000 > 0:
            autoCal = 'Busy'
        else:
            autoCal = 'Not Busy'
        if verbose:
            print("Current VCO setting: " + str(VCOSS))
            print("AutoCal: " + str(autoCal))

    def setupPins(self):
        '''Sets up the GPIO pins'''
        '''Configure the behaviour of the GPIO pins'''
        GPIO.setwarnings(False)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(PWR_EN, GPIO.OUT)
        GPIO.setup(CEN, GPIO.OUT)
        GPIO.setup(SEN, GPIO.OUT)
        GPIO.setup(SDO, GPIO.IN)
        GPIO.setup(SCK, GPIO.OUT)
        GPIO.setup(SDI, GPIO.OUT)

        '''Set default states'''
        GPIO.output(PWR_EN, GPIO.LOW)
        GPIO.output(CEN, GPIO.LOW)
        GPIO.output(SEN, GPIO.LOW)
        GPIO.output(SDI, GPIO.LOW)
        GPIO.output(SCK, GPIO.LOW)

    def power(self, powerOn=True):
        '''Turn on power to the module'''
        if powerOn:
            if verbose:
                print("Applying power to Synth")
            GPIO.output(PWR_EN, GPIO.HIGH)
            time.sleep(0.001)

            if verbose:
                print("Setting up SPI mode to HMC")
            self.setSPIMode("HMC")
            time.sleep(0.001)

            '''Configure the function of the SDO/LD line'''
            self.writeReg(0x0F, (0b000 << 5) | 1)
            time.sleep(0.001)

            if verbose:
                '''Check the device ID'''
                self.checkDeviceID()
        else:
            if verbose:
                print("Pulling serial pins low")
            GPIO.output(SCK, GPIO.LOW)
            GPIO.output(SDI, GPIO.LOW)
            time.sleep(0.001)
            if verbose:
                print("Disabling serial port")
            GPIO.output(SEN, GPIO.LOW)
            time.sleep(0.001)
            if verbose:
                print("Disabling device")
            GPIO.output(CEN, GPIO.LOW)
            time.sleep(0.001)
            if verbose:
                print("Shutting down power")
            GPIO.output(PWR_EN, GPIO.LOW)

    def setSPIMode(self, mode="HMC"):
        '''Sets up the communication mode to the module, must be the first command sent to the module.'''
        GPIO.output(SCK, GPIO.LOW)
        GPIO.output(SEN, GPIO.LOW)
        GPIO.output(CEN, GPIO.LOW)
        time.sleep(0.001)
        GPIO.output(CEN, GPIO.HIGH)
        time.sleep(0.001)

        if mode == "HMC":
            if verbose:
                print("Pulsing SEN")
            GPIO.output(SEN, GPIO.LOW)
            time.sleep(0.001)
            GPIO.output(SEN, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(SEN, GPIO.LOW)
        elif mode == "Open":
            GPIO.output(SCK, GPIO.LOW)
            time.sleep(0.001)
            GPIO.output(SCK, GPIO.HIGH)
            time.sleep(0.001)
            GPIO.output(SCK, GPIO.LOW)
        else:
            print("This mode is not supported")

    def writeReg(self, addr, data, mode="HMC"):
        '''Writes data to the specified register'''
        if mode == "HMC":
            '''Indicate a write cycle'''
            GPIO.output(SEN, GPIO.HIGH)
            GPIO.output(SDI, GPIO.LOW)
            self.pulseSCK()

            '''Send the 6 address bits'''
            # print('Address')
            self.shiftInData(addr, 6)

            '''Send 24 data bits'''
            # print('Data')
            self.shiftInData(data, 24)
            self.pulseSCK()

            '''Finish the write cycle'''
            time.sleep(0.001)
            GPIO.output(SEN, GPIO.LOW)
            time.sleep(0.001)
            GPIO.output(SDI, GPIO.LOW)

        else:
            print("Writing in this mode is currently not supported")

    def readReg(self, addr, mode="HMC"):
        '''Reads data from the specified register'''
        if mode == "HMC":
            '''Indicate a read cycle'''
            GPIO.output(SEN, GPIO.HIGH)
            GPIO.output(SDI, GPIO.HIGH)
            self.pulseSCK()

            '''Send the 6 address bits'''
            self.shiftInData(addr, 6)

            '''Read the 24 data bits'''
            recData = 0b000000000000000000000000
            self.pulseSCK()
            for i in np.arange(24):
                if GPIO.input(SDO):
                    recData += 1 << (23 - i)

                self.pulseSCK()

            '''Finish the write cycle'''
            time.sleep(0.001)
            GPIO.output(SEN, GPIO.LOW)
            time.sleep(0.001)
            GPIO.output(SDI, GPIO.LOW)

            return recData

        else:
            print("Reading in this mode is currently not supported")

    def pulseSCK(self):
        '''Send a rising edge pulse on the clock line'''
        GPIO.output(SCK, GPIO.LOW)
        GPIO.output(SCK, GPIO.HIGH)

    def shiftInData(self, data, length):
        for i in np.arange(length):
            if data & (1 << ((length - 1) - i)) > 0:
                GPIO.output(SDI, GPIO.HIGH)
            else:
                GPIO.output(SDI, GPIO.LOW)
            self.pulseSCK()

    def checkDeviceID(self):
        '''Checks if the device ID is correct'''
        print("Checking device ID..."),
        id = self.readReg(0x00)
        print(str(hex(id)) + "..."),
        if (id == 0xa7975):
            print("[OK]")
        else:
            print("[FAIL]")

    def LDLookup(self, window):
        '''Returns the ideal speed and divide settings for the window size'''
        window = window * 1e9
        if (window < 7.6):
            divide = 0b000
            if (window < 6.5):
                speed = 0b00
            elif (window < 7):
                speed = 0b01
            elif (window < 7.1):
                speed = 0b01
            elif (window < 7.6):
                speed = 0b11
        elif (window < 10.2):
            divide = 0b001
            if (window < 8):
                speed = 0b00
            elif (window < 8.9):
                speed = 0b01
            elif (window < 9.2):
                speed = 0b01
            elif (window < 10.2):
                speed = 0b11
        elif (window < 15.4):
            divide = 0b010
            if (window < 11):
                speed = 0b00
            elif (window < 12.8):
                speed = 0b01
            elif (window < 13.3):
                speed = 0b01
            elif (window < 15.4):
                speed = 0b11
        elif (window < 26):
            divide = 0b011
            if (window < 17):
                speed = 0b00
            elif (window < 21):
                speed = 0b01
            elif (window < 22):
                speed = 0b01
            elif (window < 26):
                speed = 0b11
        elif (window < 47):
            divide = 0b100
            if (window < 29):
                speed = 0b00
            elif (window < 36):
                speed = 0b01
            elif (window < 38):
                speed = 0b01
            elif (window < 47):
                speed = 0b11
        elif (window < 88):
            divide = 0b101
            if (window < 53):
                speed = 0b00
            elif (window < 68):
                speed = 0b01
            elif (window < 72):
                speed = 0b01
            elif (window < 88):
                speed = 0b11
        elif (window < 172):
            divide = 0b110
            if (window < 100):
                speed = 0b00
            elif (window < 130):
                speed = 0b01
            elif (window < 138):
                speed = 0b01
            elif (window < 172):
                speed = 0b11
        elif (window < 338):
            divide = 0b111
            if (window < 195):
                speed = 0b00
            elif (window < 255):
                speed = 0b01
            elif (window < 272):
                speed = 0b01
            elif (window < 338):
                speed = 0b11
        else:
            divide = 0b111
            speed = 0b11
        return divide, speed
