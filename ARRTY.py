import RPi.GPIO as GPIO
import ADCCon
import SynthCon
import FPGACon

import sys
import time
import numpy as np
import matplotlib.pyplot as plt


class ARRTY():
    def __init__(self):
        # Turn on Synth
        self.synth = SynthCon.SynthCon()
        self.synth.power(True)
        time.sleep(0.01)
        self.synth.setFrequency(1000.001e6)

        # Turn on ADC
        self.adc = ADCCon.ADCCon()
        time.sleep(0.01)

        time.sleep(1)
        # Initialise FPGA
        # self.fpga = FPGACon.FPGACon()
        # time.sleep(0.01)

        # Set ramping pattern
        self.adc.setFramingPattern(False)
        # self.adc.setTestValuePattern(200)
        time.sleep(0.1)

        # Set ADC internal gain to 50x
        self.adc.setGain()

        #triggerLevel = int(sys.argv[1])
        #print("Setting trigger level to " + str(triggerLevel) + "..."),
        #self.fpga.setTriggerLevel(triggerLevel)
        #print("[DONE]")
        #sampleLength = int(sys.argv[2])
        #print("Setting sample length to " + str(sampleLength) + "..."),
        #self.fpga.setSampleLength(sampleLength)
        #print("[DONE]")

    def shutdown(self):
        self.adc.power(False)
        self.synth.power(False)


try:
    arrty = ARRTY()

    while (1):
        #arrty.fpga.setTrigger()
        #print("Waiting for sample..."),
        #while (arrty.fpga.readDOUT() == 0):
        #    time.sleep(0.001)
        #print("[CAPTURED]")
        #print("Reading sample:")
        #sample = arrty.fpga.readData()
        time.sleep(100)

except KeyboardInterrupt:
    arrty.shutdown()
    print("Putting ARRTY to sleep...")
    GPIO.cleanup()
