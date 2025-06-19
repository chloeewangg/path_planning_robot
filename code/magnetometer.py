'''
!/usr/bin/env python3

magnetometer.py
Does everything magnetometer related. 
'''

# Imports
import pigpio
import sys
import time
import traceback
import numpy as np
from global_functions import *
from math import atan2, pi

# Define the magnetometer pins.
LATCH = 27
ADDRESS = 4
READY = 17
PINS = [9, 10, 11, 12, 22, 23, 24, 25]

class ADC:
    '''
    Manages the magnetometer. 
    '''
    def __init__(self, io, latch, address, ready, pins):
        self.io = io
        self.address = address
        self.latch = latch
        self.ready = ready
        self.bit_pins = pins
        self.io.set_mode(address, pigpio.OUTPUT)
        self.io.set_mode(latch, pigpio.OUTPUT)

        for pin in pins:
            self.io.set_mode(pin, pigpio.INPUT)

        self.x_bit_min = 90
        self.x_bit_max = 200
        self.y_bit_min = 90
        self.y_bit_max = 200


    def read_adc(self, address_value):
        """
        Specifies which magnetometer is read and converts analog signal to digital 
        by latching onto the signal and converting it to values that can be read by each pin. 
        Value from each pin is extracted and pushed to a  binary to int converter.
        Binary to Integer conversion
        8 bit int value += pin# value * 2^(pin #)
        ex) pin0 would be 2^0 and it would get multiplied by the value which is either 1 or 0.

        :param io: the GPIO of the raspberry pi
        :param address_value: the address value of the magnetometer
        :return: the bit value of the magnetometer address reading
        """
        self.io.write(self.latch, 0)
        self.io.write(self.address, address_value)
        self.io.write(self.latch, 1)
        self.io.write(self.latch, 0)
        self.io.write(self.latch, 1)
        ready = False
        while not ready:
            if self.io.read(self.ready) == 1:
                ready = True

        bit_value = 0
        for i in range(len(self.bit_pins)):
            bit_value = bit_value + (2**i)*self.io.read(self.bit_pins[i])

        return bit_value


    def read_angle(self):
        """
        Reads one magnetometer by specifying the address value. 
        Calls read_adc(address), which returns 8 bit int. 
        That integer is mapped to a value between -1 and 1 using the map function. 
        Then finds the angle via atan2 math function.
        
        :param io: the GPIO of the raspberry pi
        :return: the current angle reading from the magnetometer
        """
        x_bit = self.read_adc(1)
        y_bit = self.read_adc(0)

        x_bit = clamp(x_bit, self.x_bit_min, self.x_bit_max)
        y_bit = clamp(y_bit, self.y_bit_min, self.y_bit_max)

        x_component = map_values(x_bit, self.x_bit_min, self.x_bit_max, -1, 1)
        y_component = map_values(y_bit, self.y_bit_min, self.y_bit_max, -1, 1)
        angle = atan2(y_component, x_component) * 180 / pi

        return angle
    
    def read_avg_angle(self):
        angles = []

        for i in range(0, 5):
            angles.append(self.read_angle())
        
        avg_angle = np.median(angles)

        return avg_angle

#   Main: Prints out three sensor values.
#
if __name__ == "__main__":
    ############################################################
    # Prepare the GPIO interface/connection 
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")
    ############################################################

    # instantiate magnetometer object
    magnetometer  = ADC(io, LATCH, ADDRESS, READY, PINS)
    
    try:
        t0 = time.time()

        while(True):
            angle = magnetometer.read_angle()
            print("Angles:" + str(angle))
            input("hit return")


    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Stop the interface.
    io.stop()