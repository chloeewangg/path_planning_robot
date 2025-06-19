'''

!/usr/bin/env python3

IR_sensor.py
Does everything sensor related. Contains class for managing one-sensor reading
and for managing all three sensors. 

'''

# Imports
import pigpio
import sys
import time
import traceback
from global_enums import Sensor_Input

# Define the motor pins.
IR_right = 18
IR_middle = 15
IR_left = 14

class IR:
    '''
    Manages one IR sensor. 
    '''
    
    def __init__(self, io, pin): 
        '''
        Sets up the IR sensor.

        :param io: the GPIO of the raspberry pi
        :param pin: the pin number of the IR sensor
        '''
        self.io = io
        self.IR = pin
        self.io.set_mode(pin, pigpio.INPUT)

    def read_val(self):
        '''
        Sets up the IR sensor.

        :return: the IR sensor reading -- 0 for white floor and 1 for black tape
        '''
        return self.io.read(self.IR)
    
class LineSensor:
    '''
    Manages all three IR sensors.
    '''

    def __init__(self, io, pin1, pin2, pin3):
        '''
        Sets up the three IR sensors.

        :param io: the GPIO of the raspberry pi
        :param pin1: the pin number of IR sensor 1
        :param pin2: the pin number of IR sensor 2
        :param pin3: the pin number of IR sensor 3
        '''
        self.right_sensor = IR(io, pin1)
        self.middle_sensor = IR(io, pin2)
        self.left_sensor = IR(io, pin3)

    def read_vals(self):
        '''
        Sets up the IR sensor.

        :return: the IR sensor readings as a triple -- 0 for white floor and 1 for black tape
        '''
        right = self.right_sensor.read_val()
        mid = self.middle_sensor.read_val()
        left = self.left_sensor.read_val()
        return (right, mid, left)

#
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


    LINE_SENSORS = LineSensor(io, IR_right, IR_middle, IR_left)

    try:
        t0 = time.time()

        while(time.time() - t0 <= 500):
            sensor_vals = LINE_SENSORS.read_vals()
            print("IRs: L %d  M %d  R %d" % (sensor_vals[0], sensor_vals[1], sensor_vals[2]))
            print(Sensor_Input(sensor_vals).name)
            input("hit return to read value again")

        print("Ending due to overtime (20sec)")

    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Stop the interface.
    io.stop()