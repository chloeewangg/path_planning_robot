'''
!/usr/bin/env python3

robot.py
Responsible for the running the robot from the highest level.
'''

# Imports
from global_imports import * 
from shared import *
from brain import *
from behaviors import *
from ui import *
from ros import run_ros
import ctypes
from NFC_sensor import *
from electromagnet import *
import board
import busio

from adafruit_pn532.i2c import PN532_I2C

# Define the motor pins.
PIN_MOTOR1_LEGA = 7
PIN_MOTOR1_LEGB = 8
PIN_MOTOR2_LEGA = 6
PIN_MOTOR2_LEGB = 5

# Define the IR pins.
RIGHT_IR = 18
MIDDLE_IR = 15
LEFT_IR = 14

# Define the magnetometer pins.
LATCH_MAGNETOMETER = 27
ADDRESS_MAGNETOMETER = 4
READY_MAGNETOMETER = 17
PINS_MAGNETOMETER = [9, 10, 11, 12, 22, 23, 24, 25]

# Define proximity sensor pins
TRIG1 =  19
ECHO1 = 20
TRIG2 = 26
ECHO2 = 21
TRIG3 = 13 
ECHO3 = 16

# Define electromagnet pin
MAGNET_TOGGLE = 1

io = None

def shutdown(drive_system, proximity_sensor, UI_thread, ros_thread, NFC_sensor, electromagnet):
    '''
    Shuts down and joins all threads.

    :param drive_system: the object responsible for driving the wheels
    :param proximity_sensor: the object responsible for proximity sensor
    :param UI_thread: the UI thread
    :param ros_thread: the ROS thread
    :param NFC_sensor: the object responsible for reading NFCs
    '''

    drive_system.stop()
    proximity_sensor.shutdown()
    ros_thread.join()
    NFC_sensor.shutdown()
    electromagnet.off()
    UI_thread.join()
    print("Shutting down...")

#
#   Main: Makes the robot follow the line while being smart.
#
if __name__ == "__main__":

    ############################################################
    # Prepare the GPIO interface/connection (to command the motors).
    print("Setting up the GPIO...")
    io = pigpio.pi()
    if not io.connected:
        print("Unable to connection to pigpio daemon!")
        sys.exit(0)
    print("GPIO ready...")
    ############################################################

    # create shared data
    shared = Shared()

    # object responsible for moving the robot
    drive_system = DriveSystem(io, PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, PIN_MOTOR2_LEGA, PIN_MOTOR2_LEGB)
    # object responsible for reading the three sensor values
    line_sensor = LineSensor(io, RIGHT_IR, MIDDLE_IR, LEFT_IR)
    # object responsible for reading the current angle value
    angle_sensor  = ADC(io, LATCH, ADDRESS, READY, PINS)
    # object responsible for blockage detecting
    proximity_sensor = ProximitySensor(io, TRIG1, ECHO1, TRIG2, ECHO2, TRIG3, ECHO3)
    # object responsible for NFC reader
    nfc_reader = NFC()
    # object responsible for magnet
    electromagnet = Electromagnet(io, MAGNET_TOGGLE)

    
    # object responsible for all the brain's behaviors
    behaviors = Behaviors(drive_system, line_sensor, angle_sensor, proximity_sensor, nfc_reader, electromagnet)


    # start robot ros thread
    ros_thread = threading.Thread(name="ROS Thread", target = run_ros, args=(shared,))
    ros_thread.start()

    wait_seconds(0.5)

    # start robot brain
    robot_brain = Brain(shared, behaviors)
    robot_brain.robot_start()

    # start robot UI thread
    robot_UI = UI(shared)
    UI_thread = threading.Thread(name="UIThread", target=robot_UI.run_UI)
    UI_thread.daemon = True

    try:
        UI_thread.start()
        
        robot_brain.run_robot()

    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    ############################################################
    # Turn Off.

    shutdown(drive_system, proximity_sensor, UI_thread, ros_thread, nfc_reader, electromagnet)
    print("Turning off...")
    # Shuts down sensors
    proximity_sensor.shutdown()
    nfc_reader.shutdown()
    electromagnet.off()
    # Stop the interface.
    io.stop()
    ctypes.pythonapi.PyThreadState_SetAsyncExc(ctypes.c_long(ros_thread.ident), ctypes.py_object(KeyboardInterrupt))
    ros_thread.join()

