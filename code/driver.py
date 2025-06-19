'''
!/usr/bin/env python3

 driver.py

Makes the motor and drive system. Also runs flower power.
'''

# Imports
from global_imports import *

# Motor3
PIN_MOTOR1_LEGA = 7  # IN1
PIN_MOTOR1_LEGB = 8  # IN2

# Motor2
PIN_MOTOR2_LEGA = 6  # IN3
PIN_MOTOR2_LEGB = 5  # IN4
wheel_diameter = 65  # mm


class Motor():
    '''
    Makes motor object.
    '''

    def __init__(self, io, pin1, pin2, sr):
        '''
        Initializes motor object and sets PWM range and frequency.

        :param io: The GPIO of the raspberry pi
        :param pin1: First motor pin
        :param pin2: Second motor pin
        :param sr: Ratio to match motor speeds
        '''
        self.sr = sr
        self.io = io
        self.MOTOR_PIN1 = pin1
        self.MOTOR_PIN2 = pin2

        self.io.set_mode(self.MOTOR_PIN1, pigpio.OUTPUT)
        self.io.set_mode(self.MOTOR_PIN2, pigpio.OUTPUT)
        self.io.set_PWM_range(self.MOTOR_PIN1, 255)
        self.io.set_PWM_range(self.MOTOR_PIN2, 255)
        self.io.set_PWM_frequency(self.MOTOR_PIN1, 1000)
        self.io.set_PWM_frequency(self.MOTOR_PIN2, 1000)

    def set_level(self, level):
        '''
        Sets level of the motor.

        :param level: PWM level 
        '''
        level = level / self.sr
        
        check_warning(abs(level) > 255, 
                      f"WARNING: Level is out of Range. Level is {level}. Defaulting magnitude of level to 255")
        level = clamp(level, -255, 255)

        if level > 0:
            self.io.set_PWM_dutycycle(self.MOTOR_PIN2, 0)
            self.io.set_PWM_dutycycle(self.MOTOR_PIN1, level)
        else:
            self.io.set_PWM_dutycycle(self.MOTOR_PIN1, 0)
            self.io.set_PWM_dutycycle(self.MOTOR_PIN2, -level)

    def stop(self):
        '''
        Stops motor.
        '''
        self.set_level(0)

class DriveSystem():
    '''
    Makes a drive system made up of 2 motors.
    '''
    def __init__(self, io, motor1_pin1, motor1_pin2, motor2_pin1, motor2_pin2):
        '''
        Initializes drive system.

        :param io: GPIO of the raspberry pi
        :param motor1_pin1: Pin 1 of first motor
        :param motor1_pin2: Pin 2 of first motor
        :param motor2_pin1: Pin 1 of second motor
        :param motor2_pin2: Pin 2 of second motor
        '''
        self.wheel1 = Motor(io, motor1_pin1, motor1_pin2, 1)
        self.wheel2 = Motor(io, motor2_pin1, motor2_pin2, 1)

    def drive_slow_enum(self, style): 
        '''
        Drives the robot based on a given arc.

        :param style: Arc style
        '''
        levels = style.value

        left_wheel_value = levels[0] * 0.9
        right_wheel_value = levels[1] * 0.9

        self.drive((left_wheel_value, right_wheel_value))

    def drive_enum(self, style, backwards=False, speed_multiplier=1.1): 
        '''
        Drives the robot based on a given arc.

        :param style: Arc style
        :param backwards: whether to go backwards
        '''
        if(not backwards):
            levels = (style.value[0]*speed_multiplier, style.value[1]*speed_multiplier)
        else:
            levels = (style.value[0]*-speed_multiplier, style.value[1]*-speed_multiplier)
        self.drive(levels)

    def drive(self, levels):
        '''
        Drives robot based on a tuple of levels.

        :param levels: A tuple of the motor levels
        '''
        self.wheel1.set_level(levels[0])
        self.wheel2.set_level(levels[1])
    
    def stop(self):
        '''
        Stops both motors.
        '''
        self.wheel1.stop()
        self.wheel2.stop()

#
#   Main: Runs flower power
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

    drive_system = DriveSystem(io, PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, PIN_MOTOR2_LEGA, PIN_MOTOR2_LEGB)

    # Runs each arc style for 4 seconds
    try:
        # for style in Steer_Style:
            # drive_system.drive_enum(style)
            # time.sleep(4)
            # drive_system.stop()
            # input("Hit return")
        
        drive_system.drive_enum(Steer_Style.SPIN_RIGHT, speed_multiplier=1)
        time.sleep(4)
        drive_system.stop()

    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    ############################################################
    # Turn Off.
    # Note the PWM will stay at the last commanded value.  So you want
    # to be sure to set to zero before the program closes.  Else your
    # robot will run away...
    drive_system.stop()
    print("Turning off...")

    # Stop the interface.
    io.stop()

