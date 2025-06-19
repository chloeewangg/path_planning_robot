'''
!/usr/bin/env python3

proximity_behaviors.py
Contains behaviors using ultrasound sensors. Wall-following and herding.
'''
# Imports

from global_imports import *
from proximity_sensor import *
from shared import *

# Define the ultrasonic sensor pins.
trig1 =  19
echo1 = 20
trig2 = 26
echo2 = 21
trig3 = 13 
echo3 = 16


PIN_MOTOR1_LEGA = 7
PIN_MOTOR1_LEGB = 8
PIN_MOTOR2_LEGA = 6
PIN_MOTOR2_LEGB = 5


def run_ui(shared):
    '''
    The user interface for proximity behaviors.

    :param shared: the shared data object for accessing the mode.

    '''
    try:
        while True:

            cmd = input(("Right wall follow with Enum (E) / Right wall follow (R) / Stop (S) / Quit (Q)"))
            if(cmd == "E"):
                mode = 1
            elif(cmd == "R"):
                mode = 2
            elif(cmd == "S"):
                mode = 0
            elif(cmd == "Q"):
                mode = -1

            if shared.acquire():
                shared.proximity_mode = mode
                shared.release()
    except:
        print("Ending Run-UI due to exception: %s" % repr(ex))

def herding_behavior(drive_system, proximity_sensor):
    '''
    Makes the robot go in opposite direction of whatever obstacle. 

    :param drive_system: the object responsible for the wheels
    :param proximity_sensor: the object responsible for the three ultrasound sensors
    '''

    while(True):
        left_reading, front_reading, right_reading = proximity_sensor.read_vals()
        left, front, right = (get_distance_enum(left_reading, True), 
                              get_distance_enum(front_reading, False), 
                              get_distance_enum(right_reading, True))
        
        if front == Distance.NEAR:
            if left == right:
                drive_system.drive_enum(Steer_Style.STRAIGHT, backwards=True)
            elif left == Distance.NEAR:
                drive_system.drive_enum(Steer_Style.TURN_LEFT, backwards=True)
            elif right == Distance.NEAR:
                drive_system.drive_enum(Steer_Style.TURN_RIGHT, backwards=True)
        elif front == Distance.MIDDLE:
            if left == right:
                drive_system.stop()
            elif left == Distance.NEAR:
                drive_system.drive_enum(Steer_Style.SPIN_RIGHT)
            elif right == Distance.NEAR:
                drive_system.drive_enum(Steer_Style.SPIN_LEFT)
        elif front == Distance.FAR:
            if left == right:
                drive_system.drive_enum(Steer_Style.STRAIGHT)
            elif left == Distance.NEAR:
                drive_system.drive_enum(Steer_Style.TURN_RIGHT)
            elif right == Distance.NEAR:
                drive_system.drive_enum(Steer_Style.TURN_LEFT)

def get_distance_enum(reading, is_side):
    '''
    Turns a numeric value distance to a easy-to-understand distance enum

    :param reading: the numeric distance reading
    :param is_side: if the reading is from a side sensor
    :return: a Distance enum corresponding to the numeric distance reading
    '''

    if(reading > Distance.FAR.value):
        return Distance.FAR
    if(reading > Distance.MIDDLE.value):
        if(is_side): 
            return Distance.NEAR
        return Distance.MIDDLE
    return Distance.NEAR

def right_wall_follow_enum(drive_system, proximity_sensor, nominal_distance):
    '''
    Follows the wall to the right side using enums for error ranges

    :param drive_system: the object responsible for the wheels
    :param proximity_sensor: the object responsible for the three ultrasound sensors
    :param nominal_distance: the nominal distance between the wall and the robot
    '''

    threshold1 = 0.5
    threshold2 = 5
    threshold3 = 9
    stop_threshold = 10

    # trigger first so it can initialize distance
    proximity_sensor.read_vals()
    
    left, front, right = proximity_sensor.read_vals()
    
    error  = nominal_distance - right

    # stop when error gets too big or there is an object in front
    if(abs(error) > stop_threshold or front < nominal_distance):
        drive_system.stop()

    if error > threshold1:
        if error <= threshold2: # threshold1 ~ threshold2
            drive_system.drive_enum(Steer_Style.VEER_LEFT)
        elif error <= threshold3: # threshold2 ~ threshold3
            drive_system.drive_enum(Steer_Style.STEER_LEFT)
        else:
            drive_system.drive_enum(Steer_Style.TURN_LEFT)
    elif error < threshold1:
        if -threshold2 <= error: # -threshold2 ~ -threshold1
            drive_system.drive_enum(Steer_Style.VEER_RIGHT)
        elif -threshold3 <= error: # -threshold3 ~ -threshold2
            drive_system.drive_enum(Steer_Style.STEER_RIGHT)
        else:
            print("Turning right)")
            drive_system.drive_enum(Steer_Style.TURN_RIGHT)
    else: # -threshold1 ~ threshold1
        drive_system.drive_enum(Steer_Style.STRAIGHT)

def right_wall_follow(drive_system, proximity_sensor, nominal_distance):
    '''
    Follows the wall to the right side calculating pwms directly from errors

    :param drive_system: the object responsible for the wheels
    :param proximity_sensor: the object responsible for the three ultrasound sensors
    :param nominal_distance: the nominal distance between the wall and the robot
    '''

    stop_threshold = 10

    k_r = -4.57
    c_r = 210

    k_l = 5.61
    c_l = 215

    # trigger first so it can initialize distance
    proximity_sensor.read_vals()

    left, front, right = proximity_sensor.read_vals()
    
    error  = nominal_distance - right

    # stop when error gets too big or there is an object in front
    if(abs(error) > stop_threshold or front < nominal_distance):
        drive_system.stop()

    
    right_pwm = clamp(c_r + k_r * error, 0, 255)
    left_pwm = clamp(c_l + k_l * error, 0, 255)
    drive_system.drive((right_pwm, left_pwm))

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

    drive_system = DriveSystem(io, PIN_MOTOR1_LEGA, PIN_MOTOR1_LEGB, PIN_MOTOR2_LEGA, PIN_MOTOR2_LEGB)
    proximity_sensor = ProximitySensor(io, 19, 20, 26, 21, 13, 16)
    
    nominal_distance = 30
    shared = Shared()
    uithread = threading.Thread(name = "UIThread", target = run_ui, args=(shared, ))
    uithread.daemon = True
    uithread.start()

    try:
        while True:
            t_last = time.time() 
            if shared.acquire():
                mode = shared.proximity_mode
                shared.release()

            if(time.time() - t_last > 3):
                print(f"mode{mode}")
                t_last = time.time()

            if(mode < 0):
                break        
            elif(mode == 0):
                drive_system.stop()
            elif(mode == 1):
                right_wall_follow_enum(drive_system, proximity_sensor, nominal_distance) 
            elif(mode == 2):
                right_wall_follow(drive_system, proximity_sensor, nominal_distance)

    except BaseException as ex:
        # Report the error, but continue with the normal shutdown.
        print("Ending due to exception: %s" % repr(ex))
        traceback.print_exc()

    # Stop the interface.
    drive_system.stop()
    proximity_sensor.shutdown()
    io.stop()