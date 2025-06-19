'''
!/usr/bin/env python3

behaviors.py
Contains all functions necessary for the robot's behaviors.
'''

# Imports
from global_imports import *
import numpy as np
import matplotlib.pyplot as plt


class Behaviors():
    def __init__(self, drive_system, line_sensor, angle_sensor, proximity_sensor, nfc_reader, electromagnet):
        '''
        Initializes all necessary objects.

        :param drive_system: the object responsible for moving the wheels
        :param line_sensor: the object responsible for managing the three IR sensors
        :param angle_sensor: the object responsible for managing the magnetometer
        :param proximity_sensor: the object reseponsible for managing the ultrasonic sensors
        '''

        # initialize objects
        self.drive_system = drive_system
        self.line_sensor = line_sensor
        self.angle_sensor = angle_sensor
        self.proximity_sensor = proximity_sensor
        self.nfc_reader = nfc_reader
        self.electromagnet = electromagnet

        self.blocked = False

        # initialize variables
        self.time_constant = 0.1  # time constant for averaging sensor values
        # (0 or 1, 0 or 1, 0 or 1); the sensor values we are acted upon, changed when estimator exceeds self.threshold
        self.sensor_values = (0, 1, 0)
        # (0 ~ 1, 0 ~ 1, 0 ~ 1); the average of the sensor values over time
        self.sensor_estimators = (0, 1, 0)

        self.road_side_time_constant = 0.01
        # -1, 0, or +1; the road side value we are acted upon, changed when estimator exceeds self.threshold
        self.road_side_value = 0
        self.road_side_estimator = 0  # -1 ~ +1; the average of the road side value over time

        # time constant for detecting intersection
        self.intersection_time_constant = 0.12
        # 0 ~ +1; the average of the intersection value over time
        self.intersection_estimator = 0

        self.road_end_time_constant = 0.25  # time constant for detecting road end
        self.road_end_estimator = 0  # 0 ~ +1; the average of the road end value over time

        self.road_prize_time_constant = 0.1
        self.road_prize_time_estimator = 0

        # True when robot is correcting offroad, to suppress end of road detection during correction
        self.is_correcting_offroad = False

        self.threshold = 0.37  # self.threshold value for updating value from estimator

    def follow_road(self, prize=False):
        '''
        Robot behavior for moving straight. Constantly detects intersection, road end, and corrects if goes off line. Detects prizes if state of prize indicates to do so.

        :return: Road_State enum whether road ended in intersection or end
        :prize: if prize is true, then prize detector is enabled.
        '''

        t_last = time.time()

        self.__init__(self.drive_system, self.line_sensor, self.angle_sensor,
                      self.proximity_sensor, self.nfc_reader, self.electromagnet)

        while True:
            # get dt
            t_now = time.time()
            dt = t_now - t_last
            dt = min(dt, 0.005)
            t_last = t_now

            # read raw sensor values for current step
            raw_sensor_values = self.line_sensor.read_vals()

            # detect road side
            if (not self.is_correcting_offroad):
                self.road_side_value = self.road_side_detection(
                    raw_sensor_values, dt)

            # print(f"TEMP road side value: {self.road_side_value}")

            if (not self.blocked):
                # detect prize
                if (prize and self.prize_detection(raw_sensor_values, dt)):
                    self.drive_system.stop()
                    return Road_State.PRIZE

                # detect intersection
                if (self.intersection_detection(raw_sensor_values, dt)):
                    self.drive_system.stop()
                    return Road_State.INTERSECTION

                # detect road end
                if (self.road_end_detection(raw_sensor_values, dt)):
                    self.drive_system.stop()
                    return Road_State.END

            # move wheels according to raw sensor values
            self.respond_to_raw_sensor_values(raw_sensor_values)

    def road_side_detection(self, raw_sensor_values, dt):
        '''
        Detects road side when called every frame

        :param raw_sensor_values: raw sensor values read from current frame
        :param dt: time passed per frame
        '''

        # get raw road side value from current raw sensor readings
        raw_road_side_value = 0
        sensor_input = Sensor_Input(raw_sensor_values)
        match sensor_input:
            case Sensor_Input.SLIGHT_LEFT:
                raw_road_side_value = -.5
            case Sensor_Input.FAR_LEFT:
                raw_road_side_value = -1
            case Sensor_Input.SLIGHT_RIGHT:
                raw_road_side_value = .5
            case Sensor_Input.FAR_RIGHT:
                raw_road_side_value = 1

        # update road side estimator and value
        self.road_side_estimator = get_updated_estimator(
            self.road_side_estimator, raw_road_side_value, dt, self.road_side_time_constant)
        if (self.road_side_estimator > 1 - self.threshold):  # greater than 0.63
            return 1
        elif (self.road_side_estimator < self.threshold and 0 - self.threshold < self.road_side_estimator):  # between -0.37 and 0.37
            return 0
        elif (self.road_side_estimator < -1 + self.threshold):  # less than -0.63
            return -1

        return 0

    def prize_detection(self, raw_sensor_values, dt):
        '''
        Detects prizes when called every frame

        :param raw_sensor_values: raw sensor values read from current frame
        :param dt: time passed per frame
        '''
        # get raw intersection value from current raw sensor readings
        raw_prize_value = 0
        if (raw_sensor_values == (1, 0, 1)):
            raw_prize_value = 1

        # update road side estimator and value
        self.road_prize_time_estimator = get_updated_estimator(
            self.road_prize_time_estimator, raw_prize_value, dt, self.road_prize_time_constant)
        if (self.road_prize_time_estimator > 1 - self.threshold):
            return True

        return False

    def intersection_detection(self, raw_sensor_values, dt):
        '''
        Detects intersection when called every frame

        :param raw_sensor_values: raw sensor values read from current frame
        :param dt: time passed per frame
        '''

        # get raw intersection value from current raw sensor readings
        raw_intersection_value = 0
        if (raw_sensor_values == (1, 1, 1)):
            raw_intersection_value = 1

        # update road side estimator and value
        self.intersection_estimator = get_updated_estimator(
            self.intersection_estimator, raw_intersection_value, dt, self.intersection_time_constant)
        if (self.intersection_estimator > 1 - self.threshold):
            return True

        return False

    def road_end_detection(self, raw_sensor_values, dt, use_middle_only=False):
        '''
        Detects road end when called every frame

        :param raw_sensor_values: raw sensor values read from current frame
        :param dt: time passed per frame
        '''

        # get raw intersection value from current raw sensor readings
        raw_road_end_value = 0
        if (not use_middle_only):
            if (raw_sensor_values == (0, 0, 0)):
                raw_road_end_value = 1
        else:
            if (raw_sensor_values[1] == 0):
                raw_road_end_value = 1

        time_constant_multiplier = 1
        if (self.is_correcting_offroad):
            time_constant_multiplier = 5
        if (use_middle_only):
            time_constant_multiplier = 0.6

        # update road side estimator and value
        self.road_end_estimator = get_updated_estimator(
            self.road_end_estimator, raw_road_end_value, dt, self.road_end_time_constant * time_constant_multiplier)
        if (self.road_end_estimator > 1 - self.threshold):
            return True

        return False

    def respond_to_raw_sensor_values(self, raw_sensor_values):
        '''
        Adjusts wheel speeds according to sensor values from current frame

        :param raw_sensor_values: raw sensor values read from current frame
        '''
        # constant checking for blockages
        if (not self.blocked):
            if (self.read_front_blockage(average=False) < 10):
                self.drive_system.stop()
                self.blocked = True
                self.road_end_estimator = -1
                self.intersection_estimator = -1
                return
        else:
            if (self.read_front_blockage() > 15):
                self.blocked = False
                print("TEMP not blocked anymore")
            else:
                return

        if (raw_sensor_values != (0, 0, 0)):
            self.is_correcting_offroad = False

        sensor_input = Sensor_Input(raw_sensor_values)

        # adjust wheel speeds depending on raw sensor values
        match sensor_input:
            case Sensor_Input.ALL:
                self.drive_system.drive_enum(Steer_Style.STRAIGHT)

            case Sensor_Input.SLIGHT_LEFT:
                self.drive_system.drive_enum(Steer_Style.TURN_RIGHT)

            case Sensor_Input.FAR_LEFT:
                self.drive_system.drive_enum(Steer_Style.HOOK_RIGHT)

            case Sensor_Input.SLIGHT_RIGHT:
                self.drive_system.drive_enum(Steer_Style.TURN_LEFT)

            case Sensor_Input.FAR_RIGHT:
                self.drive_system.drive_enum(Steer_Style.HOOK_LEFT)

            case Sensor_Input.BRANCH:
                self.drive_system.drive_enum(Steer_Style.STRAIGHT)

            case Sensor_Input.MIDDLE:
                self.drive_system.drive_enum(Steer_Style.STRAIGHT)

            case Sensor_Input.OFF_ROAD:
                if (self.road_side_estimator > 0.25):
                    self.drive_system.drive_enum(
                        Steer_Style.HOOK_LEFT)  # previously in right
                    self.is_correcting_offroad = True
                elif (self.road_side_estimator < -0.25):
                    self.is_correcting_offroad = True
                    self.drive_system.drive_enum(Steer_Style.HOOK_RIGHT)
                else:
                    self.drive_system.drive_enum(Steer_Style.STRAIGHT)
                    self.is_correcting_offroad = False

    def turn(self, direction):
        '''
        Turns robot.

        :param direction: direction to turn. positive is right and negative is left
        :return: turn time
        '''

        # wait before turn
        wait_seconds(0.5)

        turn_boolean = 1
        turn_estimator = 1
        turn_time_constant = 0.09
        exact_turn_time_constant = 0.015

        # record turn start time to keep track of how long turn takes
        t_turn_start = time.time()
        t_last = time.time()

        # record angle before turn
        initial_angle = self.angle_sensor.read_avg_angle()

        # use leading sensor to sense getting off current road
        if (direction < 0):
            sensor = 0
            self.drive_system.drive_enum(
                Steer_Style.SPIN_RIGHT)
        else:
            sensor = 2
            self.drive_system.drive_enum(
                Steer_Style.SPIN_LEFT)

        # turn until robot exits current road, if on it
        while (turn_boolean == 1):
            t_now = time.time()
            dt = t_now - t_last
            t_last = t_now

            raw = self.line_sensor.read_vals()[sensor]

            turn_estimator = turn_estimator + dt / \
                (turn_time_constant/2) * (raw - turn_estimator)

            if (turn_estimator < self.threshold):
                turn_boolean = 0

        # turn until tobot reaches destination road
        while (turn_boolean == 0):
            t_now = time.time()
            dt = t_now - t_last
            t_last = t_now

            raw = self.line_sensor.read_vals()[sensor]

            turn_estimator = turn_estimator + dt / \
                turn_time_constant * (raw - turn_estimator)

            if (turn_estimator > 1 - self.threshold):
                turn_boolean = 1

        # stop turning
        self.drive_system.stop()

        # sleep before correcting
        wait_seconds(0.1)

        # record final angle
        final_angle = self.angle_sensor.read_avg_angle()

        # get and correct turn angle
        turn_angle = get_delta_angle(initial_angle, final_angle, direction)

        # record turn end time
        t_turn_end = time.time()

        # if turn angle was less than 45, automatically make 45
        if (t_turn_end - t_turn_start < 0.5 and turn_angle < 45):
            turn_angle += 45

        # if turn angle was more than 360, plus 360 to angle
        if (t_turn_end - t_turn_start >= 1.6 and turn_angle < 45):
            turn_angle += 360

        # if turn angle was big, correct robot until it reads middle (0, 1, 0)
        if (abs(turn_angle) > 10):
            exact_turn_estimator = 0
            exact_turn_boolean = False

            t_before_correct = time.time()
            t_now = time.time()
            while (not exact_turn_boolean and t_now - t_before_correct < 1):
                t_now = time.time()
                dt = t_now - t_last
                t_last = t_now

                raw_sensor_values = self.line_sensor.read_vals()
                sensor_input = Sensor_Input(raw_sensor_values).name

                raw_exact_turn_value = 0.1
                match sensor_input:
                    case "ALL" | "MIDDLE" | "BRANCH":
                        raw_exact_turn_value = 1
                    case "SLIGHT_LEFT" | "FAR_LEFT":
                        self.drive_system.drive_slow_enum(
                            Steer_Style.SPIN_RIGHT)
                    case "SLIGHT_RIGHT" | "FAR_RIGHT":
                        self.drive_system.drive_slow_enum(
                            Steer_Style.SPIN_LEFT)
                    case "OFF_ROAD":
                        self.drive_system.drive_slow_enum(
                            Steer_Style.SPIN_LEFT)

                exact_turn_estimator = get_updated_estimator(
                    exact_turn_estimator, raw_exact_turn_value, dt, exact_turn_time_constant)
                if (exact_turn_estimator > self.threshold):
                    exact_turn_boolean = True

            # record final angle
            final_angle = self.angle_sensor.read_avg_angle()

            # get and correct turn angle
            turn_angle = get_delta_angle(initial_angle, final_angle, direction)
            if (t_turn_end - t_turn_start >= 0.8 and turn_angle < 45):
                turn_angle = turn_angle + 360

        # stop turning
        self.drive_system.stop()

        # wait before exiting out of turn state
        wait_seconds(0.3)

        # return turn time
        return (t_turn_end - t_turn_start), turn_angle

    def pull_forward(self, duration):
        '''
        Pulls robot forward after it reaches an intersection.

        :return: True if road exists, False if not
        '''

        # sleep before pulling forward
        self.drive_system.stop()
        wait_seconds(0.2)

        self.drive_system.drive_enum(
            Steer_Style.STRAIGHT, speed_multiplier=1.05)

        # pull forward for duration seconds
        t0 = time.time()
        t_last = time.time()

        while True:
            # get dt
            t_now = time.time()
            dt = t_now - t_last
            t_last = t_now

            # read sensor values
            raw_sensor_values = self.line_sensor.read_vals()

            # detect road end
            off_road = self.road_end_detection(
                raw_sensor_values, dt, use_middle_only=True)

            if t_now - t0 >= duration:
                break
        self.drive_system.stop()
        return (not off_road)

    def pull_backward(self, duration):
        '''
        Pulls the robot backward for specified duration.

        :para, duration: how long to pull back
        '''
        self.drive_system.stop()
        wait_seconds(0.2)

        self.drive_system.drive_enum(Steer_Style.STRAIGHT, backwards=True)
        wait_seconds(duration)

        self.drive_system.stop()
        wait_seconds(0.1)

    def pull_backward_intersection(self):
        self.drive_system.stop()
        wait_seconds(0.2)

        self.intersection_estimator = 0

        t_last = time.time()
        while (True):
            # get dt
            t_now = time.time()
            dt = t_now - t_last
            dt = min(dt, 0.005)
            t_last = t_now

            # read raw sensor values for current step
            raw_sensor_values = self.line_sensor.read_vals()

            self.drive_system.drive_enum(Steer_Style.STRAIGHT, backwards=True)

            if (self.intersection_detection(raw_sensor_values, dt)):
                break

        self.drive_system.stop()
        wait_seconds(0.1)

    def tune_magnetometer(self, duration):
        '''
        Spins the robot and tunes magnetometer x and y bit min and maxes. 
        Plots the x and y bit values obtained while spinning.

        :param duration: time the robot will spin
        '''

        x_max = 0
        x_min = 255
        y_max = 0
        y_min = 255

        x_bit = []
        y_bit = []

        self.drive_system.drive_enum(Steer_Style.SPIN_RIGHT)

        t0 = time.time()
        while True:
            cur_x_bit = self.angle_sensor.read_adc(0)
            cur_y_bit = self.angle_sensor.read_adc(1)

            x_max = max(cur_x_bit, x_max)
            y_max = max(cur_y_bit, y_max)
            x_min = min(cur_x_bit, x_min)
            y_min = min(cur_y_bit, y_min)

            x_bit.append(cur_x_bit)
            y_bit.append(cur_y_bit)

            if time.time() - t0 > duration:
                self.drive_system.stop()
                break

        print(
            f"x min: {x_min}, x max: {x_max}, y min: {y_min}, y max: {y_max}\n")
        self.angle_sensor.x_bit_min = max(0, x_min)
        self.angle_sensor.y_bit_min = max(0, y_min)
        self.angle_sensor.x_bit_max = min(255, x_max)
        self.angle_sensor.y_bit_max = min(255, y_max)

        plt.plot(x_bit, y_bit)

        # Show the graph and continue
        plt.pause(0.001)

    def magnet_on(self):
        '''
        Robot turns magnet on.
        '''
        wait_seconds(0.2)
        print("Magnet on")
        self.electromagnet.on()

    def magnet_off(self):
        '''
        Robot turns magnet off.
        '''
        wait_seconds(0.2)
        print("Magnet off")
        self.electromagnet.off()

    def plot_ang_to_time(self, duration):
        '''
        Spins the robot then plots angle v. time obtained during spin. 

        :param duration: time the robot will spin
        '''

        self.drive_system.drive_enum(Steer_Style.SPIN_RIGHT)

        # check angle
        t0 = time.time()

        t = []
        angle = []

        while True:

            angle.append(self.angle_sensor.read_avg_angle())
            t.append(time.time())
            if time.time() - t0 > duration:
                self.drive_system.stop()
                break
        plt.plot(t, angle)

        # Show the graph and continue
        plt.pause(0.001)

    def read_front_blockage(self, average=True):
        '''
        Reads the front ultrasonic sensor.

        :param average: whether or not to average 5 sensor readings
        :return: the front ultrasonic sensor reading (averaged if True)
        '''

        readings = []

        if (not average):
            return self.proximity_sensor.read_vals()[1]

        while (len(readings) < 5):
            readings.append(self.proximity_sensor.read_vals()[1])
            wait_seconds(0.07)

        return np.median(readings)

    def get_nfc_reading(self):
        return self.nfc_reader.read()
