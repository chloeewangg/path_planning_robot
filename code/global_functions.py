'''
!/usr/bin/env python3

global_functions.py
Contains all helper functions.
'''

# Imports
import time
from global_constants import *
from dataclasses import dataclass

@dataclass
class Position:
    x: int = 0
    y: int = 0
    heading: int = 0

def clamp(value, min_value, max_value):
    '''
    Clamps value between min and max

    :param value: value to clamp
    :param min_value: the minimum value value can be 
    :param max_value: the maximum value value can be
    '''
    return max(min_value, min(value, max_value))

def wait_seconds(duration):
    '''
    Makes program wait for duration seconds.

    :param duration: number of seconds program should wait
    '''
    t0 = time.time()
    while True:
        t = time.time()
        if t - t0 >= duration:
            break

def get_changed_tuple(index, new_value, tuple):
    '''
    Returns a tuple with one value changed. Essentially to "modify" a tuple

    :param index: index of the value to change
    :param new_value: the new value of the changing index
    :param tuple: the original tuple
    :return: returns the modified tuple
    '''
    if(index == 0):
        return (new_value, tuple[1], tuple[2])
    elif(index == 1):
        return (tuple[0], new_value, tuple[2])
    else:
        return (tuple[0], tuple[1], new_value)

def get_updated_estimator(cur_estimator, raw_value, dt, time_constant):
    '''
    updates estimator by applying estimator updating equation.

    :param cur_estimator: the estimator value before updating
    :param raw_value: the raw value obtained in current step
    :param dt: the time passed from the last step
    :param time_constant: the time constant
    '''
    return cur_estimator + min(dt/time_constant, 1) * (raw_value - cur_estimator)

def map_values(value, min_input, max_input, min_output, max_output):
    '''
    Maps a value of an input range to a value of the output range. 

    :param value: value of interest
    :param min_input: minimum of input range
    :param max_input: maximum of input range
    :param min_output: minimum of output range
    :param max_output: maximum of output range
    :return: the new mapped value
    '''
    input_range = max_input*1.0 - min_input
    output_range = max_output*1.0 - min_output
    return ((1.0*value - min_input) / input_range * output_range + min_output)

def get_delta_angle(initial_angle, final_angle, direction=0):
    '''
    Gets the delta angle from initial to final angle, considering the direction.

    :param initial_angle: the initial angle
    :param final_angle: the final angle
    :param direction: the direction of interest; if 0, gets smaller delta angle in either direction
    :return: the delta angle
    '''
    global LEFT
    global RIGHT

    delta_angle = final_angle - initial_angle

    if(direction != 0):
        if(delta_angle * direction < 0):
            delta_angle = direction * delta_angle + 360
        return abs(delta_angle)
    else:
        return min(get_delta_angle(initial_angle, final_angle, LEFT),
                   get_delta_angle(initial_angle, final_angle, RIGHT))

def get_delta_heading(initial_heading, final_heading, direction):
    '''
    Gets the delta heading from initial to final heading, considering the direction.
    If direction is 0, get smaller delta heading in either direction.

    :param initial_heading: the initial heading
    :param final_heading: the final heading
    :param direction: the direction of interest
    :return: the delta heading
    '''

    global LEFT
    global RIGHT

    if(direction == 0):
        return min(get_delta_heading(initial_heading, final_heading, LEFT), 
                   get_delta_heading(initial_heading, final_heading, RIGHT))

    delta_heading = final_heading - initial_heading
    if(delta_heading * direction < 0):
        delta_heading = direction * delta_heading + 8
    return abs(delta_heading)

def get_user_input(avail_input, input_msg, wrong_input_msg):
    '''
    Gets a valid user input by reprompting the input when invalid.

    :param avail_input: the array of available inputs
    :param input_msg: the string message to say when asking for input
    :param wront_input_message: the string message to say when input is invalid
    :return: a valid user input inside avail_input
    '''
    user_input = input(input_msg)
    while(user_input not in avail_input):
        print(wrong_input_msg)
        user_input = input(input_msg)
    return user_input

def threshold_map(thresholds, value):
    '''
    When given an array indicating ranges, determines which range the specified value falls under.

    :param thresholds: array of numbers representing ranges
    :param value: the value of interest
    :return: the index number of the range the value falls under; when given [i, j, k, l] as
    thresholds and x as value, a return value of 2 means j(thresholds[1]) <= x < k(thresholds[2])
    '''
    
    for i in range(len(thresholds)):
            if(value < thresholds[i]):
                return i
            
    return len(thresholds)

def check_warning(condition, warning):
    '''
    Raises a warning message when a condition is met

    :param condition: the condition to check
    :param warning: the warning message to print
    :return: whether the condition is True or False
    '''
    if(condition):
        print(warning)
    return condition

def weighted_average(values, weights):
    '''
    Computes uses a weighted average utilizing a list a values and a list of corresponding weights. This function assumes that the list of weights sums to 
    
    :param values: the values to loop through to find average of
    :param weights: associated weights with each entry of values
    :return: the weighted average
    '''
    weighted_sum = 0.0
    sum_of_weights = 0.0
    for i in range(len(values)):
        weighted_sum += values[i] * weights[i]
        sum_of_weights = sum_of_weights + weights[i]
    
    return weighted_sum / sum_of_weights