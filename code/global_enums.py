'''
!/usr/bin/env python3

global_enums.py
Contains all helper enums.
'''

# Imports
from enum import Enum

class Steer_Style(Enum):
    """
    ENUM list of states with corresponding motor PWMS.
    """
    
    STRAIGHT = (190, 190) #0 done

    VEER_LEFT = (177, 203) #-45 done
    STEER_LEFT = (165, 223) #-90 done
    TURN_LEFT = (133, 231) #-180 done
    HOOK_LEFT = (0, 216) #-400 done
    SPIN_LEFT = (-225, 210) #-540 done

    VEER_RIGHT = (200, 177) #45 done
    STEER_RIGHT = (213, 175) #90 done
    TURN_RIGHT = (220, 129) #180 done
    HOOK_RIGHT = (209, 0) #400 done
    SPIN_RIGHT = (180, -231) #540
    
    """
    STRAIGHT = (174, 200) #0 done

    VEER_LEFT = (165, 200) #-45 done
    STEER_LEFT = (155, 215) #-90 done
    TURN_LEFT = (130, 230) #-180 done
    HOOK_LEFT = (0, 220) #-400 done
    SPIN_LEFT = (-220, 185) #-540 done

    VEER_RIGHT = (185, 190) #45 done
    STEER_RIGHT = (190, 165) #90 done
    TURN_RIGHT = (205, 140) #180 done
    HOOK_RIGHT = (205, 0) #400 done
    SPIN_RIGHT = (180, -210) #540
    """

class Sensor_Input(Enum):
    """
    ENUM list of states with corresponding line_sensor states.
    """
    OFF_ROAD = (0, 0, 0)
    ALL = (1, 1, 1)
    SLIGHT_RIGHT = (0, 1, 1)
    FAR_RIGHT = (0, 0, 1)
    SLIGHT_LEFT = (1, 1, 0)
    FAR_LEFT = (1, 0, 0)
    BRANCH = (1, 0, 1)
    MIDDLE = (0, 1, 0)

class Robot_State(Enum):
    """
    ENUM list of states of the robot.
    """

    STRAIGHT = 0
    TURN_LEFT = 1
    TURN_RIGHT = 2
    PULL_FORWARD = 3
    ROAD_END = 4
    INTERSECTION = 5

class Road_State(Enum):
    """
    ENUM list of states of the road.
    """

    END = 0
    INTERSECTION = 1
    PRIZE = 2

class Street_Status(Enum):
    """
    ENUM list of statuses of the road.
    """

    UNKNOWN = 0
    NONEXISTENT = 1
    UNEXPLORED = 2
    DEADEND = 3
    CONNECTED = 4

class Mapping_Status(Enum):
    """
    ENUM list of how to plan the next move.
    """

    INPUT = 0
    MAPPING = 1
    AUTO = 2


class Driving_Mode(Enum):
    """
    ENUM list of how to plan the next move.
    """

    NONE = -1
    INPUT = 0
    EXPLORE = 1
    GOAL = 2
    FETCH = 3

class Command(Enum):
    """
    ENUM list of all user UI commands.
    """

    EXPLORE = "E"
    GOAL = "G"
    FETCH = "F"
    PAUSE = "PA"
    STEP = "SP"
    RESUME = "R"
    LEFT = "LT"
    RIGHT = "RT"
    STRAIGHT = "S"
    SAVE = "SV"
    LOAD = "LO"
    CLEAR = "CL"
    POSE = "P"
    SHOW = "SH"
    CLEAR_BLOCKAGE = "CB"
    QUIT = "Q"
    NONE = " "

class Move(Enum):
    """
    ENUM list of inputs for behaviors.
    """
    
    STRAIGHT = "S"
    TURN_LEFT = "L"
    TURN_RIGHT = "R"
    PICKUP_PRIZE = "P"
    BLOCKED = "B"
    PASS = "P"
    QUIT = "Q"
    NONE = " "


class Distance(Enum):
    """
    ENUM list of distances for ultrasound readings.
    """

    NEAR = 0
    MIDDLE = 10
    FAR = 20