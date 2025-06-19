'''
!/usr/bin/env python3

global_constants.py
Contains all constants shared across files.
'''

MAP_MIN = (-10, -10)
MAP_MAX = (10, 10)

NUM_HEADINGS = 8
HALF_NUM_HEADINGS = 4

LEFT = 1
RIGHT = -1
BOTH = 0

heading_dict = {
        0: (0, 1), 
        1: (-1, 1), 
        2: (-1, 0), 
        3: (-1, -1), 
        4: (0, -1), 
        5: (1, -1),
        6: (1, 0), 
        7: (1, 1)
        }   

STRAIGHT_COST = 1
DIAGONAL_COST = 1.4
