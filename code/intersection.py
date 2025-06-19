'''
!/usr/bin/env python3

intersection.py
Contains class for maintaining each intersection
'''

#Imports
import time
import math
from global_functions import *
from global_enums import *

class Intersection():
    '''
    Makes intersection object for each intersection.
    '''
    def __init__(self, x, y):
        '''
        Initializes variables
        '''
        global NUM_HEADINGS
        
        self.location = (x, y) # coordinates of the intersection
        self.streets = [Street_Status.UNKNOWN] * 8
        self.blocked = [False] * 8

        # pathfinding variables
        self.optimal_direction = None # optimal direction to go to reach goal
        self.cost = math.inf # cost to reach goal following optimal path
        self.aim_location = (math.inf, math.inf)

        # prize-fetching variables
        self.prize_distances = {i:math.inf for i in range(1, 19)}

    def is_street_existent(self, heading, get_unknown=False):
        '''
        Returns a boolean representing whether or not a heading exists.

        :param heading: Heading to determine if exists
        :get_unknown: Boolean representing whether or not to consider an unknown street as existent
        '''
        return (self.streets[heading] == Street_Status.CONNECTED
                or self.streets[heading] == Street_Status.DEADEND
                or self.streets[heading] == Street_Status.UNEXPLORED
                or (self.streets[heading] == Street_Status.UNKNOWN and get_unknown))
    
    def get_heading(self, location):
        '''
        Returns headings to a specified location. 

        :param location: location to get the heading of
        :return: heading towards that location; None if location is not one step away 
        '''

        global heading_dict

        dir = (location[0] - self.location[0], location[1] - self.location[1])
        if(dir not in heading_dict.values()):
            return None
        
        for heading in heading_dict:
            if heading_dict[heading][0] == dir[0] and heading_dict[heading][1] == dir[1]:
                return heading
        
        return None
    
    def is_all_blocked(self):
        for heading in range(0, 8):
            if(self.streets[heading] == Street_Status.UNKNOWN):
                return False
            
            if(self.streets[heading] == Street_Status.NONEXISTENT):
                continue
            
            if(not self.blocked[heading]):
                return False
            
        return True

    def get_neighbor_headings(self, skip_headings=[], get_blocked=False, get_unknown=False):
        '''
        Returns headings of all existing streets

        :param skip_headings: array of headings to skip
        :param get_blocked: True/False; whether to get blocked streets or not
        :param get_unknown: True/False; whether to get unknown streets or not
        '''
        neighbor_headings = []

        for heading in range(0, 8):
            if(heading in skip_headings):
                continue
            if(self.is_street_existent(heading, get_unknown=get_unknown) 
               and (get_blocked or not self.blocked[heading])):
                neighbor_headings.append(heading)

        return neighbor_headings
    
    def get_connected_neighbor_headings(self, get_blocked=False):
        '''
        Returns headings of all existing streets that are connected to another intersection

        :param get_blocked: True/False; whether to get blocked streets or not
        :return: an array of headings of the neighbors
        '''
        neighbor_headings = []

        for heading in range(0, 8):
            if(self.streets[heading] == Street_Status.CONNECTED 
               and (get_blocked or not self.blocked[heading])):
                neighbor_headings.append(heading)

        return neighbor_headings

    def get_undiscovered_streets(self, get_blocked=False):
        '''
        Returns headings of all existing streets that are connected to another intersection

        :param get_blocked: True/False; whether to get blocked streets or not
        '''
        neighbor_headings = []

        for heading in range(0, 8):
            if((self.streets[heading] == Street_Status.UNEXPLORED or
                self.streets[heading] == Street_Status.UNKNOWN)
                and (get_blocked or not self.blocked[heading])):
                neighbor_headings.append(heading)

        return neighbor_headings
    
    def get_street_with_status(self, street_status, start_heading=0, skip_start=False, skip_blocked=False):
        '''
        Returns the nearest street from the start heading with specified street status
    
        :param street_status: status of interest
        :param start_heading: heading to start from; default to 0
        :param skip_start: True/False; whether to skip the start heading
        :param skip_blocked: True/False; whether to skip blocked streets
        :return: heading number of street with status closest to start heading; None if not found
        '''
        for i in range(0, 8):
            if(i == 0 and skip_start):
                continue

            right_heading = (start_heading - i) % 8
            if(self.streets[right_heading] == street_status):
                if(not(skip_blocked and self.blocked[right_heading])):
                    return right_heading
                
            left_heading = (start_heading + i) % 8
            if(self.streets[left_heading] == street_status):
                if(not(skip_blocked and self.blocked[left_heading])):
                    return left_heading

        return None

    def is_complete(self, blocked_is_complete=False):
        '''
        Checks whether intersection is complete (no streets UNKNOWN or UNEXPLORED)

        :param blocked_is_complete: whether to consider blocked streets as complete
        :return: True or False; whether intersection is complete
        '''

        for ind, street_status in enumerate(self.streets):
            if(blocked_is_complete and self.blocked[ind]):
                continue
            if(street_status == Street_Status.UNKNOWN or street_status == Street_Status.UNEXPLORED):
                return False
        
        return True
    
    def has_status(self, status, consider_blocked=True):
        '''
        Checks whether intersection has streets of specified status left

        :param status: the status to compare to
        :param consider_blocked: whether to consider blocked streets
        :return: True or False; whether intersection has unknown streets
        '''

        for ind, street_status in enumerate(self.streets):
            if(not consider_blocked and self.blocked[ind]):
                continue
            if(street_status == status):
                return True
        
        return False


