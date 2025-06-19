'''
!/usr/bin/env python3

shared.py
Keeps track of all shared flags across multiple threads.
'''

from global_imports import *

class Shared:
    def __init__(self):
        '''
        Initializes all variables.
        '''

        self.lock = threading.Lock()

        self.cmd = Command.NONE # the most recent command from the user

        self.paused = False
        self.quit = False

        self.robot_pos = Position(0, 0, 0)
        self.robot_pos_input = Position(0, 0, 0)
        self.goal = (0, 0)
        
        self.cur_prize = None
        self.prize = None
        self.prize_info_dict = dict()
        self.prize_dist_dict = dict()

        self.map_name = ""
        
        self.proximity_mode = 0


    
    def acquire(self):
        '''
        Method to acquire access to the shared data.
        '''

        return self.lock.acquire()
    
    def release(self):
        '''
        Method to release access to the shared data.
        '''

        return self.lock.release()


