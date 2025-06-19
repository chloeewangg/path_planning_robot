'''
!/usr/bin/env python3

brain.py
Responsible for the brain of the robot.
'''

# Imports
from global_imports import *
from shared import *
from intersection import *
from map import *
import matplotlib.pyplot as plt
import board
import busio
from adafruit_pn532.i2c import PN532_I2C


class Brain():
    '''
    Brain of the robot. Responsible for all decision making and executing when following line.
    '''
    global heading_dict

    global RIGHT
    global LEFT

    global NUM_HEADINGS
    global HALF_NUM_HEADINGS

    global MAP_MIN
    global MAP_MAX

    def __init__(self, shared, behavior):
        '''
        Initializes all variables.

        :param behavior: object responsible for all robot's behaviors
        '''
        self.shared = shared
        self.shared_copy = None
        self.behavior = behavior

        self.map = Map()
        self.position = Position(0, 0, 0)  # (x, y, heading)
        self.prev_intersection = None  # object of the previous intersection
        self.cur_intersection = None

        self.home = Position(0, 0, 0)

        self.goal = None
        self.goal_heading = None

        self.wait_time = 0
        self.max_wait_time = 5

        self.prizes = []
        self.prize_values = {}
        self.cur_prize = 1  # the current prize
        self.blocked_prize = None
        self.has_prize = False
        self.prize_info_dict = {}
        self.prize_dist_dict = {}

        self.prev_prize_distances = {}

        self.driving_mode = Driving_Mode.NONE

        self.pull_forward_time = 0.7
        self.pull_forward_time_start = 0.71  # has less momentum so pull forward less

        self.pull_backward_time = 1.2

        self.prize_ratio_dict = dict()

    def robot_start(self):
        '''
        Called at the very start. Will initialize the robot.
        '''

        # tune magnetometer x, y bounds
        self.behavior.tune_magnetometer(3)

        # get nfc dictionary
        if (self.shared.acquire()):
            self.prize_info_dict = self.shared.prize_info_dict
            self.prize_dist_dict = self.shared.prize_dist_dict
            self.shared.release()
        self.prizes = self.prize_info_dict.keys()
        for prize in self.prizes:
            self.prize_ratio_dict[prize] = math.inf

        # turn until robot enters line again
        self.robot_turn(LEFT)

        # follow line until robot goes to start intersection
        self.robot_straight_start()

        self.robot_switch_to_input()

    def run_robot(self):
        '''
        Makes the robot follow and map out the grid. 

        :param self.behavior: object containing all self.behaviors of robot
        '''

        running = True
        while running:
            # plot the map with the robot's current position
            self.map.plot_map(self.position)
            # self.map.plot_map(self.position, True)

            if (self.shared.acquire()):
                # safe quit when quit
                if (self.shared.quit):
                    running = False

                # send robot's position to shared
                self.shared.robot_pos.x = self.position.x
                self.shared.robot_pos.y = self.position.y
                self.shared.robot_pos.heading = self.position.heading

                # execute map commands first to avoid UI conflicts
                self.execute_map_command()

                # create a copy of shared data
                self.shared_copy = Shared()
                self.shared_copy.cmd = self.shared.cmd
                self.shared_copy.paused = self.shared.paused
                self.shared_copy.quit = self.shared.quit
                self.shared_copy.robot_pos_input = copy.deepcopy(
                    self.shared.robot_pos_input)
                self.shared_copy.goal = copy.deepcopy(self.shared.goal)
                self.shared_copy.cur_prize = self.shared.cur_prize
                self.shared_copy.map_name = self.shared.map_name
                self.shared_copy.proximity_mode = self.shared.proximity_mode

                self.shared.release()

            # plan move
            move = self.plan_move()

            # execute planned move
            self.execute_move(move)

    def execute_map_command(self):
        '''
        Executes a command related to the map. To be called before plan_move, inside
        shared.acquire() to avoid UI conflict.
        '''

        match self.shared.cmd:
            case Command.SAVE:
                self.save_map(self.shared.map_name)
                self.shared.cmd = Command.NONE
            case Command.LOAD:
                self.load_map(self.shared.map_name)
                self.shared.cmd = Command.NONE
                self.shared.loading_map = False
            case Command.CLEAR:
                self.clear_map()
                self.shared.cmd = Command.NONE

    def plan_move(self):
        '''
        Plans the next move based on input or pathfinding algorithm.

        :return: the planned move
        '''

        return_move = None  # the move to return

        match self.shared_copy.cmd:
            case Command.EXPLORE:
                if (self.driving_mode != Driving_Mode.EXPLORE):
                    self.robot_switch_to_explore()
            case Command.GOAL:
                self.set_goal_from_shared()
                self.robot_switch_to_goal()
            case Command.FETCH:
                if (self.driving_mode != Driving_Mode.FETCH):
                    self.robot_switch_to_fetch()
            case Command.PAUSE:
                return_move = Move.NONE
            case Command.STEP:
                if (self.shared_copy.paused):
                    if (self.driving_mode == Driving_Mode.EXPLORE):
                        return_move = self.plan_move_explore()
                    elif (self.driving_mode == Driving_Mode.GOAL):
                        return_move = self.plan_move_goal()
                    elif (self.driving_mode == Driving_Mode.FETCH):
                        return_move = self.plan_move_fetch()
                    else:
                        print("[WARNING] No previous autonomous action to step! \n")
                        return_move = Move.NONE
                else:
                    print("[WARNING] Can only run step command when paused! \n")
                    return_move = Move.NONE
            case Command.RESUME:
                pass
            case Command.LEFT:
                return_move = Move.TURN_LEFT
            case Command.RIGHT:
                return_move = Move.TURN_RIGHT
            case Command.STRAIGHT:
                return_move = Move.STRAIGHT
            case Command.POSE:
                x = self.shared_copy.robot_pos_input.x
                y = self.shared_copy.robot_pos_input.y
                heading = self.shared_copy.robot_pos_input.heading
                self.set_pos(x, y, heading)
            case Command.SHOW:
                self.map.plot_map(self.position)
            case Command.CLEAR_BLOCKAGE:
                self.map.clear_blocked()
            case Command.QUIT:
                return_move = Move.QUIT

        if (self.shared.acquire()):
            self.shared.cmd = Command.NONE
            self.shared.release()

        if (return_move == None and not self.shared_copy.paused):
            match self.driving_mode:
                case Driving_Mode.EXPLORE:
                    return_move = self.plan_move_explore()
                case Driving_Mode.GOAL:
                    return_move = self.plan_move_goal()
                case Driving_Mode.FETCH:
                    return_move = self.plan_move_fetch()

        if (return_move == None):
            return_move = Move.NONE

        return return_move

    def plan_move_explore(self):
        '''
        Plans the next move based on how to fill the map.

        :return: the planned move
        '''
        # set the current intersection to the current intersection
        self.cur_intersection = self.map.get_intersection(
            self.position.x, self.position.y)

        # if map complete, ending process
        if (self.map.is_complete()):
            print("[NOTICE] Map is fully explored. Ending process \n")
            self.robot_switch_to_input()
            return Move.NONE

        # if non-blocked part of the map is complte, clear blockage and try again
        if (self.map.is_complete(blocked_is_complete=True)):
            print("[WARNING] Explored all non-blocked parts. Ending process.\n")
            self.robot_switch_to_input()
            return Move.NONE

        # if current intersection has unknown street, turn in that direction
        if (self.cur_intersection.has_status(Street_Status.UNKNOWN)):
            return self.plan_move_discover_unknown()

        # if current intersection has unexplored street, go to that
        if (self.cur_intersection.has_status(Street_Status.UNEXPLORED, consider_blocked=False)):
            unexplored_street = self.cur_intersection.get_street_with_status(
                Street_Status.UNEXPLORED, self.position.heading, skip_blocked=True)
            check_warning(unexplored_street == None,
                          "[WARNING]: Attempting to get unexplored street heading when there is none.")
            return self.plan_move_aim_heading(unexplored_street)

        # if no unexplored streets, set the nearest incomplete intersection as the goal
        new_goal = self.map.get_closest_incomplete(
            self.position.x, self.position.y)
        if (new_goal != None):
            self.set_goal(new_goal[0], new_goal[1])
            return self.plan_move_aim_heading(self.cur_intersection.optimal_direction)

        # all streets in current intersection is blocked
        self.update_blockage()
        wait_seconds(0.5)
        return Move.NONE

    def plan_move_goal(self):
        '''
        Plans the next move based on a precomputed pathfinding to the goal.
        Called by plan_move_mapping and plan_move_auto.

        :param set_goal: whether to Dijkstra this turn; set to False for recursion
        :return: the planned move
        '''

        # set the current intersection to the current intersection
        self.cur_intersection = self.map.get_intersection(
            self.position.x, self.position.y)

        # if goal reached, end process
        if (self.position.x == self.goal[0] and self.position.y == self.goal[1]):
            print("[NOTICE] Goal reached. Ending process.\n")

            self.robot_switch_to_input()
            return Move.NONE

        # if goal undiscovered but map complete, goal is unreachable
        if (self.map.is_complete()):
            print("[WARNING] Goal is unreachable. Ending process.\n")

            self.robot_switch_to_input()
            return Move.NONE

        # if all neighboring streets are blocked, wait
        move_all_blocked = self.plan_move_all_blocked()
        if (move_all_blocked != Move.PASS):

            print(f"all streets blocked")
            #self.map.clear_blocked()
            return move_all_blocked

        # if goal unreachable due to blockages, clear blockages and try again
        if (self.map.is_complete(blocked_is_complete=True)):
            print(
                "[WARNING] Goal is unreachable due to blockages. Clearing blockages and trying again.\n")

            self.map.clear_blocked()

        # Dijkstra to get optimal direction
        self.set_goal(self.goal[0], self.goal[1])
        optimal_direction = self.cur_intersection.optimal_direction

        # if the exact path to the goal isn't sure (either undiscovered or blocked)
        if (self.cur_intersection.aim_location != self.goal):

            # if current intersection has unknown street, turn in that direction
            if (self.cur_intersection.has_status(Street_Status.UNKNOWN)):
                return self.plan_move_discover_unknown()

        # has reached optimal undiscovered intersection
        if (optimal_direction == None):

            unexplored_street = self.get_unexplored_to_goal()
            return self.plan_move_aim_heading(unexplored_street)

        # aim to reach optimal undiscovered intersection
        else:

            return self.plan_move_aim_heading(optimal_direction)

    def plan_move_fetch(self):
        '''
        Calculates the next move to fetch the robot's current prize.
        '''

        # If robot has a prize, go home to drop it off
        if (self.has_prize):
            if (self.is_at_location(self.home.x, self.home.y)):
                self.drop_prize()
                self.prize_ratio_dict[self.cur_prize] = self.prize_info_dict[self.cur_prize]["value"] / \
                    self.map.get_intersection(
                        self.home.x, self.home.y).prize_distances[self.cur_prize]
        
                self.robot_switch_to_input()
                return Move.NONE
            self.set_goal(self.home.x, self.home.y)
            return self.plan_move_goal()

        # If robot passed the prize, go back and pick up the prize
        if ((self.prev_intersection != None)
                and (self.cur_intersection.prize_distances[self.cur_prize] == 0.5 and self.prev_intersection.prize_distances[self.cur_prize] == 0.5)):
            optimal_direction = self.cur_intersection.get_heading(
                self.prev_intersection.location)

            # turn until facing prize
            if (self.position.heading != optimal_direction):
                return self.plan_move_aim_heading(optimal_direction)

            # if prize intersection is blocked
            if (self.update_blockage()):
                wait_seconds(0.2)
                return Move.NONE

            # if not, go straight and pick up the prize
            self.wait_time = 0
            self.robot_straight(prize=True)

        # if all neighboring streets are blocked, wait
        move_all_blocked = self.plan_move_all_blocked()
        if (move_all_blocked != Move.PASS):
            print(f"all streets blocked")
            return move_all_blocked

        # always go to the intersection closest to the prize
        closest_intersection = self.map.get_closest_to_prize(
            self.cur_prize, self.cur_intersection)

        # if there are no reachable undiscovered, clear blockage and try again
        if (closest_intersection == None):
            print(
                f"[NOTICE] All undiscovered intersections are blocked. Clearing blockages and trying again.")
            self.map.clear_blocked()
            return self.plan_move_fetch()

        # if not at intersection closest to prize, go there
        if (not self.is_at_location(closest_intersection.location[0], closest_intersection.location[1])):
            self.set_goal(
                closest_intersection.location[0], closest_intersection.location[1])
            return self.plan_move_goal()

        # Turn until there are no unknown streets in intersection
        if (self.cur_intersection.has_status(Street_Status.UNKNOWN)):
            return self.plan_move_discover_unknown()

        # look at neighbors to see if any of it have 0.5 heading, and go to it if there is
        neighbors = self.map.get_intersection_neighbors(
            self.cur_intersection, get_blocked=True)
        for neighbor in neighbors:
            if (neighbor.prize_distances[self.cur_prize] == STRAIGHT_COST / 2):
                prize_heading = self.cur_intersection.get_heading(
                    neighbor.location)

                # turn until facing prize
                if (self.position.heading != prize_heading):
                    return self.plan_move_aim_heading(prize_heading)

                # if not blocked, go straight
                if (not self.update_blockage()):
                    return Move.STRAIGHT

                # if blocked, wait until blockage is cleared
                wait_seconds(0.1)
                return Move.NONE

        # discover undiscovered streets
        optimal_direction = self.cur_intersection.get_street_with_status(
            Street_Status.UNEXPLORED, self.position.heading, skip_blocked=True)

        # all undiscovered streets are blocked
        if (optimal_direction == None):
            # get closest blocked undiscovered and turn until facing there
            blocked_direction = self.cur_intersection.get_street_with_status(
                Street_Status.UNEXPLORED, self.position.heading, skip_blocked=False)
            if (self.position.heading != blocked_direction):
                return self.plan_move_aim_heading(blocked_direction)
            if (self.update_blockage()):
                wait_seconds(0.1)
                return Move.NONE

        return self.plan_move_aim_heading(optimal_direction)

    def plan_move_discover_unknown(self):
        '''
        Makes the robot turn in the intersection until all streets are known.
        '''

        unknown_street_heading = self.cur_intersection.get_street_with_status(
            Street_Status.UNKNOWN, self.position.heading)
        if (unknown_street_heading != None):
            optimal_turn_direction = self.get_optimal_turn_direction(self.position.heading,
                                                                     unknown_street_heading)
            if optimal_turn_direction == RIGHT:
                return Move.TURN_RIGHT
            else:
                return Move.TURN_LEFT

        return Move.NONE

    def plan_move_aim_heading(self, aim_heading, wait_blocked=False):
        '''
        Plans the next move based on the aim heading. 

        :param aim_heading: the aim heading
        '''

        if (aim_heading == None):
            if (wait_blocked):
                return self.plan_move_wait_blocked(self.max_wait_time)
            return Move.TURN_RIGHT

        # if not facing optimal direction, turn
        if self.position.heading != aim_heading:
            optimal_turn_direction = self.get_optimal_turn_direction(self.position.heading,
                                                                     aim_heading)

            # figure if turning left or right would be better
            if optimal_turn_direction == RIGHT:
                return Move.TURN_RIGHT
            else:
                return Move.TURN_LEFT

        if (wait_blocked):
            return self.plan_move_wait_blocked(self.max_wait_time)

        if self.update_blockage():
            return Move.TURN_RIGHT

        return Move.STRAIGHT

    def plan_move_wait_blocked(self, max_wait_time):
        '''
        Wait max_wait_time until blockage is cleared
        '''

        if self.update_blockage():
            if (self.wait_time < max_wait_time):
                wait_seconds(0.2)
                self.wait_time += 0.2
                return Move.NONE
            self.wait_time = 0
            return Move.BLOCKED
        return Move.STRAIGHT

    def plan_move_all_blocked(self):
        '''
        If streets around the robot are all blocked, turn towards the street the robot came from and wait until blockage is cleared
        '''

        if (not self.cur_intersection.is_all_blocked()):
            return Move.PASS

        if (self.update_blockage()):
            if (self.prev_intersection == None or self.prev_intersection == self.cur_intersection):
                wait_seconds(0.1)
                return Move.NONE

        # turn towards street robot came from
        prev_heading = self.cur_intersection.get_heading(
            (self.prev_intersection.location[0], self.prev_intersection.location[1]))
        if (self.position.heading != prev_heading):
            return self.plan_move_aim_heading(prev_heading)

        # wait until not blocked anymore
        if (self.update_blockage()):
            wait_seconds(0.1)
            return Move.NONE

        return Move.PASS

    def execute_move(self, move):
        '''
        Executes a move.

        :param move: the move to execute
        '''

        match move:
            case Move.STRAIGHT:
                self.robot_straight()
            case Move.TURN_LEFT:
                self.robot_turn(LEFT)
            case Move.TURN_RIGHT:
                self.robot_turn(RIGHT)
            case Move.PICKUP_PRIZE:
                self.get_prize()

    def ask_pos(self, is_start=False):
        '''
        gets user input for robot start position and direction
        '''

        heading = int(get_user_input([f"{i}" for i in range(0, NUM_HEADINGS)],
                                     "Enter the robot's start direction (0-7): ",
                                     "Invalid heading. Enter again."))
        x = int(get_user_input([f"{i}" for i in range(MAP_MIN[0], MAP_MAX[0]+1)],
                               "Enter the x coordinate of the robot's start position: ",
                               "Invalid x coordinate. Enter again."))
        y = int(get_user_input([f"{i}" for i in range(MAP_MIN[1], MAP_MAX[1]+1)],
                               "Enter the y coordinate of the robot's start position: ",
                               "Invalid y coordinate. Enter again."))
        self.set_pos(x, y, heading, is_start=is_start)

        home_heading = int(get_user_input([f"{i}" for i in range(0, NUM_HEADINGS)],
                                     "Enter the home direction (0-7): ",
                                     "Invalid heading. Enter again."))
        home_x = int(get_user_input([f"{i}" for i in range(MAP_MIN[0], MAP_MAX[0]+1)],
                               "Enter the x coordinate of the home: ",
                               "Invalid x coordinate. Enter again."))
        home_y = int(get_user_input([f"{i}" for i in range(MAP_MIN[1], MAP_MAX[1]+1)],
                               "Enter the y coordinate of the home: ",
                               "Invalid y coordinate. Enter again."))
        
        self.home = Position(home_x, home_y, home_heading)

    def set_pos(self, x, y, heading, is_start=False):
        '''
        Sets the robot's position to a specified position.

        :param x: the new x coordinate of the robot
        :param y: the new y coordinate of the robot
        :param heading: the new heading of the robot
        '''

        self.position = Position(x, y, heading)  # (x, y, heading)
        self.cur_intersection = self.map.get_intersection(
            self.position.x, self.position.y)

        wait_seconds(0.1)  # make sure to give time for nfc to read again

        if (not is_start):
            self.behavior.pull_backward(self.pull_backward_time)
            self.update_prize_distances()
            self.robot_smart_pull_forward(self.pull_forward_time)

        if (len(self.prev_prize_distances) > 0):
            self.cur_intersection.prize_distance = self.prev_prize_distances

        for prize in self.prizes:
            self.prize_ratio_dict[prize] = math.inf

    def set_home(self):
        # set home to the intersection the robot started at
        self.home = copy.deepcopy(self.position)

    def set_goal(self, goal_x, goal_y):
        '''
        Clears any previous tree and sets the goal of the robot

        :param goal_x: x coordinate of goal
        :param goal_y: y coordinate of goal
        '''

        self.map.clear_tree()
        self.goal = (goal_x, goal_y)
        self.map.set_street(
            self.goal[0], self.goal[1], self.position.x, self.position.y)

    def set_goal_from_shared(self):
        '''
        Gets shared data for robot goal.
        '''

        self.set_goal(self.shared_copy.goal[0], self.shared_copy.goal[1])

    def set_new_prize(self):
        self.map.clear_blocked()
        self.wait_time = 0

        # prioritize prize that is unexplored, then most optimal, then closest
        max_ratio = 0
        for prize in self.prize_ratio_dict:
            if (self.blocked_prize == prize):
                continue
            ratio = self.prize_ratio_dict[prize]
            if (max_ratio < ratio or (max_ratio == ratio and (self.cur_intersection.prize_distances[max_prize] > self.cur_intersection.prize_distances[prize]))):
                max_prize = prize
                max_ratio = ratio
        self.cur_prize = max_prize

        print(f"[NOTICE] Aiming for prize {self.cur_prize}. \n")

    def robot_straight_start(self):
        '''
        Called at the start. Makes the robot go straight until it finds an intersection.
        '''
        result = self.behavior.follow_road()

        # intersection reached
        if result == Road_State.INTERSECTION:
            # pull forward while using sensor information to check in front
            self.clear_map(start=True)
            self.ask_pos(is_start=True)

            self.update_prize_distances()

            self.robot_smart_pull_forward(self.pull_forward_time_start)

        # road end reached
        elif result == Road_State.END:
            self.behavior.pull_forward(self.pull_forward_time)

            self.behavior.turn(LEFT)

            # return back to previous intersection
            self.behavior.follow_road()

            # pull forward while using sensor information to check in front
            self.clear_map(start=True)
            self.ask_pos(is_start=True)

            self.update_prize_distances()

            self.robot_smart_pull_forward(self.pull_forward_time_start)
            self.position = self.calc_uturn(LEFT, self.position)
            self.map.update_street(self.cur_intersection,
                                   self.position.heading, Street_Status.DEADEND)
            self.position = self.calc_uturn(LEFT, self.position)

    def robot_straight(self, prize=False, dropping_prize=False, block_distance=None):
        '''
        Makes the robot go straight until intersection or road end, then takes appropriate action. 
        Updates robot position and street statuses in map accordingly. 
        '''
        # prevent the robot from going straight in nonexistent street
        self.cur_intersection = self.map.get_intersection(
            self.position.x, self.position.y)
        if (check_warning(self.cur_intersection.streets[self.position.heading] == Street_Status.NONEXISTENT,
                          "[WARNING] Cannot go straight because street doesn't exist.")):
            return

        # prevent the robot from going straight if blocked
        if (check_warning(self.update_blockage(self.cur_intersection, self.position.heading),
                          "[WARNING] Cannot go straight because street is blocked.")):
            return

        # go straight until next intersection
        self.prev_intersection = self.cur_intersection
        result = self.behavior.follow_road(prize=prize)

        match result:
            # intersection reached
            case Road_State.INTERSECTION:
                print("Reached a new intersection")
                # update the robot's position
                self.position = self.calc_move(self.position)

                # set the current intersection to the current intersection
                self.cur_intersection = self.map.get_intersection(
                    self.position.x, self.position.y)
                self.update_prize_distances()

                # if coming back from another intersection, update street status to connected
                if (self.prev_intersection != None):
                    if(self.map.update_street(
                        self.prev_intersection, self.position.heading, Street_Status.CONNECTED)):
                        self.robot_declare_lost()
                    if(self.map.update_street(self.cur_intersection,
                                           (self.position.heading+HALF_NUM_HEADINGS) % NUM_HEADINGS, Street_Status.CONNECTED)):
                        self.robot_declare_lost()

            # road end reached
            case Road_State.END:
                print("Reached a dead end!")
                self.behavior.pull_forward(self.pull_forward_time * 0.8)

                direction = LEFT
                self.behavior.turn(direction)

                if (dropping_prize):
                    self.behavior.magnet_off()

                # update street status to deadend & rest to nonexistent
                if (self.prev_intersection != None):
                    if(self.map.update_street(
                        self.prev_intersection, self.position.heading, Street_Status.DEADEND)):
                        self.robot_declare_lost()

                # uturn
                self.position = self.calc_uturn(direction, self.position)

                # return back to previous intersection
                self.behavior.follow_road()
                self.cur_intersection = self.map.get_intersection(
                    self.position.x, self.position.y)
                check_warning(self.prev_intersection != None and self.prev_intersection != self.cur_intersection,
                              "[WARNING] Robot should be returning to same intersection after u-turning from a dead end.\n")

                self.update_prize_distances()

            case Road_State.PRIZE:
                self.pickup_prize()
                print("Prize was picked up! Ay Ay Ay!")

                # mid-road, so finish going to end of road
                self.robot_straight(block_distance=block_distance)

        # pull forward while using sensor information to check in front
        if (result != Road_State.PRIZE):
            self.robot_smart_pull_forward(self.pull_forward_time)

        # check if there's a blockage in front of the robot
        if (self.cur_intersection.streets[self.position.heading] != Street_Status.NONEXISTENT):
            if (self.update_blockage(block_distance=block_distance)):
                print("[NOTICE] Street ahead blocked.\n")

    def robot_smart_pull_forward(self, duration):
        '''
        Sees if there is street in front when pulling forward and update map accordingly
        '''

        on_road = self.behavior.pull_forward(duration)

        if on_road:
            if(self.map.update_street(
                self.cur_intersection, self.position.heading, Street_Status.UNEXPLORED)):
                self.robot_declare_lost()
        else:
            if(self.map.update_street(
                self.cur_intersection, self.position.heading, Street_Status.NONEXISTENT)):
                self.robot_declare_lost()

        # read nfc of intersection and update prize distances for current intersection
        self.update_prize_distances()

    def robot_turn(self, direction):
        '''
        Makes the robot go turn in the specified direction.
        Updates robot position and street statuses in map accordingly. 

        :param direction: the direction to turn; -1 for right and 1 for left
        '''

        # set the current intersection to the current intersection
        self.cur_intersection = self.map.get_intersection(
            self.position.x, self.position.y)

        # record heading before turn
        initial_heading = self.position.heading

        # turn robot
        turn_time, turn_angle = self.behavior.turn(direction)

        # calculate angle based on time and angle reading and check if they agree
        turn_heading_angle = self.angle_to_heading(turn_angle)
        turn_heading_time = self.time_to_heading(turn_time, direction)

        check_warning(turn_heading_angle != turn_heading_time,
                      "[WARNING] Magnetometer and time disagee.")

        # guess turn heading based on angle, time, and map
        turn_heading = self.determine_turn_heading(direction,
                                                   turn_time, turn_heading_time,
                                                   turn_angle, turn_heading_angle)

        # update position based on turn heading
        self.position = self.calc_turn(turn_heading, direction, self.position)

        # record heading after turn
        final_heading = self.position.heading
        cur_street_heading = initial_heading

        # update street statuses
        for i in range(1, turn_heading):
            cur_street_heading = (cur_street_heading +
                                  direction) % NUM_HEADINGS
            if(self.map.update_street(
                self.cur_intersection, cur_street_heading, Street_Status.NONEXISTENT)):
                self.robot_declare_lost()

        if (self.update_blockage()):
            print("[NOTICE] Street ahead blocked.\n")

        if(self.map.update_street(self.cur_intersection,
                               final_heading, Street_Status.UNEXPLORED)):
            self.robot_declare_lost()

    def pickup_prize(self):
        print("[NOTICE] Picking up prize \n")

        prize_heading = self.prize_info_dict[self.cur_prize]["heading"]
        optimal_direction = self.get_optimal_turn_direction(
            self.position.heading, prize_heading) * -1

        # pick up the prize
        wait_seconds(0.2)
        self.behavior.pull_forward(self.pull_forward_time)
        wait_seconds(0.2)
        self.robot_turn(optimal_direction)
        wait_seconds(0.2)
        self.behavior.magnet_on()
        self.behavior.pull_backward(self.pull_backward_time)
        wait_seconds(0.2)
        self.behavior.pull_forward(self.pull_backward_time * 1.1)
        wait_seconds(0.2)
        self.robot_turn(optimal_direction * -1)

        self.has_prize = True
        print("[NOTICE] Prize picked up. \n")

        while (self.update_blockage(block_distance=15)):
            print("[NOTICE] Street ahead blocked. Waiting until blockage is cleared.\n")
            wait_seconds(0.2)

    def drop_prize(self):
        '''
        Drops current prize at home.
        '''
        print("[NOTICE] Dropping prize. \n")

        drop_off_heading = self.calc_uturn(RIGHT, self.home).heading
        optimal_direction = self.get_optimal_turn_direction(
            self.position.heading, drop_off_heading)

        # turn until robot is at right heading
        while (self.position.heading != drop_off_heading):
            self.robot_turn(optimal_direction)

        # drop the prize
        while(self.update_blockage()):
            wait_seconds(0.5)
        self.robot_straight(dropping_prize=True)
        self.has_prize = False
        print("[NOTICE] Dropped prize at home. \n")

    def robot_switch_to_input(self):
        '''
        Switches the drive mode to input.
        '''
        self.driving_mode = Mapping_Status.INPUT
        self.map.clear_tree()

    def robot_switch_to_goal(self):
        '''
        Switches the driving mode to GOAL and sets all optimal direction.
        '''

        self.set_goal(self.goal[0], self.goal[1])
        self.driving_mode = Driving_Mode.GOAL

    def robot_switch_to_explore(self):
        '''
        Switches the driving mode to EXPLORE.
        '''

        self.driving_mode = Driving_Mode.EXPLORE
        self.map.clear_tree()

        if (not self.map.is_complete() and self.map.is_complete(blocked_is_complete=True)):
            print(
                "[NOTICE] Non-blocked paths have all been explored. Clearing blockages and exploring.")
            self.map.clear_blocked()

        self.goal = None

    def robot_switch_to_fetch(self):
        '''
        Switches the driving mode to EXPLORE.
        '''

        self.driving_mode = Driving_Mode.FETCH
        self.map.clear_tree()
        self.goal = None
        self.cur_prize = self.shared_copy.cur_prize

    def robot_declare_lost(self):
        if(self.shared.acquire()):
            clear = input("Robot may be lost. Clear map? (Y/N)")
            if(clear == 'Y'):
                self.clear_map(start=True)
                self.ask_pos()
                self.robot_switch_to_input()
            self.shared.release()

    def angle_to_heading(self, angle):
        '''
        Converts angle to heading

        :param angle: angle to convert
        :return: angle converted to heading
        '''

        angle_thresholds = [20, 65, 121, 155, 210, 255, 310, 350]
        for i in range(0, len(angle_thresholds)):
            if (angle <= angle_thresholds[i]):
                return i

        return NUM_HEADINGS

    def time_to_heading(self, time, direction):
        if (direction == RIGHT):
            time_thresholds = [0, 0.6, 0.84, 1.045, 1.25, 1.45, 2, 2.8]
            # time_thresholds = [0, 0.62, 1.05, 1.27, 1.48, 1.67, 2.4, 2.8]
            heading = threshold_map(time_thresholds, time)
        else:
            time_thresholds = [0, 0.6, 0.8, 1.0, 1.28, 1.4, 2, 2.55]
            # time_thresholds = [0, 0.65, 1.05, 1.19, 1.395, 1.74, 2.4, 2.8]
            heading = threshold_map(time_thresholds, time)

        return heading

    def get_optimal_turn_direction(self, start_heading, goal_heading):
        '''
        Gets optimal turning direction when wanting to reach a heading.

        :param start_heading: heading to start from
        :param goal_heading: heading to end
        :return: optimal turning direction to reach goal_heading from start_heading
        '''

        left = get_delta_heading(start_heading, goal_heading, LEFT)
        right = get_delta_heading(start_heading, goal_heading, RIGHT)

        # figure if turning left or right would be better
        if left >= right:
            return RIGHT
        else:
            return LEFT

    def get_unexplored_to_goal(self, prev_best_heading=None):
        '''
        Gets the unexplored street that points closest to the goal

        :param prev_best_heading: the previously considered best heading, i.e. the heading from Dijkstra
        :return: heading of unexplored street pointing closest to the goal
        '''

        best_heading = None

        # from undiscovered streets, explore the one that points closest to goal
        x_displacement = -self.goal[0] + self.position.x
        y_displacement = -self.goal[1] + self.position.y
        angle_to_goal = math.degrees(
            atan2(y_displacement, x_displacement)) + 180

        min_delta_angle = math.inf
        undiscovered_neighbor_headings = self.cur_intersection.get_undiscovered_streets()
        for undiscovered_neighbor_heading in undiscovered_neighbor_headings:
            delta_angle = get_delta_angle(
                undiscovered_neighbor_heading * 45 + 90, angle_to_goal)

            if (delta_angle < min_delta_angle):
                best_heading = undiscovered_neighbor_heading
                min_delta_angle = delta_angle

        if (prev_best_heading != None):
            delta_angle = get_delta_angle(
                prev_best_heading * 45 + 90, angle_to_goal)

            if (delta_angle < min_delta_angle):
                best_heading = prev_best_heading

        return best_heading

    def determine_turn_heading(self, direction, turn_time, turn_heading_time, turn_angle, turn_heading_angle):
        '''
        Determine the turn heading based on time, angle, and map.

        :param direction: direction of the turn
        :param turn_heading_time: turn heading determined from time
        :param turn_heading_angle: turn heading determined from angle
        '''
        # 360 turns are exact, so use angle for that
        if (turn_heading_angle == NUM_HEADINGS):
            return turn_heading_angle

        # weighted average turn heading from angle and time
        turn_weight_angle = 0.15
        turn_weight_time = 0.85
        turn_heading_averaged = weighted_average([turn_heading_angle, turn_heading_time],
                                                 [turn_weight_angle, turn_weight_time])

        # determine the actual available turn headings from the map
        avail_street_headings = self.cur_intersection.get_neighbor_headings(skip_headings=[self.position.heading],
                                                                            get_blocked=True,
                                                                            get_unknown=True)
        avail_turn_headings = []
        for avail_street_heading in avail_street_headings:
            avail_turn_headings.append(get_delta_heading(self.position.heading,
                                                         avail_street_heading,
                                                         direction))

        print(f"[TURN] \ntime: {round(turn_time,3)} s, time angle: {round(turn_heading_time*45,1)}, \nmagnetometer: {round(turn_angle,1)} degrees, magnetometer angle: {round(turn_heading_angle*45,1)}, \navailable turn angles: {[round(i*45,1) for i in avail_turn_headings]}\n")

        # if map was incorrect and turn_heading was not initialize, use time measurement
        if (check_warning(len(avail_turn_headings) == 0,
                          "[WARNING] No available streets to turn to. Maybe map is incorrect?")):
            return turn_heading_time

        # get turn heading from available turn headings that is closest to weighted average
        turn_heading = None
        min_delta_turn_heading = math.inf
        for avail_turn_heading in avail_turn_headings:
            delta_turn_heading = get_delta_heading(
                avail_turn_heading, turn_heading_averaged, 0)
            if (delta_turn_heading < min_delta_turn_heading):
                min_delta_turn_heading = delta_turn_heading
                turn_heading = avail_turn_heading
        return turn_heading

    def calc_move(self, current_position):
        '''
        Calculates new self.position based on heading.

        :param current_position: current x, y, heading of robot
        :return: new self.position of robot depending on heading
        '''

        displacement = heading_dict[current_position.heading]
        dx = displacement[0]
        dy = displacement[1]
        current_position.x = current_position.x + dx
        current_position.y = current_position.y + dy

        return current_position

    def calc_turn(self, delta_heading, direction, current_position):
        '''
        Calculates new heading based on the turn angle.

        :param angle: turn angle of the robot. in degrees
        :param direction: turn direction of the robot. positive for right and negative for left
        :param current_position: current x, y, heading of robot
        :return: new heading of robot depending on turn angle
        '''

        current_position.heading = (
            current_position.heading + delta_heading * direction) % NUM_HEADINGS

        return current_position

    def calc_uturn(self, direction, current_position):
        '''
        Calculates new heading based on the a uturn.

        :param current_position: current x, y, heading of robot
        :return: new heading of robot after a uturn
        '''

        return self.calc_turn(HALF_NUM_HEADINGS, direction, current_position)

    def is_at_location(self, x, y):
        '''
        Checks whether the current robot's location is at the specified location

        :param x: the x coordinate of interest
        :param y: the y coordinate of interest
        :return: True or False; if the robot is at the location
        '''

        return (self.position.x == x and self.position.y == y)

    def save_map(self, name):
        '''
        Saves the current map

        :param name: the name of the file containing the map
        '''

        # Pick the pickle filename.
        filename = f'{name}.pickle'
        # Save the self.map to file.
        print("Saving map to %s..." % filename)
        with open(filename, 'wb') as file:
            pickle.dump(self.map, file)

    def load_map(self, name):
        '''
        Loads a map from a specified file

        :param name: the name of the file to load the map from
        '''

        self.clear_map()

        # pick the pickle filename
        filename = f'{name}.pickle'

        # load the self.map from file
        print("Loading map from %s..." % filename)

        try:
            with open(filename, 'rb') as file:
                self.map = pickle.load(file)
        except:
            print("[WARNING] Invalid map name.\n")
            return

        wait_seconds(0.3)

        # plot the map without robot so user can see the coordinates
        self.map.plot_map(None, False)

        self.ask_pos()

    def clear_map(self, start=False):
        self.map.clear_map()

        self.prev_intersection = None

        if (not start):
            self.cur_intersection = self.map.get_intersection(
                self.position.x, self.position.y)

    def check_blockage(self, block_distance=None):
        '''
        Checks if there is a blockage in the street ahead.

        :return: True/False; whether the street ahead is blocked.
        '''

        distance = self.behavior.read_front_blockage()
        print(f"Check blockage: {distance}")

        if (block_distance != None):
            if (distance < block_distance):
                return True
        else:
            if (self.position.heading % 2 == 0):
                if (distance < 60):
                    return True
            elif (self.position.heading % 2 == 1):
                if (distance < 80):
                    return True

        return False

    def update_blockage(self, intersection=None, heading=None, block_distance=None):
        '''
        Updates map if there is a blockage.

        :param intersection: the current intersection
        :param heading: the current heading
        :return: True/False; whether the street ahead is blocked.
        '''

        if (intersection == None):
            intersection = self.cur_intersection
        if (heading == None):
            heading = self.position.heading

        blocked = self.check_blockage(block_distance=block_distance)
        self.map.update_blocked(intersection, heading, blocked)
        return blocked

    def update_prize_distances(self):
        '''
        Updates the prize distances dictionary for the current intersection based on the NFC reading
        '''
        intersection_id = self.behavior.get_nfc_reading()
        # print(f"TEMP intersection id: {intersection_id}")

        if intersection_id != None:
            self.cur_intersection.prize_distances = self.prize_dist_dict[intersection_id]
            # print(f"TEMP prize distances: {self.cur_intersection.prize_distances}")
            self.prev_prize_distances = self.cur_intersection.prize_distances.copy()
            print(self.cur_intersection.prize_distances)
