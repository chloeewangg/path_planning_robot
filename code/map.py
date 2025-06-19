'''
!/usr/bin/env python3

map.py
Contains all functions related to the map.
'''

# Imports
from global_imports import *
from intersection import *
import matplotlib.pyplot as plt
import matplotlib.patches as patches

@dataclass
class Goal:
    intersection: Intersection = None # the intersection object corresponding to goal
    cost: int = 0 # starting cost of the goal; default 0

class Map():
    # maps heading number to coordinate change
    global heading_dict

    global NUM_HEADINGS
    global HALF_NUM_HEADINGS

    global MAP_MIN
    global MAP_MAX

    global STRAIGHT_COST
    global DIAGONAL_COST

    def __init__(self):
        '''
        Initializes intersection dictionary
        '''

        self.intersections = {}

    def clear_map(self):
        '''
        Clears the map.
        '''

        self.intersections = {}

    def is_empty(self):
        '''
        Checks if the map is empty. I.E. no discovered intersections

        :return: True or False; if the map is empty
        '''

        return len(self.intersections) == 0

    def is_complete(self, blocked_is_complete=False):
        '''
        Checks if map is complete (all intersection streets are discovered)

        :param blocked_is_complete: Whether or not to consider blocked streets as complete streets
        :return: True or False; if the map is completed
        '''
        if self.is_empty():
            return False

        for intersection in self.intersections.values():
            if(not intersection.is_complete(blocked_is_complete=blocked_is_complete)):
                return False

        return True
    
    def get_distance(self, intersection1, intersection2):
        '''
        Gets the distance between two intersections

        :param intersection1: the first intersection
        :param intersection2: the second intersection
        :return: the distance between the two intersections; infinity if unreachable
        '''

        self.set_street(intersection1.location[0], intersection1.location[1])
        distance = intersection2.cost
        self.clear_tree()
        return distance

    def get_closest_incomplete(self, cur_x, cur_y):
        '''
        Gets the nearest intersection that is incomplete.

        :param cur_x: current x coordinate
        :param cur_y: current y coordinate
        :return: the location of the nearest incomplete intersection excluding 
                 the current intersection; None if map is complete
        '''

        cur_intersection = self.get_intersection(cur_x, cur_y)

        if(self.is_complete()):
            return None
        
        min_distance = math.inf
        closest_intersection_location = None

        for intersection in self.intersections.values():
            # only iterate through incomplete intersections
            if(intersection.is_complete(blocked_is_complete=True) or intersection.location == (cur_x, cur_y)):
                continue
            
            # run Dijkstra's setting current intersection as goal 
            intersection_location = intersection.location
            self.set_street(intersection_location[0], intersection_location[1])

            # if intersection is unreachable right now because of blockages, remove from candidates
            if(cur_intersection.optimal_direction == None):
                continue

            # check if the distance to the goal is the minimum distance
            if(min_distance == math.inf or cur_intersection.cost < min_distance):
                min_distance = cur_intersection.cost
                closest_intersection_location = intersection_location

        self.clear_tree()

        return closest_intersection_location
    
    def get_next_to_prize(self, prize_num, cur_x, cur_y):
        '''
        Gets intersections next to the specified prize

        :param prize_num: the number of the prize
        :param cur_x: the current x location
        :param cur_y: the current y location
        :return: any intersections next to the prize; first element is closer
        '''

        intersections = []

        for intersection in self.intersections.values():
            distance = intersection.prize_distances[prize_num]
            if(distance == STRAIGHT_COST / 2):
                intersections.append(intersection)

        cur_intersection = self.get_intersection(cur_x, cur_y)
        if(len(intersections) == 2 and 
           self.get_distance(cur_intersection, intersections[0]) > self.get_distance(cur_intersection, intersections[1])):
            intersections[0], intersections[1] = intersections[1], intersections[0]
        
        return intersections

    def get_closest_to_prize(self, prize_num, cur_intersection, skip_complete=True):
        '''
        Gets the intersection with the least distance to the specified prize

        :param prize_num: the number of the prize
        :param cur_intersection: the current intersection of robot
        :param skip_complete: True/False; whether to skip completed intersections
        :return: an incomplete intersection closest to the prize
        '''
       
        min_distance = math.inf
        min_distance_cur = math.inf
        closest_location = None

        for intersection in self.intersections.values():
            if(skip_complete and intersection.is_complete(blocked_is_complete=True)):
                continue

            if(not self.is_intersection_reachable(intersection, cur_intersection.location[0], cur_intersection.location[1])):
                continue

            distance = intersection.prize_distances[prize_num]
            distance_cur = self.get_distance(cur_intersection, intersection)
            
            # prioritize one closest to prize
            if(distance < min_distance):
                min_distance = distance
                min_distance_cur = distance_cur
                closest_location = intersection

            # prioritize one closer to robot and one that is reachable
            elif(distance == min_distance):
                if(closest_location == None):
                    min_distance = distance
                    min_distance_cur = distance_cur
                    closest_location = intersection
                elif((not self.is_intersection_reachable(closest_location, cur_intersection.location[0], cur_intersection.location[1]) and
                   self.is_intersection_reachable(intersection, cur_intersection.location[0], cur_intersection.location[1])) or
                   distance_cur < min_distance_cur):
                    min_distance_cur = distance_cur
                    closest_location = intersection
        
        self.clear_tree()

        return closest_location

    def is_intersection_discovered(self, x, y):
        '''
        Tells if intersection was previousely discovered / inside loaded graph

        :param x: x location of the intersection
        :param y: y location of the intersection
        :return: True or False; whether intersection was discovered or not
        '''
        return (x, y) in self.intersections
    
    def is_intersection_reachable(self, intersection, cur_x, cur_y):
        '''
        Checks if the intersection is reachable from the current position

        :param intersection: object representing the goal intersectoin
        :param cur_x: the current x coordinate
        :param cur_y: the current y coordinate
        :return: True/False; if the intersection is reachable
        '''

        if(not self.is_intersection_discovered(cur_x, cur_y)):
            return False
        
        if(intersection.location[0] == cur_x and intersection.location[1] == cur_y):
            return True
        
        cur_intersection = self.get_intersection(cur_x, cur_y)

        goal = [Goal(intersection, 0)]
        self.dijkstras(goal)

        is_reachable = (cur_intersection.optimal_direction != None)
        self.clear_tree()

        return is_reachable

    def get_intersection(self, x, y):
        '''
        Gets the intersection object from the location.

        :param x: x location of the intersection
        :param y: y location of the intersection
        :return: intersection object for the intersection located at (x, y)
        '''

        # Create new object for intersection if it hasn’t been used before.
        if (x,y) not in self.intersections:
            self.intersections[(x,y)] = Intersection(x,y)
        return self.intersections[(x,y)]
    
    def get_intersection_if_discovered(self, x, y):
        '''
        Gets the intersection object from the location only if it was previously discovered.

        :param x: x location of the intersection
        :param y: y location of the intersection
        :return: None if intersection is not discovered yet; otherwise, intersection object for the intersection located at (x, y)
        '''
        if(not self.is_intersection_discovered(x, y)):
            return None
        return self.get_intersection(x, y)
    
    def get_potential_intersection_location(self, x, y, heading):
        '''
        Gets the location of a potential intersection if the robot were to start from (x, y) and go in direction of heading

        :param x: x location of the intersection
        :param y: y location of the intersection
        :param heading: the heading direction
        '''
        return (x + heading_dict[heading][0], 
                y + heading_dict[heading][1])
    
    def get_intersection_neighbors(self, intersection, get_blocked=False):
        '''
        Returns headings of all existing streets that are connected to another intersection

        :param get_blocked: True/False; whether to get blocked streets or not
        :return: an array of the neighbor intersection objects
        '''

        neighbors = []
        neighbor_headings = intersection.get_connected_neighbor_headings(get_blocked=get_blocked)

        for heading in neighbor_headings:
            neighbor_location = (intersection.location[0] + heading_dict[heading][0], intersection.location[1] + heading_dict[heading][1])
            neighbors.append(self.get_intersection(neighbor_location[0], neighbor_location[1]))
        
        return neighbors

    def clear_tree(self):
        '''
        Set all optimal costs in intersection objects to 0.
        '''

        for intersection in self.intersections.values():
            intersection.cost = math.inf
            intersection.optimal_direction = None

    def update_street(self, intersection, heading, status):
        '''
        Updates the street status of a specified street.

        :param intersection: the intersection object containing the street
        :param heading: the direction of the street relative to the intersection
        :param status: the new value for the status
        '''

        cur_status = intersection.streets[heading]

        lost = False

        if (check_warning((cur_status == Street_Status.UNEXPLORED or cur_status == Street_Status.CONNECTED) and status == Street_Status.NONEXISTENT, 
                      "[WARNING] Attempted to change a previousely discovered street to NONEXISTENT. Robot may be lost.")):
            lost = True

        if(cur_status == Street_Status.UNKNOWN 
            or cur_status == Street_Status.UNEXPLORED
            or cur_status == Street_Status.NONEXISTENT):
            intersection.streets[heading] = status

        # assume no two streets are in 45 degree angle
        if(status == Street_Status.UNEXPLORED or status == Street_Status.CONNECTED or status == Street_Status.DEADEND):
            left_heading = (heading + 1) % NUM_HEADINGS
            right_heading = (heading - 1) % NUM_HEADINGS
            self.update_street(intersection, left_heading, Street_Status.NONEXISTENT)
            self.update_street(intersection, right_heading, Street_Status.NONEXISTENT)

        return lost
    
    def update_blocked(self, intersection, heading, blocked=False):
        '''
        Updates if the street in front of the robot is blocked.

        :param intersection: the intersection object containing the street
        :param heading: the direction of the street relative to the intersection
        :param blocked: a boolean representing if the street is blocked or not
        '''

        intersection.blocked[heading] = blocked

        opposite_intersection_location = (intersection.location[0]+heading_dict[heading][0], 
                                          intersection.location[1]+heading_dict[heading][1])
        opposite_intersection = self.get_intersection_if_discovered(opposite_intersection_location[0], 
                                                                    opposite_intersection_location[1])
        if(opposite_intersection != None):
            opposite_intersection.blocked[(heading + HALF_NUM_HEADINGS) % NUM_HEADINGS] = blocked
    
    def clear_blocked(self):
        '''
        Updates all blockages in each intersection to be False and clears all blockages on the map.
        '''

        for intersection in self.intersections.values():
            intersection.blocked = [False] * 8

    def set_street(self, xgoal, ygoal, cur_x=None, cur_y=None): 
        '''
        Runs pathfinding algorithm to set optimal direction and weight to reach goal for every intersection.

        :param xgoal: x location of goal
        :param ygoal: y location of goal
        '''
        
        goals = []

        # if going to discovered goal, just Dijkstra
        if(self.is_intersection_discovered(xgoal, ygoal)):
            goal_intersection = self.get_intersection(xgoal, ygoal)
            if(cur_x == None or self.is_intersection_reachable(goal_intersection, cur_x, cur_y)):
                goal = Goal(goal_intersection, 0)
                goals.append(goal)
                self.dijkstras([Goal(self.get_intersection(xgoal, ygoal), 0)])
                return

        # if going to undiscovered goal or blocked goal, do directed explore with multiple goals
        # start with every discovered intersection that is not fully complete as goal
        for intersection in self.intersections.values():
            if(intersection.is_complete(blocked_is_complete=True)):
                continue

            x_displacement = abs(xgoal - intersection.location[0])
            y_displacement = abs(ygoal - intersection.location[1])
            displacement = math.sqrt(x_displacement**2 + y_displacement**2) # Euclidean distance to goal

            cost = displacement * 2
            goal = Goal(intersection, cost)
            goals.append(goal)
        
        self.dijkstras(goals)

    def dijkstras(self, goals):
        '''
        Uses Dijkstra's algorithm to set optimal direction and weight to reach goal for every intersection.

        :param goals: intersection object of the goal(s)
        '''
        
        # initialize
        self.clear_tree()

        onDeck = SortedQueue(lambda intersection: intersection.cost)

        for goal in goals:
            goal.intersection.cost = goal.cost
            goal.intersection.aim_location = goal.intersection.location
            onDeck.push(goal.intersection)

        while onDeck.size() != 0:
            node = onDeck.pop()
            # iterate through all neighbors of the next_intersection
            neighbor_headings = node.get_connected_neighbor_headings()
            for heading in neighbor_headings:
                # get the neighbor object
                neighbor_location = (node.location[0] + heading_dict[heading][0], node.location[1] + heading_dict[heading][1])
                neighbor = self.get_intersection(neighbor_location[0], neighbor_location[1])

                # get potential cost for neighbor
                potential_direction = (heading + HALF_NUM_HEADINGS) % NUM_HEADINGS
                potential_cost = node.cost + (STRAIGHT_COST if heading % 2 == 0 else DIAGONAL_COST)

                # if potential cost lower than neighbor's known cost, update it
                if (potential_cost < neighbor.cost):
                    onDeck.remove(neighbor)
                    neighbor.cost = potential_cost
                    neighbor.optimal_direction = potential_direction
                    neighbor.aim_location = node.aim_location
                    onDeck.push(neighbor)

    def plot_map(self, position, with_robot=True):   
        '''
        Plots the current map.

        :param position: (x, y, heading) position of the robot; can be None if with_robot is False
        :param with_robot: True or False; whether to draw the robot or not
        '''
        street_color_dict = {
            Street_Status.UNKNOWN : 'black',
            Street_Status.NONEXISTENT : 'lightgray',
            Street_Status.UNEXPLORED : 'blue',
            Street_Status.DEADEND : 'red',
            Street_Status.CONNECTED : 'green'
        }

        # Clear the current, or create a new figure.
        plt.clf()

        plt_min = MAP_MIN
        plt_max = MAP_MAX

        # Create a new axes, enable the grid, and set axis limits.
        plt.axes()
        plt.gca().set_xlim(plt_min[0] + 0.5, plt_max[0] - 0.5)
        plt.gca().set_ylim(plt_min[1] - 0.5, plt_max[1] + 0.5)
        plt.gca().set_aspect('equal')

        # Show all the possible locations.
        for x in range(plt_min[0], plt_max[0] + 1):
            for y in range(plt_min[1], plt_max[1] + 1):
                plt.plot(x, y, color='lightgray', marker='o', markersize=8, zorder=1)

        # Draw lines with the correct color for each street
        for intersection in self.intersections.values():
            for heading, street_status in enumerate(intersection.streets):
                xfrom = intersection.location[0]
                xto = xfrom + heading_dict[heading][0] / 2
                yfrom = intersection.location[1]
                yto = yfrom + heading_dict[heading][1] / 2
                c = street_color_dict[street_status]

                plt.plot([xfrom, xto], [yfrom, yto], color=c, zorder=2)

        # Draw squares to indicate blockage
        for intersection in self.intersections.values():
            for heading, street_status in enumerate(intersection.streets):
                if (intersection.blocked[heading]):
                    square_location = (intersection.location[0] + heading_dict[heading][0] / 2,
                                       intersection.location[1] + heading_dict[heading][1] / 2)
                    square_side = 0.3
                    square_x = [square_location[0]+square_side/2, square_location[0]+square_side/2,
                                square_location[0]-square_side/2, square_location[0]-square_side/2]
                    square_y = [square_location[1]+square_side/2, square_location[1]-square_side/2,
                                square_location[1]-square_side/2, square_location[1]+square_side/2]
                    plt.fill(square_x, square_y, color="orange", zorder=3)

        # Draw arrows showing all optimal directions
        for intersection in self.intersections.values():
            optimal_heading = intersection.optimal_direction

            if(optimal_heading != None):    
                xbase = intersection.location[0] - heading_dict[optimal_heading][0] / 4
                ybase = intersection.location[1] - heading_dict[optimal_heading][1] / 4
                xtip = intersection.location[0] + heading_dict[optimal_heading][0] / 4
                ytip = intersection.location[1] + heading_dict[optimal_heading][1] / 4

                plt.arrow(xbase, ybase, xtip-xbase, ytip-ybase,
                    width=0.05,
                    head_width=0.2,
                    head_length=0.1,
                    color='orange', zorder=4)

        # Draw an arrow in robot's current location
        if (with_robot == True):
            xbase = position.x - heading_dict[position.heading][0] / 4
            ybase = position.y - heading_dict[position.heading][1] / 4
            xtip = position.x + heading_dict[position.heading][0] / 4
            ytip = position.y + heading_dict[position.heading][1] / 4

            plt.arrow(xbase, ybase, xtip-xbase, ytip-ybase,
                width=0.2,
                head_width=0.3,
                head_length=0.1,
                color='magenta', zorder=5)

        # Show the graph and continue
        plt.pause(0.001)
    
