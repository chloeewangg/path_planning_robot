from global_imports import *

class UI():
    '''
    Class for all UI related functions.
    '''

    def __init__(self, shared):
        '''
        Initializes shared.
        '''
        self.shared = shared

    def run_UI(self):
        '''
        Get user input and update the shared flags based on it. This will influence how the 
        robot plans its action.

        :param shared: the shared data
        '''
        global MAP_MIN
        global MAP_MAX

        global NUM_HEADINGS
        global HALF_NUM_HEADINGS

        running = True
        while running:
            cmd = self.get_cmd()

            if self.shared.acquire():
                self.shared.cmd = cmd
                
                match cmd: 
                    case Command.EXPLORE: 
                        self.shared.paused = False
                        
                    case Command.GOAL: # set goal location
                        x_goal = int(get_user_input([f"{i}" for i in range(MAP_MIN[0], MAP_MAX[0]+1)], 
                                                "Enter the x coordinate of the goal: ",
                                                "Invalid x coordinate. Enter again."))
                        y_goal = int(get_user_input([f"{i}" for i in range(MAP_MIN[1], MAP_MAX[1]+1)], 
                                                "Enter the y coordinate of the goal: ",
                                                "Invalid y coordinate. Enter again."))
                        self.shared.goal = (x_goal, y_goal)
                        self.shared.paused = False
                    
                    case Command.FETCH: 
                        self.shared.cur_prize = int(get_user_input([f"{i}" for i in range(1, 19)],
                                                    f"Enter the aim prize: ",
                                                    "Invalid prize. Enter again."))
                    
                    case Command.PAUSE:
                        self.shared.paused = True

                    case Command.RESUME:
                        self.shared.paused = False
            
                    case Command.POSE: # set new robot position
                        robot_heading = int(get_user_input([f"{i}" for i in range(0, NUM_HEADINGS)],
                                            f"Enter the robot's start direction (0-{NUM_HEADINGS-1}): ",
                                            "Invalid heading. Enter again."))
                        robot_position_x = int(get_user_input([f"{i}" for i in range(MAP_MIN[0], MAP_MAX[0]+1)], 
                                                            "Enter the x coordinate of the robot's start position: ",
                                                            "Invalid x coordinate. Enter again."))
                        robot_position_y = int(get_user_input([f"{i}" for i in range(MAP_MIN[1], MAP_MAX[1]+1)], 
                                                            "Enter the y coordinate of the robot's start position: ",
                                                            "Invalid y coordinate. Enter again."))
                        self.shared.robot_pos_input = Position(robot_position_x, robot_position_y, robot_heading) # (x, y, heading)

                    case Command.SAVE:
                        self.shared.map_name = input("Enter name to save map to:")

                    case Command.LOAD:
                        self.shared.map_name = input("Enter name of map to load:")

                    case Command.QUIT:
                        self.shared.quit = True
                        running = False
                    
                # Release the self.shared data.
                self.shared.release()

    def get_cmd(self):
        '''
        Prints the list of instructions and gets the user command.

        :return: the user command
        '''
        avail_input = [command.value for command in Command]
        input_msg = "------------------------------\n"
        for command in Command:
            if(command == Command.NONE):
                continue
            input_msg += command.name + ": " + command.value + "\n"
        input_msg += "-----------------------------\n"
        cmd = Command(get_user_input(avail_input, input_msg, "Illegal command. Enter again: "))

        return cmd