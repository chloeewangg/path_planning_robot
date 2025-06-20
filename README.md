# Path Planning Robot

<div align="center">
  <img src="https://github.com/user-attachments/assets/332d929c-6cd8-472c-bd2a-5496780ea97d" width="500">
</div>

[DEMO VIDEO](https://www.youtube.com/watch?v=NP6a2vPLHQw)

A path planning robot which can explore, build maps, find goals, and find prizes on simple maps. The robot features a collection of robust behaviors, including line-following and blockage-detection, which are produced from imperfect hardware and limited information. This project was developed as part of the Experimental Robotics course at Caltech, aiming to turn simple mechanical devices equipped with simple sensors into robust, autonomous robots.

## Features
### Line Following and Road Feature Detection
The robot relies on 3 IR sensors to follow thick black tape (shown in demo). As it drives, it responds to feedback laws involving each sensor reading and the intensity it turns while driving. For example, if the sensors indicate the robot is on the far left side of the tape, then it will drive to the right to correct itself. These are done in very short time steps, so the robot follows the line very rigidly. 

If the robot is pushed off the road, it can correct itself (shown in demo). This is done by filtering incoming sensor readings to maintain a running average estimate of which side of the road the robot is on. When the robot is pushed off, it then relies on this estimate to turn back onto the road.

The robot also uses this filtering technique to detect road features, such as intersections and dead ends. This makes it robust to noise, such as small debris and thin tape on the map. 

### Turning and Self-Correction
To turn, the robot uses the previously mentioned filtering technique to determine when it has turned off the current street and onto the next. It then self-corrects to ensure it is angled well with the oncoming street. 

To determine the angle it turned, it relies on (i) how long the turn took, (ii) the magnetometer reading, and (iii) a list of the possible angles it could have turned according to the map. (i) is weighed more heavily in determining the turn angle since we found it to be more robust than (ii), but the user is still notified if (i) and (ii) disagree. 

If the turn angle determined by (i) and (ii) is not in (iii), then, if possible, turn angle is automatically corrected to the nearest angle in (iii). If correction is not possible, then the robot most likely has or will turn onto a nonexistent street (meaning the street is not there according the map). The robot will then declare itself lost and ask for help from the user.

### Blockage Detection
The robot is equipped with 3 ultrasonic sensors which can detect objects. However, we found our robot worked sufficiently using only the front sensor. 

As the robot is driving, if it detects a blockage within 10 cm, it will stop and wait for it to clear. If the robot is at an intersection and detects a blockage on the street it is facing, then the map will be updated to show a blockage. To make blockage detection more robust, the robot takes multiple readings at each intersection, and responds to the median value. 

### Manual Driving
The user can manually drive the robot around the map by commanding it to turn right, turn left, and go straight. The map is updated as the user drives. If the street ahead is blocked or nonexistent, the program will prevent the user from driving forward.

### Autonomous Exploration and Mapping
The robot autonomously explores by driving to intersections, discovering and storing the information about the streets of each intersection, and plotting this information on a map. It will continue exploring until all streets are explored or it is fully blocked. Restarting an explore will clear the blockages, resulting in the robot checking any previous blockages again. If these blockages have been removed, the robot will continue exploring and building upon the same map. 

An example of a map and a table of the corresponding street colors are shown below. Note that for the maps we worked with, we could assume only 8 possible streets per intersection. Orange squares indicate blockages.

<img src="https://github.com/user-attachments/assets/826ff758-3dc2-45d9-92d9-90329e1c304c" width="500">

| Street Color | Description |
|:---|:---|
| Black | Undiscovered |
| Gray | Nonexistent |
| Blue | Unexplored (but discovered) |
| Green | Connected to another street |
| Red | Dead end |

### Goal Finding via Dijkstra's Algorithm
The robot can be given a goal to go towards, whether or not that goal has been discovered and plotted. If the robot has already explored the goal, then it uses Dijkstra's algorithm with Manhattan distances from intersections as costs in order to find the lowest cost path towards the goal. If it has not explored the goal, the robot will perform a "directed explore", where it will search for the closest street facing the direction of the goal, and continue exploring down these paths until it reaches the goal.

For our implementation of Dijkstra's algorithm results, we assign each intersection an "optimal direction" to the goal (indicated by yellow arrows on the map). The example below shows the robot moving towards the goal (0, -2).

<img src="https://github.com/user-attachments/assets/9d823fa7-8d3f-491f-9610-90eccb07f385" width="500">

If the robot has fully explored the map, and an invalid goal is given, then the user is told the robot cannot reach that goal. However, if the robot has not fully explored the map, then it will keep searching for this goal until the map is fully explored. If there are blockages that prevent the robot from fully exploring the map, it will continue to recheck the blockages indefinitely. 

### Retrieving Prizes
The robot can retrieve prizes (magnetic washers) throughout the map. Beneath each intersection is an NFC tag with ID information about that intersection. The robot is equipped with an NFC reader so that each time it pulls up on an intersection, it reads this ID. The robot is also provided with a dictionary so that using this ID, it can determine how far each prize is. 

<img src="https://github.com/user-attachments/assets/27c1f8ac-cb92-4dfd-82e6-228a5a37ad88" width="500">  

The fetch process follows the proceeding steps:
1. The robot stores the prize information for the intersection it is currently at.
2. The robot turns to determine where all the streets of that intersection are. Then it chooses the closest unexplored street and goes down that street.
3. The robot checks and stores the prize information of this next intersection. It then compares how far the prize is from the current intersection and from the previous intersection. If the current intersection is closer, it will repeat going to the closest unexplored street. If the current intersection is further, it will return to the previous intersection and go down a different unexplored street.
4. Repeat until 2 intersections that are 0.5 streets from the prize are found (meaning the prize is between these 2 intersections).
5. To retrieve the prize, the robot drives between the 2 intersections until it detects a prize feature.
6. The robot picks up the prize using the electromagnet.
7. The robot uses Dijkstra's algorithm to return home (the robot's starting position).
8. The robot drops the prize off at the dead end connected to this home intersection.

### User Interface
The robot operates according to the following input commands:
|Command|Input|Description|
|:---|:---|:---|
|Explore|E|Starts the explore behavior.|
|Goal|G|Asks the user for the goal's coordinates and goes to that goal.|
|Fetch|F|Asks the user for the prize number and fetches that prize.|
|Pause|PA|Pauses the robot if it is exploring, going to a goal, or fetching.|
|Step|SP|Allows the robot to move one step and stop again if it is exploring, going to a goal, or fetching.|
|Resume|R|Resumes exploring, going to a goal, or fetching if the robot is paused.|
|Left|LT|Makes the robot turn left.|
|Right|RT|Makes the robot turn right.|
|Straight|S|Makes the robot go straight.|
|Save|SV|Saves the current map to a .pickle file.|
|Load|LO|Loads a previously saved map.|
|Clear|CL|Clears the current map.|
|Pose|P|Moves the robot to a new coordinate and heading.|
|Show|SH|Shows the most recently updated map.|
|Clear blockage|CB|Clears all blockages from the map.|
|Quit|Q|Exits the program.|

### Multi-threading
We utilize multiple threads to protect data as it is updated between files. Below are descriptions of the threads.
|Thread|Description|
|:---|:---|
|Main (robot)| Runs the main functions of the robot. |
|UI| Ensures the program is constantly listening for user commands. |
|GPIO| Handles GPIO interactions. |
|Trigger| Allows the ultrasonic sensors to trigger at random times (to protect against interference). |
|NFC| Allows the robot to constant read for NFC tags. |
|Matplotlib| Handles Matplotlib operations. |

### ROS Interface
The ROS interface allows the robot to receive external commands, enabling a remote computer to send commands to the robot and view its position.

### Battery Case
The robot features a cat ᓚ₍ ^. .^₎ battery case to maintain a consistent positioning of the battery and raspberry pi. Since we used simple and cheap hardware, the positioning of the battery widely affected the driving of the robot, necessitating the case. 

## Hardware
- Raspberry Pi
- IR Sensors (x3)
- Ultrasound Sensors (x3)
- Magnetometer (x2)
- Geared DC Motors (x2)
- L298N DC Motor Driver Module 
- Electromagnet
- NFC Tag Reader
- Battery + Battery case 

## Software
The code is organized according to the following structure:
| Level | File(s) | Purpose | Handles |
|:---|:---|:---|:---|
| Highest | [robot](https://github.com/chloeewangg/path_planning_robot/blob/main/code/robot.py) | Runs the entire system | Threading |
| Upper Middle | [brain](https://github.com/chloeewangg/path_planning_robot/blob/main/code/brain.py) | Decision making and logic | Logic, Coordinates, Mapping | 
| Lower Middle | [behaviors](https://github.com/chloeewangg/path_planning_robot/blob/main/code/behaviors.py) | Manages behaviors (i.e. line following, turning, etc.) | Feedback and detectors |
| Lowest | [IR_sensor](https://github.com/chloeewangg/path_planning_robot/blob/main/code/IR_sensor.py) [NFC_sensor](https://github.com/chloeewangg/path_planning_robot/blob/main/code/NFC_sensor.py) [driver](https://github.com/chloeewangg/path_planning_robot/blob/main/code/driver.py) [electromagnet](https://github.com/chloeewangg/path_planning_robot/blob/main/code/electromagnet.py) [magnetometer](https://github.com/chloeewangg/path_planning_robot/blob/main/code/magnetometer.py) [proximity_sensor](https://github.com/chloeewangg/path_planning_robot/blob/main/code/proximity_sensor.py) | Hardware Interface | Pins, PWM, bits |

### Software Used
- Python
- Numpy
- Matplotlib

## Learning Outcomes
### Skills
- Integrated a variety of sensor modalities and applied signal processing techniques to sensor data.
- Implemented a multi-threaded architecture.
- Practiced debugging techniques for both hardware and software.
- Produced robust autonomous behavior from imperfect hardware and limited information.

### Challenges and Known Issues
- The turn timing of the robot is very sensitive to the mass of the robot and battery level. If the robot collides as it is turning, this can also cause it the turn time to increase and not match the turn times for each heading.
- We sought for robustnessness over optimality, so some behaviors (e.g. searching for an unknown goal) can take a very long time.
- The cone of the front ultrasonic sensor can sometimes cause the robot to read blockages that are not directly in front of it. Fixing this will require further tuning of the ultrasound distance thresholds.
- The robot can end up in a "stalemate" with another robot if it drives down a street at the same time as another robot. Fixing this will require implementing a u-turn behavior after waiting at a blockage for some time.

## Contributors
- Ritta Choi (mchoi2@caltech.edu)
- Chloe Wang (chloew@caltech.edu)
- Luis Serrano Laguna (lserrano@caltech.edu)

## Acknowledgements
Thank you to our professor, Gunter Niemeyer, and TA, Emily Baylock, for guiding us through this project!
