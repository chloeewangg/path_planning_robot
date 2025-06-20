# Path Planning Robot

<div align="center">
  <img src="https://github.com/user-attachments/assets/332d929c-6cd8-472c-bd2a-5496780ea97d" width="500">
</div>

[DEMO VIDEO](https://www.youtube.com/watch?v=NP6a2vPLHQw)

A path-planning robot which can explore, build maps, find goals, and find prizes on simple maps. The robot features a collection of robust behaviors, including line-following and blockage-detection. This project was developed as part of the Experimental Robotics course at Caltech, aiming to turn simple mechanical devices equipped with simple sensors into robust, autonomous robots.

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

### Autonomous Exploration and Mapping

### Goal Finding and Directed Explore via Dijkstra's Algorithm

### Retrieving Prizes

### Multi-threading

### ROS Interface

### User Interface

## Hardware
- Raspberry Pi
- IR Sensors (x3)
- Ultrasound Sensors (x3)
- Magnetometer (x2)
- Geared DC Motors (x2)
- L298N DC Motor Driver Module 
- Electromagnet
- NFC Tag Reader
- Battery + Battery case **Robot movement is heavily subject to battery positioning

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
State estimation.
Filtering.

### Challenges and Known Issues

## Contributors
- Ritta Choi (mchoi2@caltech.edu)
- Chloe Wang (chloew@caltech.edu)
- Luis Serrano Laguna (lserrano@caltech.edu)

## Acknowledgements
Thank you to our professor, Gunter Niemeyer, and TA, Emily Baylock, for guiding us through this project!
