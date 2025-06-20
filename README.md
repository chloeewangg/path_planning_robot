# Path Planning Robot

<div align="center">
  <img src="https://github.com/user-attachments/assets/332d929c-6cd8-472c-bd2a-5496780ea97d" width="500">
</div>

[DEMO VIDEO](https://www.youtube.com/watch?v=NP6a2vPLHQw)

A path-planning robot which can explore, build maps, find goals, and find prizes on simple maps. The robot features a collection of robust behaviors, including line-following and blockage-detection. This project was developed as part of the Experimental Robotics course at Caltech, aiming to turn simple mechanical devices equipped with simple sensors into robust, autonomous robots.

## Features and Behaviors
### Line Following and Road Feature Detection

### Turning and Self-Correction

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
