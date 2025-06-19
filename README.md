# Path Planning Robot

<div align="center">
  <img src="https://github.com/user-attachments/assets/332d929c-6cd8-472c-bd2a-5496780ea97d" width="500">
</div>

[DEMO VIDEO](https://www.youtube.com/watch?v=NP6a2vPLHQw)

A path-planning robot which can explore, build maps, find goals, and find prizes on simple maps. The robot features a collection of robust behaviors, including line-following and blockage-detection. This project was developed as part of the Experimental Robotics course at Caltech, aiming to turn simple mechanical devices equipped with simple sensors into robust, autonomous robots.

## Features and Behaviors
The robot is capable of line following and self-correction if it deviates off the line. The robot can also distinguish betwwe a dead-end in the path of a line and an intersection of lines, which it maps for user visualization. The robot can also determine the optimal path towards a specified intersection node through Djikstra's algorithm and can replan if it encounters a blocked road reading. The robot is also capable of turning at intersections and uses sensor and time-based feedback to accurately distinguish the robot's heading on the map. There also exists a ROS user interface that can control the robot, telling it to either explore, djikstra's to a node, or search for and retrieve a specified prize.

## Hardware
- Raspberry Pi
- IR Sensors
- Ultrasound Sensors
- Magnetometer
- Geared DC Motors
- L298N DC Motor Driver Module
- Electromagnet
- NFC Tag Reader

## Software
- Python
- RealVNC virtual machine

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
