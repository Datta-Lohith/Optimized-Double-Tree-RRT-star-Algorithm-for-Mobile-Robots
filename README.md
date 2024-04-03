# Project 5

# ENPM661: Planning for Autonomous Robots

## Contents

- double_tree_rrt_star.py
- dbtree_rrt_star_ros.py
- dbtree_rrt_star_slam.py
- output.html    

## Prerequisties
- Ubuntu 20.04 LTS
- ROS noetic
- Gazebo11
- Turtlebot3 Packages

## Dependencies
- python 3.8
- Visual studio code

## Libraries
- Visualization
    - import numpy as np
    - import random
    - import plotly as py
    - import cv2 as cv
    - import time
    - from shapely.geometry import Point
    - from plotly import graph_objs as go
    - from rtree import index
    - from operator import itemgetter
- ROS and SLAM
    - import numpy as np
    - import random
    - import plotly as py
    - import cv2 as cv
    - import time
    - from shapely.geometry import Point
    - from plotly import graph_objs as go
    - from rtree import index
    - from operator import itemgetter
    import numpy as np
    - import rospy
    - from tqdm import tqdm
    - from geometry_msgs.msg import Twist
    - from tf.transformations import euler_from_quaternion
    - from nav_msgs.msg import Odometry

## Workspace and Turtlebot(paste the following commands line by line)
```
$ mkdir catkin_ws/src
$ cd catkin_ws
$ catkin_make
$ source catkin_ws/src/devel/setup.bash
$ cd catkin_ws/src
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
$ git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
$ cd  ../ && catkin_make
```

## How to run the code

- Download the zip file and extract it to your work space
- Install Python 3 and the libraries mentinoned above
- For Visualizing the working of the algorithm, run `double_tree_rrt_star.py` and follow the instructions in the terminal.
- For ROS Simulation and SLAM
    - Setup the workspace and install the necessary libraries mentioned
    - Use the other two files accordingly

