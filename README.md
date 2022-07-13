# HybridMotionPlanning

## Overview
In this project, we implemented Informed RRT* in C++ and Python to find a path on a map in Gazebo. The map was mapped using laser scan and a cost map was generated to perform planning. A turtlebot was used to move from the current location to the goal location. Custom plugins were made for local and global planners.

---

## Results
**Map used for planning**
![WhatsApp Image 2022-04-26 at 1 22 28 AM](https://user-images.githubusercontent.com/35029771/178841203-43d9fb26-d69c-43c8-82eb-7a00a3e3c9ab.jpeg)

**Paths found by Informed RRT** 

![WhatsApp Image 2022-05-01 at 10 23 24 PM (3)](https://user-images.githubusercontent.com/35029771/178841224-acf93c0d-a095-417d-90d2-3a725350fd46.jpeg)
![pyplot](https://user-images.githubusercontent.com/35029771/178841753-70dce98f-a9b2-41ae-afb8-a4d1e792fb16.jpeg)

---
## Tools and Frameworks
**ROS Version:** Noetic

**Gazebo Version:** 11.9.1

**IDE:** VS Code <br>
*(Any IDE of youe choice)*

---
## Building

Install point-cloud to laser scan package through ros noetic
to  run gmapping using ros

Install ros-gmapping
```sh
rosrun gmapping slam_gmapping scan:=/laser_scan _base_frame:=ego_vehicle _map_update_interval:=0.5
```

Install Turtlebot3 packages

```sh
sudo apt-get install ros-noetic-turtlebot3
sudo apt-get install ros-noetic-turtlebot3-gazebo
sudo apt-get install ros-noetic-base-local-planner
sudo apt-get install ros-noetic-nav-core
sudo apt-get install ros-noetic-dwa-local-planner

```


---
## Usage

Launch the world using

```sh 
roslaunch hybrid_planner_sim  turtlebot3_world.launch
```

To see an example of planning: 

rostopic pub /move_base_simple/goal geometry_msgs/PoseStamped "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: 'base_footprint'
pose:
  position:
    x: 1
    y: 0
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1" 

---
