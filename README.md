# RobustMotionPlanning

 install point-cloud to laser scan package through ros noetic
to  run gmapping using ros

install ros-gmapping


rosrun gmapping slam_gmapping scan:=/laser_scan _base_frame:=ego_vehicle _map_update_interval:=0.5


## Running the simulation

Install Turtlebot3 packages
apt-get install ros-noetic-turtlebot3
apt-get install ros-noetic-turtlebot3_gazebo

```sh 
roslaunch robust_planner  turtlebot3_world.launch
```

To see example of planning: 

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


