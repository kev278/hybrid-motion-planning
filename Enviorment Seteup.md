Clone Git library 
https://github.com/chaolmu/gazebo_models_worlds_collection
Follow readme.md
Clone https://github.com/husky/husky
Husky Simulator Installation ROS Noetic Tutorial

Dependencies

Reference: http://wiki.ros.org/LMS1xx

$ sudo apt-get install ros-noetic-lms1xx

Reference: https://bitbucket.org/DataspeedInc/ve...

$ sudo apt-get install ros-noetic-velodyne-description

Reference: http://wiki.ros.org/robot_localization

$ sudo apt-get install ros-noetic-robot-localization

Reference: http://wiki.ros.org/interactive_marke...

$ sudo apt-get install ros-noetic-interactive-marker-twist-server

Reference: http://wiki.ros.org/twist_mux

$ sudo apt-get install ros-noetic-twist-mux

Reference: http://wiki.ros.org/joy

$ sudo apt-get install ros-noetic-joy

Reference: http://wiki.ros.org/teleop_twist_joy

$ sudo apt-get install ros-noetic-teleop-twist-joy

Reference: http://wiki.ros.org/dwa_local_planner

$ sudo apt-get install ros-noetic-dwa-local-planner

Create Husky workspace

$ mkdir husky_ws

$ cd husky_ws

$ mkdir src

$ cd src

$ git clone https://github.com/gmsanchez/husky.git

$ cd ..

$ catkin_make

$ echo 'source ~/husky_ws/devel/setup.bash'>>~/.bashrc

$ source ~/.bashrc

Launching simulation

$ cd

$ export HUSKY_GAZEBO_DESCRIPTION=$(rospack find husky_gazebo)/urdf/description.gazebo.xacro

$ roslaunch husky_gazebo husky_empty_world.launch
