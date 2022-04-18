git clone https://github.com/chaolmu/gazebo_models_worlds_collection
cd gazebo_models_worlds_collection
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:gazebo_models_worlds_collection/models
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:gazebo_models_worlds_collection/worlds
#Test 1
rosrun gazebo_ros gazebo small_city.world

#Huskysetup
sudo apt-get update
sudo apt-get install ros-noetic-husky-desktop
sudo apt-get install ros-noetic-husky-simulator

#Husky world Test
roslaunch husky_gazebo spawn_husky.launch
roslaunch husky_viz view_robot.launch