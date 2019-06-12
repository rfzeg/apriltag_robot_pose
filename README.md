# AprilTag Robot Pose 
This ROS package implements robot localization using AprilTag markers.

## Install

First install AprilTag and its ROS wrapper by cloning this repository into the src directory of your catkin_workspace:  
`$ git clone https://github.com/RIVeR-Lab/apriltags_ros.git`  

Then clone this repository:  
`$ git clone https://github.com/rfzeg/apriltag_robot_pose.git`  

Then build.

## Run

First launch a Gazebo world that includes AprilTag markers, e.g.:  
`$ roslaunch plywood_mazes maze_1_6x5.launch`  

Spawn a robot that publishes camera images into ROS. The standard for cameras on ROS is to publish to topics such as:  
+ /image_raw - an unmodified camera image
+ /camera_info - information about the camera calibration

