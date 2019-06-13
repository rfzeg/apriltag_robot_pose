# AprilTag Robot Pose
Author: Roberto Zegers R.

This ROS package implements a robot localization system using AprilTag markers.

## Dependencies
Packages on which this package depends:
+ [apriltags_ros](https://github.com/RIVeR-Lab/apriltags_ros)

## Install

First install AprilTag and its ROS wrapper by cloning the next repository into the /src directory of your catkin_workspace:  
`$ git clone https://github.com/RIVeR-Lab/apriltags_ros.git`  

Then clone this repository:  
`$ git clone https://github.com/rfzeg/apriltag_robot_pose.git`  

Compile the package with `catkin_make`.

## Run

First launch a Gazebo world that includes AprilTag markers, e.g.:  
`$ roslaunch plywood_mazes maze_1_6x5.launch`  

Then spawn a robot that publishes camera images into ROS. The standard for cameras on ROS is to publish to topics such as:  
+ /image_raw - an unmodified camera image
+ /camera_info - information about the camera calibration

Finally start the AprilTag detector node by executing:  
`$ roslaunch apriltag_robot_pose apriltag_detector.launch`  

## Optional Checks

On a new terminal run this command to see the existing topics:

`$ rostopic list`  

A set of topics published by the apriltag detector node should appear:
<img src="https://raw.githubusercontent.com/rfzeg/apriltag_robot_pose/master/docs/imgs/rostopic_list_result.png">
Fig.1 The available topics shown by using the **rostopic list** command  

Then check that AprilTags are being detected by placing the robot's camera in front of a tag and running:

`$ rostopic echo /tag_detections`  

<img src="https://raw.githubusercontent.com/rfzeg/apriltag_robot_pose/master/docs/imgs/rostopic_echo_result.png">
Fig.2 The detected tags shown by using the **rostopic echo** command  

## Known Issues
+ Gazebo is crashing as it is starting up: Usually, it is enough to run it again (probably several times).

This package has only been tested on Ubuntu 16.04 LTS with ROS Kinetic and Gazebo 7.15.

