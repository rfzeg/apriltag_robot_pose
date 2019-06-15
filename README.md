<img src="https://raw.githubusercontent.com/rfzeg/apriltag_robot_pose/master/docs/imgs/ros_logo.png" align="right" width="101" height="27" />

# AprilTag Robot Pose
Author: Roberto Zegers R.

This ROS package implements a robot localization system using AprilTag markers. The tags used correspond to the family tag36h11, which has 587 different tags.

## Dependencies
Packages on which this package depends:
+ [apriltags_ros](https://github.com/RIVeR-Lab/apriltags_ros)

## Install

First install AprilTag and its ROS wrapper by cloning the next repository into the /src directory of your catkin_workspace:  
`$ git clone https://github.com/RIVeR-Lab/apriltags_ros.git`  

Then clone this repository:  
`$ git clone https://github.com/rfzeg/apriltag_robot_pose.git`  

Compile the package with `catkin_make` and source your workspace.

## Run

First launch a Gazebo world that includes AprilTag markers, e.g.:  
`$ roslaunch plywood_mazes maze_1_6x5.launch`  

Then spawn a robot that publishes camera images into ROS. The standard for cameras on ROS is to publish to topics such as:  
+ /image_raw - an unmodified camera image
+ /camera_info - information about the camera calibration

Next start the AprilTag detector node by executing:  
`$ roslaunch apriltag_robot_pose apriltag_detector.launch`  

Finally run the robot_pose node:  
`$ rosrun apriltag_robot_pose robot_pose.py`  

## Optional Checks

On a new terminal run this command to see the existing topics:  
`$ rostopic list`  

If everything is correct, a list of topics published by the apriltag detector node should appear:  
<img src="https://raw.githubusercontent.com/rfzeg/apriltag_robot_pose/master/docs/imgs/rostopic_list_result.png">  
Fig.1 The available topics shown by using the **rostopic list** command  

Then check that AprilTags are being detected by placing the robot's camera in front of a tag and running:

`$ rostopic echo /tag_detections`  

<img src="https://raw.githubusercontent.com/rfzeg/apriltag_robot_pose/master/docs/imgs/rostopic_echo_result.png">  
Fig.2 When a tag is detected values similar to these are displayed when running the **rostopic echo** command  

To view raw images, for instance on the topic /udacity_bot/camera1/image\_raw, use:  
`$ rosrun image_view image_view image:=/udacity_bot/camera1/image_raw`  

To check that the parameters defined in the tag\_sizes.yaml file were loaded into the param server type:
`$ rosparam get /apriltag_detector/tag_descriptions`  

## Known Issues
+ Gazebo is crashing as it is starting up: Usually, it is enough to run it again (probably several times).
+ ImportError No module named apriltags.msg: When using a custom messages, make sure the package containing it has been compiled.

This package has only been tested on Ubuntu 16.04 LTS with ROS Kinetic and Gazebo 7.15.
