<img src="https://raw.githubusercontent.com/rfzeg/apriltag_robot_pose/master/docs/imgs/ros_logo.png" align="right" width="101" height="27" /> 
 
## AprilTag Robot Pose
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

**Note:**  
The package requires to spawn a robot that publishes camera images into ROS. The standard for cameras on ROS is to publish to topics such as:  
+ /image_raw - an unmodified camera image
+ /camera_info - information about the camera calibration

## Run

Start all neccesary components by executing the included bash script (xterm required):  
`./start_demo.sh`

Or alternatively start the demo launching every node manually:  

1.  **Fire up Gazebo world that includes AprilTag markers, e.g.:**  
    `roslaunch plywood_mazes maze_3_6x6.launch`
2.  **In a new terminal load a robot URDF file to the parameter server, e.g.:**  
    `$ roslaunch udacity_bot robot_description.launch`
3.  **And spawn the robot model, e.g.:**  
    `$ roslaunch udacity_bot spawn_udacity_bot.launch`
4.  **In a new window kick off the static transform broadcaster node:**  
    `$ roslaunch apriltag_robot_pose static_transforms.launch`
5.  **Then launch the AprilTag detector node to detect AR markers in the camera image:**  
    `$ roslaunch apriltag_robot_pose apriltag_detector.launch`
6.  **Next execute the robot pose estimator node:**  
    `$ rosrun apriltag_robot_pose robot_pose.py`
7.  **In order to see the robot pose estimator node in action open RViz:**  
    `$ roslaunch apriltag_robot_pose rviz.launch`

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

To look at the numeric values of a transform between the map frame and any specific AR marker tag:  
`$ rosrun tf tf_echo map tag_0`  
or visualize the complete tf tree using RQT:  
`$ rosrun rqt_tf_tree rqt_tf_tree`  

If you would like the apriltag_robot_pose node to display output at the DEBUG verbosity level use:  
`$rosservice call /apriltag_robot_pose/set_logger_level "{logger: 'rosout', level: 'debug'}"`  

## Troubleshooting
+ Gazebo is crashing as it is starting up: Usually, it is enough to run it again (probably several times).
+ ImportError No module named apriltags.msg: When using custom messages, make sure the packages containing them have been compiled.

This package has only been tested on Ubuntu 16.04 LTS with ROS Kinetic and Gazebo 7.15.

## Resources
+ http://wiki.ros.org/tf2/Tutorials/Writing%20a%20tf2%20broadcaster%20%28Python%29
