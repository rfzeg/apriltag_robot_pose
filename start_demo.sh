#!/bin/bash

echo ""
echo "ROS AR Tag Robot Pose Estimator Demo"
echo ""
# The -e option to xterm is useful for running a single command and then exiting (or closing xterm after a Ctrl+C)

# Fire up Gazebo world with maze environment
xterm -e "roslaunch plywood_mazes maze_3_6x6.launch" &
sleep 6

# Prompt for a key press to continue after Gazebo has loaded
#read -n 1 -r -s -p "Press any key to continue once Gazebo has loaded or Ctrl+C to abort..."
#echo ""

# Allow for noisy odometry choice
echo ""
read -p "After Gazebo has fully loaded, do you want to start a noisy odometry node? (y/n): " input_choice_1

if [ "$input_choice_1" = "y" ]
then
  # Load robot description to parameter server and spawn a robot
  xterm -e "roslaunch rtab_dumpster spawn_rtab_dumpster.launch odometryTopic:=odom_perfect" &
  #xterm -e "roslaunch udacity_bot spawn_udacity_bot.launch odometryTopic:=odom_perfect" &
  sleep 4

  # Start noisy odometry node
  xterm -e "roslaunch noisy_odometry noisy_odometry.launch" &
  sleep 4

elif [ "$input_choice_1" = "n" ]
then
  # Load robot description to parameter server and spawn a robot
  xterm -e "roslaunch rtab_dumpster spawn_rtab_dumpster.launch" &
  sleep 4
  echo ""
else
  echo ""
  echo "Warning: Not an acceptable option. Do you want to start a noisy odometry node? Choose (y/n)"
  echo ""
fi

# Kick off static transform broadcaster node
xterm -e "roslaunch apriltag_robot_pose static_transforms.launch" &
sleep 2

# Launch AprilTag detector node to detect AR markers in space
xterm -e "roslaunch apriltag_robot_pose apriltag_detector.launch image_topic:=/camera/rgb/image_raw info_topic:=/camera/rgb/camera_info" &
sleep 4

# Execute the robot pose estimator node.
xterm -hold -e "roslaunch apriltag_robot_pose robot_pose.launch camera_frame:=/camera_rgbd_frame" &
sleep 2

# Allow for Rviz choice
echo ""
read -p "Do you want to start RVIZ with a preconfigured view (y/n): " input_choice_2

if [ "$input_choice_2" = "y" ]
then
  # Start RVIZ
  xterm -e "roslaunch apriltag_robot_pose rviz.launch" &
  sleep 4
  # Load pre-made map using map server
  xterm -e "roslaunch plywood_mazes map_server_maze_3.launch" &
  echo ""
elif [ "$input_choice_2" = "n" ]
then
  echo ""
  echo "Rviz *NOT* started!"
  echo ""
else
  echo ""
  echo "Warning: Not an acceptable option. Choose (y/n)"
  echo ""
fi
