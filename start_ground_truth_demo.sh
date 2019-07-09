#!/bin/bash

echo ""
echo "ROS Robot Pose Ground Truth Demo"
echo ""
# The -e option to xterm is useful for running a single command and then exiting (or closing xterm after a Ctrl+C)

# Fire up Gazebo world with maze environment
xterm -e "roslaunch plywood_mazes maze_3_6x6.launch" &
sleep 6

# Prompt for a key press to continue after Gazebo has loaded
read -n 1 -r -s -p "Press any key to continue once Gazebo has loaded or Ctrl+C to abort..."
echo ""

# Load robot description to parameter server and spawn a robot
xterm -e "roslaunch rtab_dumpster spawn_rtab_dumpster.launch diff_drive_publishTf:=false diff_drive_publishOdomTF:=false" &
sleep 4

# Get ground truth odometry from Gazebo and publish as odom msg
xterm -e "roslaunch noisy_odometry odom_msg_from_gazebo.launch" &
sleep 4

# Broadcast odom w.r.t. map Tf from odom msg
xterm -e "roslaunch noisy_odometry odom_msg_to_map_tf.launch" &
sleep 4

# Allow for Rviz choice
echo ""
read -p "Do you want to start RVIZ with a preconfigured view (y/n): " input_choice

if [ "$input_choice" = "y" ]
then
  # Start RVIZ
  xterm -e "roslaunch apriltag_robot_pose rviz.launch" &
  sleep 4
  # Load pre-made map using map server
  xterm -e "roslaunch plywood_mazes map_server_maze_3.launch"
elif [ "$input_choice" = "n" ]
then
  echo ""
  echo "Rviz *NOT* started!"
  echo ""
else
  echo ""
  echo "Warning: Not an acceptable option. Choose (y/n)"
  echo ""
fi
