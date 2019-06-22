#!/bin/bash

echo ""
echo "ROS AR Tag Robot Pose Estimator Demo"
echo ""
# The -e option to xterm is useful for running a single command and then exiting (or closing xterm after a ctrl+C)

# Fire up Gazebo world with maze environment
xterm -e "roslaunch plywood_mazes maze_3_6x6.launch" &
sleep 6

# Prompt for a key press to continue after Gazebo has loaded
read -n 1 -r -s -p "Press any key to continue once Gazebo has loaded or Ctrl+C to abort..."

# Load robot description to parameter server and spawn a robot
xterm -e "roslaunch udacity_bot spawn_udacity_bot.launch" &
sleep 4

# Kick off static transform broadcaster node
xterm -e "roslaunch apriltag_robot_pose static_transforms.launch" &
sleep 2

# Launch AprilTag detector node to detect AR markers in space
xterm -e "roslaunch apriltag_robot_pose apriltag_detector.launch" &
sleep 4

# Execute the robot pose estimator node. The window will stay open until the user presses X.
xterm -hold -e "rosrun apriltag_robot_pose robot_pose.py" &
sleep 2

# Allow for Rviz choice
read -p "Do you want to start RVIZ with a preconfigured view (y/n): " input_choice

if [ "$input_choice" = "y" ]
then
  # Start RVIZ
  xterm -e "roslaunch apriltag_robot_pose rviz.launch"
elif [ "$input_choice" = "n" ]
then
  echo "Rviz *NOT* started!"
  break
else
  echo "Warning: Not an acceptable option. Choose (y/n).
          "
fi
