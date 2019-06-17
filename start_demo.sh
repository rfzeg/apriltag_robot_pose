#!/bin/sh
echo ""
echo "ROS AR Tag Robot Pose Estimator Demo"
echo ""
# The -e option to xterm is useful for running a single command and then exiting (or closing xterm after a ctrl+C)

# Fire up Gazebo world with maze environment
xterm -e "roslaunch plywood_mazes maze_3_6x6.launch" &
sleep 6

# Load robot description to parameter server
xterm -e "roslaunch udacity_bot robot_description.launch" &
sleep 2

# Spawn a robot
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

# Start RVIZ with a preconfigured view
xterm -e "roslaunch apriltag_robot_pose rviz.launch"
