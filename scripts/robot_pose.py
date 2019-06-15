#!/usr/bin/python

# Calculate the absolute pose a robot based on the position of detected AprilTag markers
# Author: Roberto Zegers R.
# Date: 2019 June

import rospy
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Pose

rospy.init_node('apriltag_robot_pose', anonymous=True)

def main():
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size = 1)
    rospy.sleep(1)
    rospy.spin()

def apriltag_callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    for detection in data.detections:
        tag_id = detection.id  # tag id
        p = Pose()
        p.position.x = detection.pose.pose.position.x
        p.position.y = detection.pose.pose.position.y
        p.position.z = detection.pose.pose.position.z
        p.orientation = detection.pose.pose.orientation
        rospy.loginfo("Tag ID: %s", tag_id)
        rospy.loginfo("Detected pose: \n %s", p)

if __name__=='__main__':
    main()
