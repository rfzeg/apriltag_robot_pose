#!/usr/bin/python

# Calculate the absolute pose a robot based on the position of detected AprilTag markers
# Author: Roberto Zegers R.
# Date: 2019 June

import rospy
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseStamped
import tf.transformations as tfm
import numpy as np
import tf

nrTfRetrys = 1
retryTime = 0.05
rospy.init_node('apriltag_robot_pose', anonymous=True)
# tf listener
lr = tf.TransformListener()

def main():
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size = 1)
    rospy.sleep(1)
    rospy.spin()

def pose2poselist(pose):
    return [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]

def transformPose(lr, pose, sourceFrame, targetFrame):
    _pose = PoseStamped()
    _pose.header.frame_id = sourceFrame
    if len(pose) == 6:
        pose.append(0)
        pose[3:7] = tfm.quaternion_from_euler(pose[3], pose[4], pose[5]).tolist()

    _pose.pose.position.x = pose[0]
    _pose.pose.position.y = pose[1]
    _pose.pose.position.z = pose[2]
    _pose.pose.orientation.x = pose[3]
    _pose.pose.orientation.y = pose[4]
    _pose.pose.orientation.z = pose[5]
    _pose.pose.orientation.w = pose[6]

    for i in range(nrTfRetrys):
        try:
            t = rospy.Time(0)
            _pose.header.stamp = t
            _pose_target = lr.transformPose(targetFrame, _pose)
            p = _pose_target.pose.position
            o = _pose_target.pose.orientation
            return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
        except Exception as ex:
            rospy.logwarn(ex.message)
            rospy.sleep(retryTime)

    return None

def xyzquat_from_matrix(matrix):
    return tfm.translation_from_matrix(matrix).tolist() + tfm.quaternion_from_matrix(matrix).tolist()

def matrix_from_xyzquat(arg1, arg2=None):
    return matrix_from_xyzquat_np_array(arg1, arg2).tolist()

def matrix_from_xyzquat_np_array(arg1, arg2=None):
    if arg2 is not None:
        translate = arg1
        quaternion = arg2
    else:
        translate = arg1[0:3]
        quaternion = arg1[3:7]

    return np.dot(tfm.compose_matrix(translate=translate) ,
                   tfm.quaternion_matrix(quaternion))

def invPoselist(poselist):
    return xyzquat_from_matrix(np.linalg.inv(matrix_from_xyzquat(poselist)))

def apriltag_callback(data):
    # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data)
    if data.detections:
        for detection in data.detections:
            tag_id = detection.id  # tag id

            # TO-DO: add check that detected tag is part of broadcasted transforms

            child_frame_id = "/tag_" + str(tag_id)

            rospy.loginfo("Tag ID: %s", tag_id)
            rospy.loginfo("Detected pose: \n %s", detection.pose)

            poselist_tag_camera = pose2poselist(detection.pose)
            rospy.loginfo("poselist_tag_camera: \n %s", poselist_tag_camera) # for debug

            poselist_tag_base = transformPose(lr, poselist_tag_camera, '/camera', '/robot_footprint')
            print "transformPose(lr, poselist_tag_camera, '/camera', '/robot_footprint'):", poselist_tag_base, '\n' # for debug

            poselist_base_tag = invPoselist(poselist_tag_base)
            print 'invPoselist(poselist_tag_base):', poselist_base_tag, '\n' # for debug

            poselist_base_map = transformPose(lr, poselist_base_tag, child_frame_id, targetFrame = '/map')
            print "transformPose(lr, poselist_base_tag, sourceFrame = '/apriltag', targetFrame = '/map'):", poselist_base_map, '\n' # for debug

            rospy.loginfo("Robot pose: %s", poselist_base_map)

if __name__=='__main__':
    main()
