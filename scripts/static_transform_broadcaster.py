#!/usr/bin/env python

# Broadcasts static transforms
# Author: Roberto Zegers R.
# Date: 2019 June

import rospy

# Because of transformations
import tf_conversions

import tf2_ros
import geometry_msgs.msg

def broadcast_pose():
    br = tf2_ros.TransformBroadcaster()
    t = geometry_msgs.msg.TransformStamped()

    t.header.stamp = rospy.Time.now()
    t.header.frame_id = "world"
    t.child_frame_id = "tag_0"
    t.transform.translation.x = 0.5
    t.transform.translation.y = 0.011
    t.transform.translation.z = 0.3
    q = tf_conversions.transformations.quaternion_from_euler(1.5708, 0, 3.14159)
    t.transform.rotation.x = q[0]
    t.transform.rotation.y = q[1]
    t.transform.rotation.z = q[2]
    t.transform.rotation.w = q[3]

    br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tag_broadcaster')
    rospy.loginfo("Initialized static transform broadcaster")
    rate = rospy.Rate(10) # 10hz or 100 ms
    while not rospy.is_shutdown():
        broadcast_pose()
        rate.sleep()
