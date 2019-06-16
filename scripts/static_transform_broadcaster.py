#!/usr/bin/env python

# Broadcasts static transforms. Used here to publish multiple map frame to tag frame transforms.
# Author: Roberto Zegers R.
# Date: 2019 June

import rospy
import tf_conversions # because of transformations
import tf2_ros
import geometry_msgs.msg
import yaml

def broadcast_pose(tag_info):
    br = tf2_ros.TransformBroadcaster()

    for tag_id, tf_data in tag_info.iteritems():
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = rospy.Time.now()
        t.header.frame_id = "map"
        t.child_frame_id = "tag_" + str(tag_id)
        t.transform.translation.x = tf_data[0]
        t.transform.translation.y = tf_data[1]
        t.transform.translation.z = tf_data[2]
        q = tf_conversions.transformations.quaternion_from_euler(tf_data[3], tf_data[4], tf_data[5])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        br.sendTransform(t)

if __name__ == '__main__':
    rospy.init_node('tag_broadcaster')
    rospy.loginfo("\n Initialized static transform broadcaster \n")

    ## Load tag_info parameters from yaml file
    # Get a parameter from our private namespace
    param_path = rospy.get_param("~tag_poses_param_path")
    rospy.loginfo("Tag poses broadcasted: %s", param_path)
    f = open(param_path, 'r')
    params_raw = f.read()
    f.close()

    params = yaml.load(params_raw)
    # Dictionary with tag poses: [x, y, z, Roll, Pitch, Yaw]
    tag_info = params['tag_info'] 
    #rospy.loginfo("Tag info dict: %s", tag_info)

    rate = rospy.Rate(10) # 10hz or 100 ms

    while not rospy.is_shutdown():
        broadcast_pose(tag_info)
        rate.sleep()
