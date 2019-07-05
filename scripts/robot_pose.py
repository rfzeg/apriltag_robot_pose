#!/usr/bin/python

# Estimates the absolute pose a robot in a map based on the position of the AprilTag markers in the robot's camera field of view
# Broadcasts the transform of odom w.r.t. map to correct odometry drift
# Author: Roberto Zegers R.
# Date: 2019 July

import rospy
from apriltags_ros.msg import AprilTagDetection, AprilTagDetectionArray
from geometry_msgs.msg import Pose, PoseStamped, TransformStamped, Point, Quaternion
from tf.transformations import quaternion_from_euler, euler_from_quaternion, translation_from_matrix, quaternion_from_matrix, compose_matrix, quaternion_matrix, rotation_matrix
import numpy as np
import tf2_ros
import tf2_geometry_msgs

## Global variables
nrTfRetrys = 1
retryTime = 0.05
rospy.init_node('apriltag_robot_pose', log_level=rospy.INFO, anonymous=False)

# Initializes a tf2 listener
tf_buffer = tf2_ros.Buffer(rospy.Duration(10.0)) #tf buffer length
tf_listener = tf2_ros.TransformListener(tf_buffer)

# Initializes a tf2 broadcaster for our map(parent)->odom(child) == odom w.r.t. map transform
br_odom_wrt_map = tf2_ros.TransformBroadcaster()

# Initializes an empty TransformStamped object for our map(parent)->odom(child) == odom w.r.t. map transform
ts_odom_wrt_map = TransformStamped()

r = rospy.Rate(10) # 10hz

# Initializes an empty PoseStamped object the pose of the robot_base w.r.t map
robot_pose = PoseStamped()

new_translation = []
new_rotation = []

def strip_forward_slash(frame_id):
    '''
    Removes forward slash for tf2 to work
    '''
    if frame_id[0] == "/":
        new_frame_id = frame_id[1:]
    else:
        new_frame_id = frame_id
    return new_frame_id

# get the robot's base frame
if rospy.has_param("~base_frame"):
    # forward slash must be removed to work with tf2
    base_frame = strip_forward_slash(rospy.get_param("~base_frame"))
else:
    base_frame = "base_footprint"
    rospy.logwarn("base_footprint frame is set to default: base_footprint")

# get odom frame, the (noisy) odometry
if rospy.has_param("~odom_frame"):
    odom_frame = strip_forward_slash(rospy.get_param("~odom_frame"))
else:
    odom_frame = "odom"
    rospy.logwarn("odometry frame of reference is set to default: odom")

# get world_fixed_frame
if rospy.has_param("~world_fixed_frame"):
    world_fixed_frame = strip_forward_slash(rospy.get_param("~world_fixed_frame"))
else:
    world_fixed_frame = "map"
    rospy.logwarn("world_fixed_frame frame is set to default: map")

# get the camera frame
if rospy.has_param("~camera_frame"):
    camera_frame = strip_forward_slash(rospy.get_param("~camera_frame"))
else:
    camera_frame = "camera"
    rospy.logwarn("camera frame of reference is set to default: camera")

def main():
    rospy.Subscriber("/tag_detections", AprilTagDetectionArray, apriltag_callback, queue_size = 1)
    rospy.loginfo("Started node to broadcast odom wrt map transform!")
    # get base w.r.t. odom transform
    while not rospy.is_shutdown():
        try:
            # Look for the odom->base_footprint transform
            rospy.logdebug("Looking for the odom->robot_footprint transform")
            ts_base_wrt_odom = tf_buffer.lookup_transform(odom_frame, base_frame, rospy.Time(), rospy.Duration(1.0)) # wait 1s for transform to become available
            rospy.logdebug("ts_base_wrt_odom: %s", ts_base_wrt_odom)
            # note: robot_pose (base_frame wrt map) is calculated every time the subscriber callback is executed
            broadcast_last_transform()
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException), ex:
            rospy.logerr(ex)
        r.sleep()

def pose2poselist(pose):
    ''' Transforms a pose object into the form of a python list'''
    return [pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w]

def transformPose(pose, sourceFrame, target_frame):
    '''
    Converts a pose represented as a list in the source_frame
    to a pose represented as a list in the target_frame
    '''
    _pose = PoseStamped()
    _pose.header.frame_id = sourceFrame
    if len(pose) == 6:
        pose.append(0)
        pose[3:7] = quaternion_from_euler(pose[3], pose[4], pose[5]).tolist()

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
            # converts a Pose object from its reference frame to a Pose object in the frame target_frame
            transform = tf_buffer.lookup_transform(target_frame, 
                                                   _pose.header.frame_id, #source frame is the current object's frame of reference
                                                   rospy.Time(0), #get the tf at first available time
                                                   rospy.Duration(1.0)) #wait for 1 second
            pose_transformed = tf2_geometry_msgs.do_transform_pose(_pose, transform)
            p = pose_transformed.pose.position
            o = pose_transformed.pose.orientation
            return [p.x, p.y, p.z, o.x, o.y, o.z, o.w]
        except (tf2_ros.LookupException), e1:
            print("ERROR: LookupException!")
            rospy.logerr(e1)
            rospy.logwarn("No tf frame with name %s found. Check that the detected tag ID is part of the transforms that are being broadcasted by the static transform broadcaster.", target_frame)
            continue
        except (tf2_ros.ConnectivityException), e2:
            rospy.logwarn(e2)
            rospy.logerr("ERROR: ConnectivityException!")
            continue
        except (tf2_ros.ExtrapolationException), e3:
            rospy.logwarn(e3)
            rospy.logerr("ERROR: ExtrapolationException!")
            continue
        except Exception as e4:
            rospy.logwarn(e4)
            rospy.logerr("Unexpected error when transforming Pose")
        finally:
            rospy.sleep(retryTime)
    return None

def xyzquat_from_matrix(matrix):
    return translation_from_matrix(matrix).tolist() + quaternion_from_matrix(matrix).tolist()

def matrix_from_xyzquat(arg1, arg2=None):
    return matrix_from_xyzquat_np_array(arg1, arg2).tolist()

def matrix_from_xyzquat_np_array(arg1, arg2=None):
    if arg2 is not None:
        translate = arg1
        quaternion = arg2
    else:
        translate = arg1[0:3]
        quaternion = arg1[3:7]

    return np.dot(compose_matrix(translate=translate),quaternion_matrix(quaternion))

def invPoselist(poselist):
    return xyzquat_from_matrix(np.linalg.inv(matrix_from_xyzquat(poselist)))

def base_wrt_map_pose(pose=[0,0,0,0,0,0,1], child_frame_id='obj', parent_frame_id = world_fixed_frame, npub=1):
    '''
    Converts from a representation of a pose as a list to a TransformStamped object (translation and rotation (Quaternion) representation)
    Then keeps that as a TransformStamped object
    Note:
    In Rviz it will be shown as an arrow from the robot base (child) to the map (parent)
    In RQT it will be shown as an arrow from the map (parent) to the robot base (child)
    '''
    global robot_pose
    if len(pose) == 7:
        quaternion = tuple(pose[3:7])
    elif len(pose) == 6:
        quaternion = quaternion_from_euler(*pose[3:6])
    else:
        rospy.logerr("Bad length of pose")
        return None

    position = tuple(pose[0:3])
    # Fill in PoseStamped object: stamps the transform with the current time
    robot_pose.header.stamp = rospy.Time.now()
    # Sets the frame ID of the transform to the map frame
    robot_pose.header.frame_id = parent_frame_id
    # Fill in coordinates
    if len(pose) == 6:
        pose.append(0)
        pose[3:7] = quaternion_from_euler(pose[3], pose[4], pose[5]).tolist()

    robot_pose.pose.position.x = pose[0]
    robot_pose.pose.position.y = pose[1]
    robot_pose.pose.position.z = pose[2]
    robot_pose.pose.orientation.x = pose[3]
    robot_pose.pose.orientation.y = pose[4]
    robot_pose.pose.orientation.z = pose[5]
    robot_pose.pose.orientation.w = pose[6]

def averagePose(pose_list):
    '''
    Calculates the averge pose from a list of poses
    Position is the average of all estimated positions
    Orientation uses the orientation of the first detected marker
    '''
    avg_pose = []
    avg_pose.append(np.mean([pose[0] for pose in pose_list]))
    avg_pose.append(np.mean([pose[1] for pose in pose_list]))
    avg_pose.append(np.mean([pose[2] for pose in pose_list]))
    # Use the orientation of the first detected marker
    avg_pose.extend(pose_list[0][3:7])
    return avg_pose

def apriltag_callback(data):
    # rospy.logdebug(rospy.get_caller_id() + "I heard %s", data)
    if data.detections:
        poselist_base_wrt_map = []
        for detection in data.detections:
            tag_id = detection.id  # tag id
            rospy.logdebug("Tag ID detected: %s \n", tag_id)
            child_frame_id = "tag_" + str(tag_id)

            # Convert the deteced tag Pose object to tag pose representation as a list, only for convinience
            poselist_tag_wrt_camera = pose2poselist(detection.pose)
            rospy.logdebug("poselist_tag_wrt_camera: \n %s \n", poselist_tag_wrt_camera)

            # Calculate transform of tag w.r.t. robot base (in Rviz arrow points from tag (child) to robot base(parent))
            poselist_tag_wrt_base = transformPose(poselist_tag_wrt_camera, camera_frame, base_frame)
            rospy.logdebug("transformPose(poselist_tag_wrt_camera, 'camera', 'robot_footprint'): \n %s \n", poselist_tag_wrt_base)

            # Calculate transform of robot base w.r.t. tag (in Rviz arrow points from robot base (child) to tag(parent))
            poselist_base_wrt_tag = invPoselist(poselist_tag_wrt_base)
            rospy.logdebug("invPoselist( poselist_tag_wrt_base): \n %s \n", poselist_base_wrt_tag)

            # Calculate transform of robot base w.r.t. map (in Rviz arrow points from robot base (child) to map (parent)), returns pose of robot in the map coordinates
            poselist_base_wrt_map.append(transformPose(poselist_base_wrt_tag, child_frame_id, target_frame = world_fixed_frame))
            rospy.logdebug("transformPose(poselist_base_wrt_tag, sourceFrame = '%s', target_frame = 'map'): \n %s \n", child_frame_id, poselist_base_wrt_map[-1])

        #for counter, robot_pose in enumerate(poselist_base_wrt_map):
        #    rospy.logdebug("\n Robot pose estimation nr. %s: %s \n",str(counter), robot_pose)

        estimated_avg_pose = averagePose(poselist_base_wrt_map)
        rospy.logdebug("\n Robot's estimated avg. pose from all AR tags detected:\n %s \n", estimated_avg_pose)

        # Calculate transform of robot base w.r.t. map or pose of robot in the map coordinates
        base_wrt_map_pose(pose = estimated_avg_pose, child_frame_id = base_frame, parent_frame_id = world_fixed_frame)

        map_to_odom_transform()

def convert_pose_inverse_transform(input_pose):
        """ Helper method to invert a transform (this is built into the tf C++ classes, but ommitted from Python) """
        translation = np.zeros((4,1))
        translation[0] = -input_pose.pose.position.x
        translation[1] = -input_pose.pose.position.y
        translation[2] = -input_pose.pose.position.z
        translation[3] = 1.0

        rotation = (input_pose.pose.orientation.x, input_pose.pose.orientation.y, input_pose.pose.orientation.z, input_pose.pose.orientation.w)
        euler_angle = euler_from_quaternion(rotation)
        rotation = np.transpose(rotation_matrix(euler_angle[2], [0,0,1]))       # the angle is a yaw
        transformed_translation = rotation.dot(translation)

        translation = (transformed_translation[0], transformed_translation[1], transformed_translation[2])
        rotation = quaternion_from_matrix(rotation)
        return (translation, rotation)

def convert_translation_rotation_to_pose(translation, rotation):
        """ Convert from representation of a pose as translation and rotation (Quaternion) tuples to a geometry_msgs/Pose message """
        return Pose(position=Point(x=translation[0],y=translation[1],z=translation[2]), orientation=Quaternion(x=rotation[0],y=rotation[1],z=rotation[2],w=rotation[3]))

def map_to_odom_transform():
        """ This method constantly updates the offset of the map and
            odometry coordinate systems based on the latest results from
            the localizer
        """
        global new_translation, new_rotation
        (translation, rotation) = convert_pose_inverse_transform(robot_pose)
        map_wrt_base_pose = PoseStamped(pose=convert_translation_rotation_to_pose(translation,rotation))
        map_wrt_base_pose.header.stamp=rospy.Time.now()
        map_wrt_base_pose.header.frame_id=base_frame
        # access/look-up the transform odom (target/new frame) wrt base(source/current frame):
        # tf_buffer.lookup_transform(target_frame, source_frame, query_time, timeout_secs)
        ts_odom_wrt_base = tf_buffer.lookup_transform(odom_frame, base_frame, rospy.Time(), rospy.Duration(1.0)) # wait 1s for transform to become available
        map_wrt_odom_pose = tf2_geometry_msgs.do_transform_pose(map_wrt_base_pose, ts_odom_wrt_base)
        # invert map_wrt_odom_to get odom_wrt_map and store as tuple position, quaternion
        (new_translation, new_rotation) = convert_pose_inverse_transform(map_wrt_odom_pose)

def broadcast_last_transform():
        """ Make sure that we are always broadcasting the last
            map to odom transformation.  This is necessary so
            move_base can work properly. """
        # Sets the frame ID of the transform to the map frame
        ts_odom_wrt_map.header.frame_id = world_fixed_frame
        # Stamps the transform with the current time
        ts_odom_wrt_map.header.stamp = rospy.Time.now()
        # Sets the child frame ID to odom
        ts_odom_wrt_map.child_frame_id = odom_frame
        if new_translation:
            # Fill in coordinates
            list(new_translation) # convert an np array to an ordinary list
            ts_odom_wrt_map.transform.translation.x = new_translation[0][0]
            ts_odom_wrt_map.transform.translation.y = new_translation[1][0]
            ts_odom_wrt_map.transform.translation.z = new_translation[2][0]
            ts_odom_wrt_map.transform.rotation.x = new_rotation[0]
            ts_odom_wrt_map.transform.rotation.y = new_rotation[1]
            ts_odom_wrt_map.transform.rotation.z = new_rotation[2]
            ts_odom_wrt_map.transform.rotation.w = new_rotation[3]
            try:
                br_odom_wrt_map.sendTransform(ts_odom_wrt_map)
                rospy.logdebug("Broadcasted odom wrt map transform!")
            except Exception as exc:
                rospy.logwarn(exc)
                rospy.logerr("Unexpected error in broadcast of map to odom transformation")

if __name__=='__main__':
    main()
