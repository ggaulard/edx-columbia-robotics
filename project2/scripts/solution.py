#!/usr/bin/env python

import rospy

import tf
import tf2_ros
import math

from geometry_msgs.msg import Pose, Transform, TransformStamped

def message_from_transform(m):
    r = Transform()

    t = tf.transformations.translation_from_matrix(m)
    r.translation.x = t[0]
    r.translation.y = t[1]
    r.translation.z = t[2]

    q = tf.transformations.quaternion_from_matrix(m)
    r.rotation.x = q[0]
    r.rotation.y = q[1]
    r.rotation.z = q[2]
    r.rotation.w = q[3]
    
    return r

def transform_base_to_object():
    T1 = tf.transformations.concatenate_matrices(
	    tf.transformations.translation_matrix((0.0,1.0,1.0)),
	    tf.transformations.quaternion_matrix(tf.transformations.quaternion_from_euler(0.79,0.0,0.79))
    )
    T1_stamped = TransformStamped()
    T1_stamped.header.stamp = rospy.Time.now()
    T1_stamped.header.frame_id = "world"
    T1_stamped.child_frame_id = "robot"
    T1_stamped.transform = message_from_transform(T1)
    return T1_stamped

def ros_start():
    rate = rospy.Rate(10)
    br = tf.TransformBroadcaster()
    while not rospy.is_shutdown():
        br.sendTransformMessage(transform_base_to_object())
        rate.sleep()

if __name__=="__main__":
    rospy.init_node("simple_marker")
    ros_start()