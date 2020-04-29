#!/usr/bin/env python

import rospy

import tf
import tf2_ros
import math

from interactive_markers.interactive_marker_server import *
from visualization_msgs.msg import *

def createRobot():
    m = Marker()  
    m.header.frame_id = "/world_frame"
    m.ns = "robot"
    m.id = 0
    m.type = Marker.CUBE
    m.scale.x = 0.5
    m.scale.y = 0.5
    m.scale.z = 0.5
    m.color.r = 0.0
    m.color.g = 0.9
    m.color.b = 0.0
    m.color.a = 1.0

    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 0

    return m

def createCamera():
    m = Marker()  
    m.header.frame_id = "/world_frame"
    m.ns = "camera"
    m.id = 1
    m.type = Marker.ARROW
    m.scale.x = 0.8
    m.scale.y = 0.4
    m.scale.z = 0.4
    m.color.r = 0.0
    m.color.g = 0.9
    m.color.b = 0.0
    m.color.a = 1.0

    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 1

    return m

def createObject():
    m = Marker()  
    m.header.frame_id = "/world_frame"
    m.ns = "object"
    m.id = 2
    m.type = Marker.CYLINDER
    m.scale.x = 0.4
    m.scale.y = 0.4
    m.scale.z = 0.4
    m.color.r = 0.0
    m.color.g = 0.9
    m.color.b = 0.0
    m.color.a = 1.0

    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 2

    return m

def createBase():
    m = Marker()  
    m.header.frame_id = "/world_frame"
    m.ns = "base_x"
    m.id = 90
    m.type = Marker.ARROW
    m.scale.x = 1.0
    m.scale.y = 0.1
    m.scale.z = 0.1
    m.color.r = 1.0
    m.color.g = 0.0
    m.color.b = 0.0
    m.color.a = 1.0

    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0.0,0.0,0.0)
    m.pose.orientation.x = q[0]
    m.pose.orientation.y = q[1]
    m.pose.orientation.z = q[2]
    m.pose.orientation.w = q[3]

    m1 = m

    m = Marker()  
    m.header.frame_id = "/world_frame"
    m.ns = "base_y"
    m.id = 91
    m.type = Marker.ARROW
    m.scale.x = 1.0
    m.scale.y = 0.1
    m.scale.z = 0.1
    m.color.r = 0.0
    m.color.g = 1.0
    m.color.b = 0.0
    m.color.a = 1.0

    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0.0,0.0,math.pi/2)
    m.pose.orientation.x = q[0]
    m.pose.orientation.y = q[1]
    m.pose.orientation.z = q[2]
    m.pose.orientation.w = q[3]

    m2 = m

    m = Marker()  
    m.header.frame_id = "/world_frame"
    m.ns = "base_z"
    m.id = 92
    m.type = Marker.ARROW
    m.scale.x = 1.0
    m.scale.y = 0.1
    m.scale.z = 0.1
    m.color.r = 0.0
    m.color.g = 0.0
    m.color.b = 1.0
    m.color.a = 1.0

    m.pose.position.x = 0
    m.pose.position.y = 0
    m.pose.position.z = 0
    q = tf.transformations.quaternion_from_euler(0.0,-math.pi/2,0.0)
    m.pose.orientation.x = q[0]
    m.pose.orientation.y = q[1]
    m.pose.orientation.z = q[2]
    m.pose.orientation.w = q[3]

    m3 = m

    return [m1,m2,m3]

def publishStaticScene():

    # transform = geometry_msgs.msg.Transform()

    robot = createRobot()
    camera = createCamera()
    obj = createObject()
    base = createBase()

    pub_marker = rospy.Publisher('/visualization_marker', Marker, queue_size=10)
    pub_position_base = rospy.Publisher('/position/base', Marker, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pub.publish(robot)
        pub.publish(camera)
        pub.publish(obj)
        pub.publish(base[0])
        pub.publish(base[1])
        pub.publish(base[2])
        rate.sleep()

if __name__=="__main__":
    rospy.init_node("simple_marker")
    # publishInteractiveCube()
    # rospy.spin()

    publishStaticScene()
    
    