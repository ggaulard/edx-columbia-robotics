#!/usr/bin/env python  
import rospy

from project1.msg import TwoInts
from std_msgs.msg import Int16

pub = None

def solutionCallback(data):
    global pub
    pub.publish(data.a + data.b)

def solution():
    global pub
    rospy.init_node('RandomAdd', anonymous=True)
    pub = rospy.Publisher('sum', Int16, queue_size=10)
    sub = rospy.Subscriber("two_ints", TwoInts, solutionCallback)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == "__main__":
    try:
        solution()
    except rospy.ROSInterruptException:
        pass