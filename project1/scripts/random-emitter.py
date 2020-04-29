#!/usr/bin/env python  
import rospy
import random

from project1.msg import TwoInts

def randomEmitter():
    pub = rospy.Publisher('two_ints', TwoInts, queue_size=10)
    rospy.init_node('RandomEmitter', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        twoInts = TwoInts()
        twoInts.a = random.randint(0,100)
        twoInts.b = random.randint(0,100)
        pub.publish(twoInts)
        rate.sleep()

if __name__ == "__main__":
    try:
        randomEmitter()
    except rospy.ROSInterruptException:
        pass