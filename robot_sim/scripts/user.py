#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Pose2D

def listener():
    rospy.init_node('user', anonymous=True)
    rospy.Subscriber('Debugging', Pose2D)
    rospy.spin()

if __name__ == '__main__':
    listener()
