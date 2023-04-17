#!/usr/bin/env python


#takes in PointCloud2 message from Depth Camera sensor and forwards to cloud_in topic for Octomap mapping
import rospy
import tf
import math
import numpy as np
import std_msgs.msg
from nav_msgs.msg import Odometry
from sensor_msgs.msg import PointCloud2


class Messenger():
    def __init__(self):
        self.msg_pub = rospy.Publisher('cloud_in', PointCloud2,queue_size=1)
    
    def callback(self,data):
        self.msg_pub.publish(data)


if __name__ == '__main__':
    rospy.init_node('Trajectory', anonymous=True)
    r = rospy.Rate(1)  #hz
    
    rospy.loginfo('Program Started')
    
    myMessenger = Messenger()



    while not rospy.is_shutdown():
        camDepth_sub = rospy.Subscriber("/red/camera/depth_registered/points", PointCloud2, myMessenger.callback)
        r.sleep()