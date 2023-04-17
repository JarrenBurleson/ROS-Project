#!/usr/bin/env python

import rospy
import tf
import math
import numpy as np
import std_msgs.msg
from geometry_msgs.msg import Transform, Quaternion, Point, Twist
from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

class Trajectory():
    def __init__(self):

        self.msg = MultiDOFJointTrajectoryPoint()
        self.header = std_msgs.msg.Header()
        self.header.frame_id = 'frame'
        self.quaternion = tf.transformations.quaternion_from_euler(0,0,0)
        self.velocities = Twist()
        self.accelerations = Twist()


    def update(self):
            for i in range(0,10):
                x = 0
                y = 0
                z = i+1
                print(x,y,z)
                transforms = Transform(translation=Point(x,y,z), rotation=Quaternion(self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3]))
                p = MultiDOFJointTrajectoryPoint([transforms], [self.velocities], [self.accelerations], rospy.Time(i))
                self.msg = p
            return self.msg

if __name__ == '__main__':
    rospy.init_node('Trajectory_Test', anonymous=True)
    r = rospy.Rate(10)  #hz
    rospy.loginfo('ProgramSTarted')
    traj_pub = rospy.Publisher('red/position_hold/trajectory', MultiDOFJointTrajectoryPoint,queue_size=1)
    
    do_it = Trajectory()

    while not rospy.is_shutdown():
        traj_pub.publish(do_it.update())
        r.sleep()