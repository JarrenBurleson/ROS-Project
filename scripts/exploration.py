#!/usr/bin/env python


#takes goal_out data and publishes to 'red/tracker/input_pose' to move the drone
import rospy
import tf
import math
import numpy as np
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Pose, PoseStamped, PoseWithCovariance



class Trajectory():
    def __init__(self):
        self.traj_pub = rospy.Publisher('red/tracker/input_pose', PoseStamped,queue_size=1)
        self.msg = PoseStamped()
        self.header = std_msgs.msg.Header()
        self.header.frame_id = 'frame'
        self.quaternion = tf.transformations.quaternion_from_euler(0,0,0)
        self.euler = (0,0,0)
        self.velocities = Twist()
        self.accelerations = Twist()


    def update(self,x,y,z,pitch = None,yaw = None,roll = None):

            self.header.stamp = rospy.Time()
            #self.msg.header = self.header
            position=Point(x,y,z)
            if pitch == None:
                orientation=Quaternion(self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3])
            else:
                self.euler = [pitch,yaw,roll]
                self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3] = tf.transformations.quaternion_from_euler(pitch,yaw,roll)
                orientation=Quaternion(self.quaternion[0],self.quaternion[1],self.quaternion[2],self.quaternion[3])
            
            pose = Pose(position, orientation)
            p = PoseStamped(self.header, pose)
            self.msg = p
            return self.msg
    
    def update(self,data):
        self.traj_pub.publish(data)
        



#def callback(data):



     

if __name__ == '__main__':
    rospy.init_node('Trajectory', anonymous=True)
    r = rospy.Rate(1)  #hz
    
    rospy.loginfo('Program Started')
    
    do_it = Trajectory()



    while not rospy.is_shutdown():
        #pitch,yaw,roll = 0,0,0
        #traj_pub.publish(do_it.update(x,y,z))
        #traj_pub.publish(do_it.update(x,y,z,pitch,yaw,roll))
        traj_sub = rospy.Subscriber("/rtabmap/goal_out", PoseStamped, do_it.update)
        r.sleep()