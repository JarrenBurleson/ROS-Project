#!/usr/bin/env python

#Sends goal points to '/rtabmap/goal'
#Will eventually read points from poi into '/rtabmap/goal'
#exploration.py publishes goal_out points to red/tracker/input_pose

import rospy
import tf
import math
import numpy as np
import std_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Transform, Quaternion, Point, Twist, Pose, PoseStamped, PoseWithCovariance


iter = 0
#points = [[5,12,4],[10,10,5],[25,10,20],[25,20,5],[10,30,5]]
#points = [[7.56, 8.32, 9.1], [7.57, 14.60, 5.5], [3.00, 43.36, 11.1], [9.3, 43.36, 4.8], [10.36, 45.3, 3.8], [10.36, 46.2, 12.1], [10.36, 44.5, 8.35], [16.2, 44.275, 7.3], [5.92, 12, 4.35], [6.34, 11, 4.35]]
points = [[7.5,7,3.15],[5, 12, 4], [6.34, 11, 4.35],[7.57, 14.60, 5.5],[3.00, 43.36, 11.1]]
#points = [[5,10,5],[5,2,0],[6.34, 11, 4.35]]

class Trajectory():
    def __init__(self):

        self.msg = PoseStamped()
        self.header = std_msgs.msg.Header()
        self.header.frame_id = 'map'
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


def callback(data):
    global iter
    #distance = data.pose.position.x - points[iter]
    distance = math.sqrt(((data.pose.position.x-points[iter][0])**2) + (data.pose.position.y-points[iter][1])**2 + (data.pose.position.z-points[iter][2])**2)
    if distance < 2:
        if iter < 2: iter +=1


     

if __name__ == '__main__':
    rospy.init_node('Trajectory', anonymous=True)
    r = rospy.Rate(1)  #hz
    rospy.loginfo('Program Started')
    traj_pub = rospy.Publisher('input_goal', PoseStamped,queue_size=1)
    
    do_it = Trajectory()



    while not rospy.is_shutdown():
        x,y,z = points[iter][0],points[iter][1],points[iter][2]
        pitch,yaw,roll = 0,0,15
        #traj_pub.publish(do_it.update(x,y,z))
        traj_pub.publish(do_it.update(x,y,z,pitch,yaw,roll))
        traj_sub = rospy.Subscriber("/red/pose", PoseStamped, callback)
        r.sleep()