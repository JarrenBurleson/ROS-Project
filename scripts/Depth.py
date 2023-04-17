#!/usr/bin/env python

import rospy 
import tf 
import math
import numpy as np
import std_msgs.msg
from geometry_msgs.msg import Transform, Quaternion, Point, Twist

from trajectory_msgs.msg import MultiDOFJointTrajectory, MultiDOFJointTrajectoryPoint

from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2

import matplotlib.pylab as plt

class DepthReader():
  
  def __init__(self):
    rospy.init_node ('depth_reader', anonymous=True)

  def useDepthData(self, data):
    #rospy. loginfo(rospy.get_caller_id() + "I heard %s", data.fields)
    #self.points = PointCloud2.read_points(data)
    #rospy. loginfo(rospy.get_caller_id() + "I heard %s", data.fields)
    #self.points = PointCloud2.read_points(data)
    self.points = sensor_msgs.point_cloud2.read_points(data, skip_nans=True, field_names = ['x','y','z'])
    self.fields = data.fields
    self.height = data.height
    self.width = data.width

    self.is_bigendian = data.is_bigendian # Is this data bigendian?
    self.point_step = data.point_step  # Length of a point in bytes
    self.row_step = data.row_step     # Length of a row in bytes
    self.data = data.data        # Actual point data, size is (row_step*height)
    self.is_dense = data.is_dense
    
  
  def getDepthData(self):
    rospy.Subscriber("/red/camera/depth_registered/points", PointCloud2, self.useDepthData)
    #rospy.Subscriber ("mavros/global _position/local", PointCloud2, self.callback) 
    rospy.spin()

dr = DepthReader()

if __name__ == "__main__":
  dr.getDepthData()
  #print("Points: ")
  #for point in dr.points:
  #  print(point)
  print("Fields: ")
  print(dr.fields)
  x = np.array([])
  y = np.array([])
  z = np.array([])

  uniform_data = list(dr.points)
  for point in uniform_data:
    #print(point[0])
    x=np.append(x,point[0])
    y=np.append(y,point[1])
    z=np.append(z,point[2])
  #ax = sns.heatmap(uniform_data, linewidth=0.5)
#print(x)
plt.scatter(x,y)

plt.show()
  #print("moving")
  #if dr.points: print(dr.points)
