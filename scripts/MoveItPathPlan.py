#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from moveit_commander import MoveGroupCommander, RobotCommander
import moveit_commander
from tf.transformations import quaternion_from_euler, euler_from_quaternion
import math

# Set up MoveIt commander
rospy.init_node('moveit_path_planning')
robot = RobotCommander()
scene = moveit_commander.PlanningSceneInterface()
group = MoveGroupCommander('Quad_base')

#create a DisplayTrajectory ROS publisher used to display trajectories in Rviz
# display_trajectory_publisher = rospy.Publisher(
#     "/move_group/display_planned_path",
#     moveit_msgs.msg.DisplayTrajectory,
#     queue_size=20,
# )

# Define callback function to plan and execute trajectory to new goal
def goal_callback(data):
    # Set goal pose
    # goal = PoseStamped()
    # goal.header.stamp = rospy.Time.now()
    # goal.header.frame_id = 'map'
    # goal.pose = data.pose

    # # Get current robot pose and calculate desired orientation
    # current_pose = group.get_current_pose().pose
    # dx = goal.pose.position.x - current_pose.position.x
    # dy = goal.pose.position.y - current_pose.position.y
    # yaw = euler_from_quaternion((current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w))[2]
    # q = quaternion_from_euler(0, 0, yaw + math.atan2(dy, dx))

    # # Set desired orientation and plan trajectory to goal
    # goal.pose.orientation.x = q[0]
    # goal.pose.orientation.y = q[1]
    # goal.pose.orientation.z = q[2]
    # goal.pose.orientation.w = q[3]
    #group.setMaxAccelerationScalingFactor(0.5) ?????????
    group.set_pose_target(data)
    trajectory = group.plan()

    # Execute trajectory
    group.execute(trajectory)

# Subscribe to input_goal topic and call goal_callback function on new messages
rospy.Subscriber('input_goal', PoseStamped, goal_callback)

# Spin until node is stopped
rospy.spin()
