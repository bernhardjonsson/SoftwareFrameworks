#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates as  msg
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import String
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf.transformations import quaternion_from_euler
#import tf2_ros
import numpy as np
import sys

def transformFromQR(x, y, z):
    R = [[0.0173, -0.7574, -3.8170], [0.7574, 0.0173, 0.7918], [0, 0, 1.0000]] #Rotation matrix, replace this with whatever you got.
    local = np.matmul(R,np.array([[x],[y],[1]]))
    local[2] = z
    return local[0],local[1], local[2]
    

if __name__ == '__main__':
    print("==========Initializing==========")
    rospy.init_node('TestGoal')
    args = rospy.myargv(argv=sys.argv)
    if len(args) != 3:
        print "Please specify desired x and y!"
        sys.exit(1)
    x_in = float(args[1])
    y_in = float(args[2])
    z_const = -0.000965
    x_dest, y_dest, z_dest = transformFromQR(x_in,y_in, z_const)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = x_dest
    goal_pose.pose.position.y = y_dest
    goal_pose.pose.position.z = z_dest
    q = quaternion_from_euler(0.000154, 0.007177, -0.1000)
    goal_pose.pose.orientation.x = q[0]
    goal_pose.pose.orientation.y = q[1]
    goal_pose.pose.orientation.z = q[2]
    goal_pose.pose.orientation.w = q[3]

    global GoalPos_publisher
    GoalPos_publisher = rospy.Publisher('/Goal_pos',Pose, queue_size = 1000) 
    while not rospy.is_shutdown():
        GoalPos_publisher.publish(goal_pose.pose)
        rospy.sleep(10)

