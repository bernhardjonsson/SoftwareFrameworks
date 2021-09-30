#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates as  msg

global gazebo_obj

rospy.init_node('CubeFinder')

def find_substring(lis):
    pass

def sub_cal(msg):
    global gazebo_obj
    gazebo_obj = msg.name

rospy.Subscriber('/gazebo/model_states', msg, sub_cal, queue_size = 1000)

while not rospy.is_shutdown():
    print(gazebo_obj)
    #print(gazebo_obj.pose[2])
    rospy.sleep(10000)
