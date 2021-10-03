#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates as  msg
from ex_mini.msg import CubeFinder

global gazebo_obj
gazebo_obj = 0

rospy.init_node('CubeFinder')

def find_substring(lis,sub_str):
    res = []
    for i in range(len(lis)):
        if sub_str in lis[i]:
            res.append(i)
    return res

def sub_cal(msg):
    global gazebo_obj
    gazebo_obj = msg.name

#initialize subscriber
rospy.Subscriber('/gazebo/model_states', msg, sub_cal, queue_size = 1000)

#initialize publisher
#rospy.Publisher('

#rospy.sleep(2)
while not rospy.is_shutdown():
    if gazebo_obj is not 0: #check if variable is empty:
        print(gazebo_obj)
        print(find_substring(gazebo_obj,'cube'))
    #print(gazebo_obj.pose[2])
    #rospy.sleep(100)
