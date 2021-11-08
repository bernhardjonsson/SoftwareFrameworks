#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates as  msg
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String

#global gazebo_obj
#gazebo_obj = msg()
global message
message = ""

rospy.init_node('letters')
r = rospy.Rate(2) # Set Frequency

def sub_cal(String):
    global message
    message = String

#initialize subscriber
rospy.Subscriber('/visp_auto_tracker/code_message', String, sub_cal, queue_size = 1000)

#List of letters
Lett = ['']*5

r.sleep() #wait for msg from gazebo

print("==========Running==========")

while not rospy.is_shutdown():

    try:
    	indx = message.data.index('N=')
    	indx2 = message.data.index('L=')
    except ValueError:
    	continue

    try:
        Number = message.data[indx+2]
        Letterr = message.data[indx2+2]
    except IndexError:
        continue

    if Letterr in Lett:
        continue
    else:
        Lett[int(Number)-1] = Letterr
        print(Lett)
    
    r.sleep()
