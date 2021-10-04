#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates as  msg
from geometry_msgs.msg import PoseArray, Pose

global gazebo_obj
gazebo_obj = msg()

rospy.init_node('CubeFinder')
r = rospy.Rate(2) # Set Frequency

def find_substring(lis,sub_str):
    res = []
    for i in range(len(lis)):
        if sub_str in lis[i]:
            res.append(i)
    return res

def sub_cal(msg):
    global gazebo_obj
    gazebo_obj = msg

#initialize subscriber
rospy.Subscriber('/gazebo/model_states', msg, sub_cal, queue_size = 1000)

#initialize publisher
Cube_pub = rospy.Publisher('/CubePos',PoseArray, queue_size = 1000)
Bucket_pub = rospy.Publisher('/BucketPos',Pose, queue_size = 1000)

cube = PoseArray()
Bucket = Pose()

r.sleep() #wait for msg from gazebo
indx = find_substring(gazebo_obj.name,'cube')

while not rospy.is_shutdown():
    if gazebo_obj is not 0: #check if variable is empty:
        cube.poses = gazebo_obj.pose[indx[0]:indx[-1]+1]
        Bucket = gazebo_obj.pose[-1]
    
    Bucket_pub.publish(Bucket)
    Cube_pub.publish(cube)
    r.sleep()
