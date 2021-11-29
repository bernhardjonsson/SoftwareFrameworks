#!/usr/bin/env python
import rospy
import actionlib
import random
import math
import tf

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates as  msg #<---
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import Int16
 
global GoalStatus
GoalStatus = -1

global gazebo_obj
gazebo_obj = msg()

global burger_robot_pose
burger_robot_pose = Pose()

def find_substring(lis,sub_str):
    res = []
    for i in range(len(lis)):
        if sub_str in lis[i]:
            res.append(i)
    return res

def RandomPos(GoalPos_pub):
  global GoalStatus
  global burger_robot_pose
  if (GoalStatus == 1) or (GoalStatus == -1):
    #Succesful or Failed -> Go to new random pose: if 0  Still Going.. Skip
    RndAng = random.uniform(-3.14,3.14)
    RndDist = random.uniform(0.6,1.5)
    newPos = burger_robot_pose
    newPos.position.x = burger_robot_pose.position.x + math.cos(RndAng)*RndDist
    newPos.position.y = burger_robot_pose.position.y + math.sin(RndAng)*RndDist
    quaternionOrientation = tf.transformations.quaternion_from_euler(RndAng,0,0)
    newPos.orientation.x = quaternionOrientation[0]
    newPos.orientation.y = quaternionOrientation[1]
    newPos.orientation.z = quaternionOrientation[2]
    newPos.orientation.w = quaternionOrientation[3]
    newPos.orientation.x = 0
    newPos.orientation.y = 0
    newPos.orientation.z = 0
    newPos.orientation.w = 1
    GoalPos_pub.publish(newPos)
    print("publishing new goal: " , newPos.position, 'which is ', RndDist, ' units away.')
 
def sub_GoalPosStatus(GoalPoseStatus):
  global GoalStatus
  GoalStatus = GoalPoseStatus.data
  print('updated goal status to be:', GoalPoseStatus.data )
 
def sub_cal(msg):
  global gazebo_obj
  gazebo_obj = msg
  indx = find_substring(gazebo_obj.name,'turtlebot3_burger')
  if len(indx) == 0: #Not found
    print("Turtlebot pose not found in gazebo !!!")
  elif gazebo_obj is not 0: #Check if variable is empty:
    burger_robot_pose = gazebo_obj.pose[indx[0]:indx[-1]+1]
 

 
if __name__ == '__main__':
  print("==========Initializing==========")
  GoalPos_publisher = rospy.Publisher('/Goal_pos',Pose, queue_size = 1000)
  rospy.Subscriber('/Goal_pos_status', Int16, sub_GoalPosStatus, queue_size=1000)
  rospy.Subscriber('/gazebo/model_states', msg, sub_cal, queue_size = 1000)
  rospy.sleep(3) 
  rospy.init_node('PatrolJAP', anonymous=True) 

  while True:
    print("==========Running==========")
    RandomPos(GoalPos_publisher)
    rospy.sleep(5)
# END ALL