#!/usr/bin/env python
 
import rospy
import actionlib
 
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from gazebo_msgs.msg import ModelStates as  msg #<---
 
#waypoints = [  
#    [(1.13, -1.6, 0.0), (0.0, 0.0, -0.16547, -0.986213798314)],
#    [(0.13, 1.93, 0.0), (0.0, 0.0, -0.64003024, -0.76812292098)]
#]
 
 
#def goal_pose(pose):  
#    goal_pose = MoveBaseGoal()
#    goal_pose.target_pose.header.frame_id = 'map'
#    goal_pose.target_pose.pose.position.x = pose[0][0]
#    goal_pose.target_pose.pose.position.y = pose[0][1]
#    goal_pose.target_pose.pose.position.z = pose[0][2]
#    goal_pose.target_pose.pose.orientation.x = pose[1][0]
#    goal_pose.target_pose.pose.orientation.y = pose[1][1]
#    goal_pose.target_pose.pose.orientation.z = pose[1][2]
#    goal_pose.target_pose.pose.orientation.w = pose[1][3]
# 
#    return goal_pose


#<----
global GoalStatus
GoalStatus = -1

global gazebo_obj
gazebo_obj = msg()

global burger_robot_pose
burger_robot_pose = Pose()

def RandomPos(GoalPos_pub):
  global GoalStatus
  global burger_robot_pose
  if (GoalStatus == 1) or (GoalStatus == -1):
    #Succesful or Failed -> Go to new random pose:
    RndAng = random.uniform(-3.14,3.14)
    RndDist = random.uniform(0.3,0.5)
    newPos = burger_robot_pose
    newPos.position.x = burger_robot_pose.x + cos(RndAng)*RndDist
    newPos.position.y = burger_robot_pose.y + sin(RndAng)*RndDist
    GoalPos_pub.publish(newPos)
  elif GoalStatus == 0:
    #Still Going.. Skip

def sub_GoalPosStatus(GoalPoseStatus):
    global GoalStatus
    GoalStatus = GoalPoseStatus

def sub_cal(msg):
    global gazebo_obj
    gazebo_obj = msg
    indx = find_substring(gazebo_obj.name,'turtlebot3_burger')
    if len(indx) == 0: #Not found
      print("Turtlebot pose not found in gazebo !!!")
    elif gazebo_obj is not 0: #Check if variable is empty:
        burger_robot_pose = gazebo_obj.pose[indx[0]:indx[-1]+1]

#<----



 
 
if __name__ == '__main__':
    #rospy.init_node('patrol')
 
    GoalPos_pub = rospy.Publisher('/Goal_pos',Pose, queue_size = 1000) #<---
    rospy.Subscriber('/Goal_pos_status', int, sub_cal, queue_size=1000) #<---
    rospy.Subscriber('/gazebo/model_states', msg, sub_cal, queue_size = 1000) #<---
    rospy.sleep(3) #<--- 



    #client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    #client.wait_for_server()
   
    while True:
      RandomPos(GoalPos_pub)
      rospy.sleep(3)


      
      #  for pose in waypoints:   
      #      goal = goal_pose(pose)
      #      client.send_goal(goal)
      #      client.wait_for_result()
      #      rospy.sleep(3)
      #      ...
