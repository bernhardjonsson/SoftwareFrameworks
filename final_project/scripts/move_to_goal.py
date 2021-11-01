#!/usr/bin/env python
 
import rospy
import actionlib
import copy
from geometry_msgs.msg import Pose
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
 

def Get_goal_pos(msg):
    global goal_pose
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = msg.position.x
    goal_pose.target_pose.pose.position.y = msg.position.y
    goal_pose.target_pose.pose.position.z = msg.position.z
    goal_pose.target_pose.pose.orientation.x = msg.orientation.x
    goal_pose.target_pose.pose.orientation.y = msg.orientation.y
    goal_pose.target_pose.pose.orientation.z = msg.orientation.z
    goal_pose.target_pose.pose.orientation.w = msg.orientation.w

def print_goal(target_pose):
    print("Goal position:\n  x: " + str(target_pose.pose.position.x) +
            "\n  y: " + str(target_pose.pose.position.y) +
            "\n  z: " + str(target_pose.pose.position.z) +
            "\nGoal orientation:\n  x: " + str(target_pose.pose.orientation.x) +
            "\n  y: " + str(target_pose.pose.orientation.y) +
            "\n  z: " + str(target_pose.pose.orientation.z) +
            "\n  w: " + str(target_pose.pose.orientation.w))


#subcribe to topics
rospy.Subscriber("Goal_pos", Pose, Get_goal_pos, queue_size = 1000)


if __name__ == '__main__':
    print("==========Initializing==========")
    rospy.init_node('Move_to_goal')
    
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    client.wait_for_server()
    goal_pose = MoveBaseGoal()
    last_goal = MoveBaseGoal()
    listening = False

    print("==========Runnning==========")
    while True:
        # if not empty and at goal :
        if goal_pose != last_goal:
            print("==========Going to goal==========")
            print_goal(goal_pose.target_pose)
            client.send_goal(goal_pose)
            print(client.get_result())
            client.wait_for_result()
            last_goal = copy.deepcopy(goal_pose)
            listening = False

        elif goal_pose == last_goal and not listening:
            print("==========Listening !==========")
            listening = True

        rospy.sleep(3)
