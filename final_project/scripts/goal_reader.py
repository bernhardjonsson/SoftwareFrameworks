#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates as  msg
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import String

global pose_list
pose_list = []
for i in range(5):
    stamped_pose = PoseStamped()
    pose_list.append(stamped_pose)

def update_Marker_Position(PoseStamped):
    global marker_Pose
    marker_Pose = PoseStamped.pose

def transformation_Matrix(localPose, globalPose):
    print("should transform here")
    


def sub_cal(String):
    global message
    message = String
    try:
        #Return if empty data message
        if(len(message.data) < 1):
            return
        #Parse the string into coordinates and index
        coords = message.data.split('\r\n')
        x = float(coords[0][2:])
        y = float(coords[1][2:])
        X_next = float(coords[2][7:])
        Y_next = float(coords[3][7:])
        index = int(coords[4][2])
        #We consider the 5th point to be the 0th for convencience sake.
        if (index == 5):
            index = 0 
        #Might use this for coordinate transformation but idk. Just store it for now.
        global currentPose
        currentPose = PoseStamped()
        currentPose.header.frame_id = 'map'
        currentPose.pose.position.x = x
        currentPose.pose.position.y = y
        #Store the next goal.
        global goal_pose
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.pose.position.x = X_next
        goal_pose.pose.position.y = Y_next
        if not goal_pose in pose_list:
            pose_list[index] = goal_pose 
    except ValueError:
        print('no good value')
        #continue
    



#initialize subscriber
rospy.Subscriber('/visp_auto_tracker/code_message', String, sub_cal, queue_size = 1000)
rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, update_Marker_Position, queue_size=1000)

if __name__ == '__main__':
    print("==========Initializing==========")
    rospy.init_node('goal_reader')
    global GoalPos_publisher
    GoalPos_publisher = rospy.Publisher('/Goal_pos',Pose, queue_size = 1000) #<---
    r = rospy.Rate(1)
    found_points = 0
    while not rospy.is_shutdown():
        prev_points = found_points
        found_points = 0
        for pose in pose_list:
            if(len(pose.header.frame_id) > 0):
                found_points += 1
                GoalPos_publisher.publish(pose.pose)   
        if found_points != prev_points:
            print("found ", len(pose_list), " poses")
            for pose in pose_list:
                print(pose.pose.position)

            if (found_points == 2):
                transformation_Matrix
            if (found_points >= 2):
                for i in range(5):
                    if (i == 4):
                        if(len(pose_list[i].header.frame_id) > 0 and len(pose_list[0].header.frame_id) == 0 ):
                            GoalPos_publisher.publish(pose_list[i].pose)
                            print("\n publishing goal: ", pose_list[i].pose)
                            break
                    else:
                        if(len(pose_list[i].header.frame_id) > 0 and len(pose_list[i+1].header.frame_id) == 0 ):
                            GoalPos_publisher.publish(pose_list[i].pose)
                            print("\n publishing goal: ", pose_list[i].pose)
                            break
                print("Should have found all points by now!")


            
        r.sleep()