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
import tf

global spottedQR
spottedQR = False
def transformFromQR(x, y, z):
   # R = [[0.0173, -0.7574, -3.8170], [0.7574, 0.0173, 0.7918], [0, 0, 1.0000]] #Rotation matrix, replace this with whatever you got.
    H = np.array([[0.017269084380610382404434985834002, -0.75743804308797116496985424943252, 0, -3.8169985761220827720494326123856],
    [0.017269084380610382404434985834002, -0.75743804308797116496985424943252, 0, -3.8169985761220827720494326123856],
    [ 0.75743804308797116496985424943252, 0.017269084380610382404434985834002, 0, 0.79184828240574552018884319058704],
    [ 0.75743804308797116496985424943252, 0.017269084380610382404434985834002, 0, 0.79184828240574552018884319058704],
    [0, 0, 1,0],
    [0, 0, 1,0],
    [0, 0, 0,1],
    [0, 0, 0,1]])
    #local = np.matmul(R,np.array([[x],[y],[0]]))
    local = np.matmul(H,np.array([[x],[y],[0.0], [1]]))
    #local[2] = z
    #return local[0],local[1], local[2]
    #return local[0],local[2], z
    return x, y, z

def handle_QR_pose(msg, poseName='qrCode'):
    if spottedQR == True:
        print('broadcasting transform')
        br = tf.TransformBroadcaster()
        br.sendTransform((msg.pose.position.x, msg.pose.position.y, 0), 
                        tf.transformations.quaternion_from_euler(0,0,0),
                        rospy.Time.now(),
                        poseName,
                        "map")

def sub_cal(message):
    global spottedQR
    if(len(message.data) < 1):
        spottedQR = False
    else:
        spottedQR = True
        
        
    

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
    print("Going to coordinates: x=", x_dest, ", y=", y_dest)
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

    rospy.Subscriber('/visp_auto_tracker/code_message', String, sub_cal, queue_size = 1000)
    rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, handle_QR_pose, queue_size=1000)


    global GoalPos_publisher
    GoalPos_publisher = rospy.Publisher('/Goal_pos',Pose, queue_size = 1000) 
    while not rospy.is_shutdown():
        GoalPos_publisher.publish(goal_pose.pose)
        rospy.sleep(10)

