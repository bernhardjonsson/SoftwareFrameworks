#!/usr/bin/env python

import rospy
from gazebo_msgs.msg import ModelStates as  msg
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import String, Int16
import tf
import copy
import random


global goalStatus
goalStatus = 0

class markerData:
    def __init__(self):
        self.codes = {}
        self.object_position = PoseStamped()
        self.current_goal = PoseStamped()
        self.current_target = 0

    def add_code(self,code):
        index = int(code.data['N'])
        if self.new_QR_Code(code):
            self.codes[index] = code
            return True
        else:
            return False

    def new_QR_Code(self,code):
        if int(code.data['N']) in self.codes:
            return False
        else:
            return True

class VispData:
    def __init__(self,pose,msg):
        self.pose = pose #object position from /visp_auto_tracker/object_position
        self.pose.header.frame_id = '/camera_optical_link'
        self.data = {}
        for keypair in msg.data.split("\r\n"):
            pair = keypair.split("=")
            key = pair[0]
            value = pair[1]
            self.data[key] = value

def goal_pose(x_dest, y_dest):  
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.pose.position.x = x_dest
    goal_pose.pose.position.y = y_dest
    goal_pose.pose.position.z = 0.0
    #q = quaternion_from_euler(0.000154, 0.007177, -0.1000)
    q = [0,0,0,1]
    goal_pose.pose.orientation.x = q[0]
    goal_pose.pose.orientation.y = q[1]
    goal_pose.pose.orientation.z = q[2]
    goal_pose.pose.orientation.w = q[3]
    return goal_pose.pose


def update_Marker_Position(PoseStamped):
    global marker_Pose
    marker_Pose = PoseStamped

def get_tf(markerData):
    x = []
    y = []
    x_n = []
    y_n = []
    for value in list(markerData.codes.values())[:2]:
        x_n.append(value.pose.pose.position.x)
        y_n.append(value.pose.pose.position.y)
        x.append(value.data['X'])
        y.append(value.data['Y'])
    
    print('x_n is:' , x_n)
    print('y_n is:' ,y_n)
    print('x is:' , x)
    print('y is:' , y)
    x = map(float,x)
    y = map(float,y)
    x_n = map(float,x_n)
    y_n = map(float,y_n)
    c = (x[0] * x_n[0] - x[0] * x_n[1] - x_n[0] * x[1] + x[1] * x_n[1] + y[0] * y_n[0] - y[0] * y_n[1] - y_n[0] * y[1] + y[1] * y_n[1]) / (x[0] ** 2 - 2 * x[0] * x[1] + x[1] ** 2 + y[0] ** 2 - 2 * y[0] * y[1] + y[1] ** 2)
    s = (x[0] * y_n[0] - x[0] * y_n[1] - x_n[0] * y[0] + x_n[0] * y[1] - x[1] * y_n[0] + x[1] * y_n[1] + x_n[1] * y[0] - x_n[1] * y[1]) / (x[0] ** 2 - 2 * x[0] * x[1] + x[1] ** 2 + y[0] ** 2 - 2 * y[0] * y[1] + y[1] ** 2)
    a1 = -(c ** 2 * x[0] + s ** 2 * x[0] - c * x_n[0] - s * y_n[0]) / (c ** 2 + s ** 2)
    a2 = -(c ** 2 * y[0] + s ** 2 * y[0] - c * y_n[0] + s * x_n[0]) / (c ** 2 + s ** 2)
    print('got values c,s,a1,a2: ',c,s,a1,a2)    
    
    print('calculated constants to be: c=',c,', s=',s,', a1= ',a1,', a2=',a2)
    return c, s, a1, a2

def transformToMap(x_next, y_next, c,s,a1,a2):
    x = float(x_next)
    y = float(y_next)
    x1n = c*x - s*y + a1
    y1n = s * x + c * x + a2
    return x1n, y1n

def find_Next_Goal(markerData, goalPublisher, thisNode, c, s, a1, a2):  
    global goalStatus
    if len(markerData.codes) < 5:
        if (markerData.current_target  % 5) + 1 not in markerData.codes.keys() and markerData.current_target > 0:
            if abs(goalStatus) == 1:
                #We apparently reached the goal without managing to scan it. oops. Let's move around randomly for a bit to see if we can spot it.
                value = markerData.codes[str(markerData.current_target)]
                x_dest, y_dest = transformToMap(value.data['X_next'],value.data['Y_next'],c,s,a1,a2)
                x = float(x_dest)
                y = float(y_dest)
                x = x - (random.uniform(-0.3,0.3)*(x/abs(x)))
                y = y - (random.uniform(-0.3,0.3)*(y/abs(y)))
                offset_goal = goal_pose(x,y)
                goalPublisher.publish(offset_goal)
                markerData.current_goal = offset_goal
                print("republished goal:", offset_goal)
                #We should add some rotation data to the pose so it will still look at the marker position.

            goalPublisher.publish(markerData.current_goal)
            rospy.sleep(2)
            return
        else:
            for key, value in markerData.codes.items():
                if (key % 5) + 1 in markerData.codes.keys():
                    print((key % 5) + 1, ' already exists in dictionary')
                    continue
                else:
                    #Navigate to this key since we don't have it yet
                    x_dest, y_dest = transformToMap(value.data['X_next'],value.data['Y_next'],c,s,a1,a2)
                    goal = goal_pose(x_dest,y_dest)
                    goalPublisher.publish(goal)
                    print('next goal is: ',(key % 5) + 1)
                    print('converting goal', value.data['X_next'], value.data['Y_next'],' to map coordinates')
                    print("Published goal:" , goal.position.x, " , ", goal.position.y )
                    markerData.current_goal = goal
                    markerData.current_target = (key % 5) + 1
                    rospy.sleep(2)
                    return
    else:
        print("complete!")
        thisNode.kill()
        rospy.sleep(20)

def qr_code_msg_cb(message, args):
    tf_util = args[0]
    markers = args[1]
    try:
        #Return if empty data message
        if(len(message.data) < 1):
            return
        #Parse the string into coordinates and index
        code = VispData(marker_Pose,message)
        if markers.new_QR_Code(code): #It's a new QR code 
            print("spotted new QR code")
        else:
            return #We already saw this code.
        if len(markers.codes) >= 2:
            pose = copy.deepcopy(code.pose)
            pose.header.frame_id - '/camera_optical_link'
            code = VispData(pose,message)
            markers.add_code(code)
            
            print("Added QR code {}. We now have the following QR Codes {}").format(code.data['N'], markers.codes.keys())
        else:
            cam_pose = copy.deepcopy(code.pose)
            pose = tf_util.transform_pose(cam_pose, 'camera_optical_link', 'map')
            code = VispData(pose,message)
            markers.add_code(code)
            print("Added QR code {}. We now have a total of {} QR Codes").format(code.data['N'], len(markers.codes))
    except ValueError:
        print('no good value')
        return
def GoalPosStatus_cb(msg): #Use this to check if we failed to read QR code.
    global goalStatus
    goalStatus = msg.data
    if goalStatus == 1:
        print("reached goal ")
    

#Borrowed form https://gist.github.com/awesomebytes/960456009d81365b6c635e0c6c1cf5ca#file-tf_for_me-py
class TF_stuff(object):
    def __init__(self):
        self.tf_l = tf.TransformListener()
        rospy.sleep(0.5)

    def transform_pose(self, pose_stamped, from_frame, to_frame):
        ps = PoseStamped()
        # ps.header.stamp = #self.tf_l.getLatestCommonTime(from_frame,
        # to_frame)
        ps.header.frame_id = from_frame
        ps.pose = pose_stamped.pose
        transform_ok = False
        while not transform_ok and not rospy.is_shutdown():
            try:
                target_ps = self.tf_l.transformPose(to_frame, ps)
                transform_ok = True
            except tf.ExtrapolationException as e:
                rospy.logwarn(
                    "Exception on transforming pose... trying again \n(" + str(e) + ")")
                rospy.sleep(0.2)
                ps.header.stamp = self.tf_l.getLatestCommonTime(
                    from_frame, to_frame)
            except tf.LookupException as e:
                rospy.logwarn(
                    "Exception on transforming pose... trying again \n(" + str(e) + ")")
                rospy.sleep(0.2)

        return target_ps
    






if __name__ == '__main__':
    print("==========Initializing==========")
    reader_node = rospy.init_node('goal_reader')
    #Listen for transform to camera and wait for it to init. Then set up usable class from it.
    listener = tf.TransformListener()
    rospy.sleep(2.0)
    try:
        now = rospy.Time.now()
        listener.waitForTransform('/map', '/camera_optical_link', now, rospy.Duration(4.0))
        listener = TF_stuff() 
    except:
         (tf.LookupException, tf.ConnectivityException)

    markers = markerData()
    #initialize subscribers
    rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, update_Marker_Position, queue_size=1000)
    rospy.Subscriber('/visp_auto_tracker/code_message', String, qr_code_msg_cb, [listener,markers], queue_size = 1000)
    rospy.Subscriber('/Goal_pos_status', Int16, GoalPosStatus_cb, queue_size=1000) #<---
    #Intialize publishers
    GoalPos_publisher = rospy.Publisher('/Marker_pos',Pose, queue_size = 1000) #<---
    goal_status_pub = rospy.Publisher('/Goal_pos_status', Int16, queue_size=1000)
    r = rospy.Rate(1)
    found_points = 0
    goal_status_pub.publish(-1) #Initialize patrol script
    #Define transform parameters so we can check if they were set
    c = s = a1 = a2 = None
    while not rospy.is_shutdown():
        if len(markers.codes) == 5:
            print("complete!")
            reader_node.kill()
        if len(markers.codes) >= 2 and c == None:
            #We need to define our transformation matrix first
            c,s,a1,a2 = get_tf(markers)
        if len(markers.codes) >= 2 and c is not None:
            #We have a fully defined rotation matrix and can publish goals
            find_Next_Goal(markers, GoalPos_publisher, reader_node, c, s, a1, a2)
            continue
        r.sleep()