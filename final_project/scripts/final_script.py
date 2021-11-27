#!/usr/bin/env python
# BEGIN ALL
# based on wander.py script of 31393 sffas course

import rospy
from geometry_msgs.msg import Twist, PoseStamped, PoseArray, PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String, Int8
import actionlib
import tf
import copy
import time 
from random import random
import datetime

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

## Objective of this class: establish the coordinate frame of the QRCodes 
class QRTransformEstablisher:
  def __init__(self):
    self.codes = []
    self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction) 
    self.client.wait_for_server()
    self.robotpose = PoseStamped()
    self.pose = PoseStamped() ##contains the current message of /visp_auto_tracker/object_position
    self.data = String()
    self.stop = False # Tells the robot whether it has the approval to move around
    self.pub1 = rospy.Publisher('/test_pose', PoseStamped, queue_size=100)
    self.pub2 = rospy.Publisher('/test_pose2', PoseStamped, queue_size=100)
    self.tracker_state = 0
    self.stop_time = rospy.Time(0)  # Point in time where we have stopped the movement to wait 
    self.time_since_qr = rospy.Time.now()
    self.live_qr_data = None
    self.init_qr_time = None
    self.live_qr_time = None
    self.in_transit = False

  def append_code(self, code):
    # Appends a QR code to the self.codes array if it's a new one we haven't encountered yet
    # returns True if it's a new one and we have appended it, returns false if already known 
    if not self.isCodeLogged(code):
      self.codes.append(code)
      self.time_since_qr = rospy.Time.now()
      return True
    else:
      return False
  def _get_current_codes(self):
    # Private function, returns the QR codes we already know (indexed by N)
    # Returns array of QR code indices that are already known
    codes = []
    for code in self.codes:
      codes.append(code.data['N'])
    return codes
  def isCodeLogged(self, code):
    if code.data['N'] not in self._get_current_codes():
      return False 
    else:
      return True


class QRData:
  # Class to summarize QR data
  def __init__(self, pose, data):
    self.pose = pose
    self.data = {}
    for keypair in data.split("\r\n"):
      pair = keypair.split("=")
      key = pair[0]
      value = pair[1]
      self.data[key] = value
  
# from https://gist.github.com/awesomebytes/960456009d81365b6c635e0c6c1cf5ca#file-tf_for_me-py
class TF_stuff(object):
    def __init__(self):
        self.tf_l = tf.TransformListener()
        rospy.sleep(0.5)

    def transform_pose(self, pose_stamped, from_frame, to_frame):
        ps = PoseStamped()
        ps.header.stamp = self.tf_l.getLatestCommonTime(from_frame,
        to_frame)
        ps.header = pose_stamped.header
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


def qr_position_callback(msg, current):
  current.pose = msg

def qr_status_callback(msg, current):
  current.tracker_state = msg.data

def qr_live_data_callback(msg, current):
  if msg.data is not '':
    current.live_qr_data = msg.data
    current.live_qr_time = datetime.datetime.now()
    try:
      if current.in_transit:
        print "qr spotted cancelling goal"
        current.client.cancel_goal()

    except KeyboardInterrupt:
    	 raise
    current.in_transit = False
    current.time_since_qr = rospy.Time.now()
  
def qr_data_callback(msg, args ):
  if msg.data is not '':
    current = args[0]
    cmd_vel_pub = args[1]
    trf = args[2]
    code = QRData(current.pose, msg.data)
    if current.isCodeLogged(code):
      pass
    else:
      if len(current.codes) >= 2:
        pose = copy.deepcopy(current.pose) 
        pose.header.frame_id = '/camera_optical_link'
        current.append_code(QRData(pose, msg.data))
      else:
        current.stop = True
        cmd_vel_pub.publish(Twist()) # stopping
        print "Trying to stop and go to sleep"
        time.sleep(10.0)
        print "Waking up"
        print "Current time: %s" % rospy.get_rostime()
        if current.tracker_state == 3:
          pose = copy.deepcopy(current.pose) 
          pose.header.frame_id = '/camera_optical_link'
          pose_raw = copy.deepcopy(pose)
          pose = trf.transform_pose(pose_raw, 'camera_optical_link', 'map')
          if current.live_qr_data != msg.data:
            current.stop = False
            return
          if current.append_code(QRData(pose, msg.data)):
            print "New QR code scanned"
            print current.init_qr_time, current.live_qr_time
            current.data = msg.data
            print "Giving permission to drive because we have registered QR"
            current.stop = False 
          else:
            print "\rQR code already exists in database",
        else:
          print "Lost track of QR code, continue with movement"
          current.stop = False
      print "\rQR count: %d"%len(current.codes)

def get_tf2d(x1n, y1n, x1, y1, x2n, y2n, x2, y2):
  x1=float(x1)
  y1=float(y1)
  x2=float(x2)
  y2=float(y2)
  c = (x1 * x1n - x1 * x2n - x1n * x2 + x2 * x2n + y1 * y1n - y1 * y2n - y1n * y2 + y2 * y2n) / (x1 ** 2 - 2 * x1 * x2 + x2 ** 2 + y1 ** 2 - 2 * y1 * y2 + y2 ** 2)
  s = (x1 * y1n - x1 * y2n - x1n * y1 + x1n * y2 - x2 * y1n + x2 * y2n + x2n * y1 - x2n * y2) / (x1 ** 2 - 2 * x1 * x2 + x2 ** 2 + y1 ** 2 - 2 * y1 * y2 + y2 ** 2)
  a = -(c ** 2 * x1 + s ** 2 * x1 - c * x1n - s * y1n) / (c ** 2 + s ** 2)
  b = -(c ** 2 * y1 + s ** 2 * y1 - c * y1n + s * x1n) / (c ** 2 + s ** 2)
  return c, s, a, b

def do_tf2d(x1, y1, c, s, a, b):
  x1=float(x1)
  y1=float(y1)
  x1n = c * a - s * b + c * x1 - s * y1
  y1n = a * s + b * c + c * y1 + s * x1
  return x1n, y1n
    

 #one step is one degree, so 1-21 is 
 # if the robot is moving in negative x-direction, the scan should include the area from 200 to 160 degrees
def scan_callback(msg):
  global g_range_ahead
  tmp=[msg.ranges[0]]
  for i in range(1,21):
    tmp.append(msg.ranges[i])
  for i in range(len(msg.ranges)-21,len(msg.ranges)):
    tmp.append(msg.ranges[i])
  g_range_ahead = min(tmp)

def robo_pos_callback(msg):
  pose = PoseStamped()
  pose.header=msg.header
  pose.pose=msg.pose.pose
  current.robotpose = pose
 
# From patrol.py script
def goal_pose(pose):  
    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = 'map'
    goal_pose.target_pose.pose.position.x = pose[0][0]
    goal_pose.target_pose.pose.position.y = pose[0][1]
    goal_pose.target_pose.pose.position.z = pose[0][2]
    goal_pose.target_pose.pose.orientation.x = pose[1][0]
    goal_pose.target_pose.pose.orientation.y = pose[1][1]
    goal_pose.target_pose.pose.orientation.z = pose[1][2]
    goal_pose.target_pose.pose.orientation.w = pose[1][3]
    return goal_pose

def shutdown_fun():
  pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  pub.publish(Twist())

def move_to_next(Q):
  c, s, a, b = get_tf2d(Q.codes[0].pose.pose.position.x, Q.codes[0].pose.pose.position.y, Q.codes[0].data['X'], Q.codes[0].data['Y'], Q.codes[1].pose.pose.position.x, Q.codes[1].pose.pose.position.y, Q.codes[1].data['X'], Q.codes[1].data['Y'])
  queue_head = 0
  queue_size = len(Q.codes)
  while len(Q.codes) < 5:
    is_in_list = False
    for code in Q.codes:
      if int(Q.codes[queue_head].data['N']) == 5:
        if int(code.data['N']) == 1:
          is_in_list = True
          break
      if int(Q.codes[queue_head].data['N'])+1 == int(code.data['N']):
        is_in_list = True
        break
    if is_in_list:
      queue_head += 1
      if queue_head == len(Q.codes):
        queue_head = 0
      continue
    else:
      print "going for", queue_head
      X_map, Y_map = do_tf2d(Q.codes[queue_head].data['X_next'], Q.codes[queue_head].data['Y_next'], c, s, a, b)
      print(' at: ', X_map,Y_map)
      pose3 = [(X_map, Y_map, 0.0), (0,0,0,1)] #Next QR code position 
      goal = goal_pose(pose3)
      Q.client.send_goal(goal)
      wait_status = Q.client.wait_for_result(rospy.Duration(20))
      rospy.sleep(3)
      if wait_status: print "I'm at the center of the QR code"
      if len(Q.codes) == queue_size:
        rand_x = random()*10-5
        rand_y = random()*6-3
        pose3 = [(rand_x, rand_y, 0.0), (0,0,0,1)] #Next QR code position 
        goal = goal_pose(pose3)
        Q.client.send_goal(goal)
        wait_status = Q.client.wait_for_result(rospy.Duration(20))
        rospy.sleep(3)
      queue_head += 1
      if queue_head == len(Q.codes):
        queue_head = 0
      queue_size = len(Q.codes)


if __name__ == "__main__":
  starting_datetime = datetime.datetime.now()
  g_range_ahead = 1 # anything to start
  rospy.init_node('wander')
  listener = tf.TransformListener()
  rospy.sleep(5.0)
  listener.waitForTransform('/map', '/camera_optical_link', rospy.Time.now()+rospy.Duration(1), rospy.Duration(10))

  listener = TF_stuff()

  current = QRTransformEstablisher()
  scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
  cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
  robo_pos_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, robo_pos_callback) 
  qr_data_sub = rospy.Subscriber('/visp_auto_tracker/code_message', String, qr_data_callback, [current, cmd_vel_pub, listener], queue_size=10)
  qr_live_data_sub = rospy.Subscriber('/visp_auto_tracker/code_message', String, qr_live_data_callback, current, queue_size=10)
  qr_position_sub = rospy.Subscriber('/visp_auto_tracker/object_position', PoseStamped, qr_position_callback, current, queue_size=10)
  qr_status_sub = rospy.Subscriber('/visp_auto_tracker/status',Int8, qr_status_callback, current, queue_size=10 )
  rospy.on_shutdown(shutdown_fun)
  

  state_change_time = rospy.Time.now() + rospy.Duration(1)
  driving_forward = True
  rate = rospy.Rate(60)
  area = 1
  end_points = [[(-4.0, 0.0, 0.0), (0,0,0,1)],[(6.0, -1.0, 0.0), (0,0,0,1)]]
  
  while not rospy.is_shutdown():
    if len(current.codes) == 5:
      for i in range(1,6):
        for code in current.codes:
          if int(code.data["N"]) == i:
            print "N: ", code.data["N"], ",  Letter: ", code.data["L"]
      print "execution time ", datetime.datetime.now()-starting_datetime
      rospy.signal_shutdown("byebye")
    if len(current.codes)>=2:
      move_to_next(current)
      continue
    if (rospy.Time.now().secs - current.time_since_qr.secs) > 120:
      print "changing end" ,datetime.datetime.now()
      current.in_transit = True
      current.stop = True
      goal = goal_pose(end_points[area%2])
      current.client.send_goal(goal)
      wait_status = current.client.wait_for_result(rospy.Duration(60))
      if current.in_transit:
        print "timeout cancelling goal"
      	current.client.cancel_goal()
      	current.in_transit = False
      rospy.sleep(3)
      area += 1
      current.time_since_qr = rospy.Time.now()
      current.stop = False
      print "changed end"
    if g_range_ahead < 1.0:
      # TURN
      driving_forward = False
    else: # we're not driving_forward
      driving_forward = True # we're done spinning, time to go forward!
      #DRIVE
    twist = Twist()
    if driving_forward:
      twist.linear.x = .3
      twist.angular.z = 0.0
    else:
      twist.linear.x = 0.0
      twist.angular.z = .1
    if not current.stop:
      cmd_vel_pub.publish(twist)
    rate.sleep()

  # END ALL