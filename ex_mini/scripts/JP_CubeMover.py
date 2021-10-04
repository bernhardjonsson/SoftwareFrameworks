#!/usr/bin/env python
import roslib
roslib.load_manifest('ex_mini')
 

import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseArray, Pose
import shape_msgs.msg as shape_msgs
from sensor_msgs.msg import JointState
from numpy import zeros, array, linspace
import math
 
from std_msgs.msg import String

currentJointState = JointState() 
haveBucket = False
Busy = False

def jointStatesCallback(msg):
  global currentJointState
  currentJointState = msg
  
def Move_Arm(group, robot, display_trajectory_publisher, scene, pose_x, pose_y, pose_z, orientation_x, orientation_w):
  ## ======================
  ## Planning to 1 - pose goal
  ## ======================
  print "============ Generating plan 1"
  pose_goal = group.get_current_pose().pose
  print "===== Init POSE: "
  print pose_goal
  waypoints = []
  waypoints.append(pose_goal)
  pose_goal.position.x =pose_x
  pose_goal.position.y =pose_y
  pose_goal.position.z =pose_z
  if(orientation_x != 0):
    pose_goal.orientation.x = 0.0
    pose_goal.orientation.y = -0.707106781187
    pose_goal.orientation.z = 0.0
    pose_goal.orientation.w = 0.707106781187
  
  print pose_goal
 
  #Create waypoints
  waypoints.append(pose_goal)
 
  #Createcartesian  plan
  (plan1, fraction) = group.compute_cartesian_path(
                                      waypoints,   # waypoints to follow
                                      0.01,        # eef_step
                                      0.0)         # jump_threshold
  print("Found path fraction: ", fraction)
  #print("our plan is :" + plan1)
  #plan1 = group.retime_trajectory(robot.get_current_state(), plan1, 1.0)
  group.set_pose_target(pose_goal)
  #plan1 = group.plan()
  
  
  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(2.5) #used to be 0.5
 
 
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
  print "============ Visualizing plan 1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan1)
  display_trajectory_publisher.publish(display_trajectory);
  print "============ Waiting while plan 2 is visualized (again)..."
  rospy.sleep(2.5)
  
  #If we're coming from another script we might want to remove the objects
  if "table" in scene.get_known_object_names():
    scene.remove_world_object("table")
  if "table2" in scene.get_known_object_names():
    scene.remove_world_object("table2")
  if "groundplane" in scene.get_known_object_names():
    scene.remove_world_object("groundplane")
  print "============ Waiting while Executing Plan..."
  group.execute(plan1,wait=True)
  print "============ Executed Plan..."
  #group.go(wait=True)
  rospy.sleep(10.)
 
def Gripper_Close(JointState, pub):
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  tmp = 0.7
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    print 'Published!'
    rate.sleep()
  print 'Gripper Closed!'
  currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  
def Gripper_Open(JointState, pub):
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  tmp = 0.005
  #tmp_tuple=tuple([tmp] + list(currentJointState.position[1:]))
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    print 'Published!'
    rate.sleep()
  print 'Gripper Opened!'
  currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 
def JP_Cube_Mover():
  ## Initialize moveit_commander and rospy.
  print "============ Move Box setup"
  moveit_commander.roscpp_initialize(sys.argv)
  #rospy.init_node('Cube_Mover2',
  #                anonymous=True)
  global robot; global scene; global group
  robot = moveit_commander.RobotCommander()
  scene = moveit_commander.PlanningSceneInterface()
  group = moveit_commander.MoveGroupCommander("Arm") #Control the arm
 
  ## Enable RVIZ to visualize trajectories
  global display_trajectory_publisher
  display_trajectory_publisher = rospy.Publisher(
                                      '/move_group/display_planned_path',
                                      moveit_msgs.msg.DisplayTrajectory)
 
 
  ## Print INFO
  print "============ Starting Program "
  print "============ Reference frame: %s" % group.get_planning_frame() ## Get the name of the reference frame for this robot
  print "============ End effector frame: %s" % group.get_end_effector_link() ## Get the name of the end-effector link for this group
  print "============ Robot Groups:" ## Get a list of all the groups in the robot
  print robot.get_group_names()
  print "============ Printing robot state"
  print robot.get_current_state()   ## Get the entire state of the robot.
  print "============"
 
 
  ## Plannet Setup:
  #group.set_planning_time(0.0)
  group.set_goal_orientation_tolerance(0.02)
  group.set_goal_tolerance(0.02)
  group.set_goal_joint_tolerance(0.03)
  group.set_num_planning_attempts(100) #Not supported anymore
  group.set_max_velocity_scaling_factor(1.0)
  group.set_max_acceleration_scaling_factor(1.0)
  
  
  ## Grapper Setup:
  global pub
  pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)

  ##Subscriber to listen for cube positons
  rospy.Subscriber("/CubePos", PoseArray, moveCubes)
  
  #TEST coordinates to be read in:
  init_pose = group.get_current_pose().pose
  print init_pose
 # Move_Arm(group, robot, display_trajectory_publisher, scene, init_pose.position.x, init_pose.position.y, init_pose.position.z+0.2, 0, -1.57, 0)  
  
  ## FICTIVE WHILE LOOP FOR EACH BOX:                                                                                                                                             #Move The Arm
  # Move_Arm(group, robot, display_trajectory_publisher, scene, box_x, box_y, box_z+0.2, 0, -1.57, 0)        #Move above box
  # Move_Arm(group, robot, display_trajectory_publisher, scene, box_x, box_y, box_z, 0, -1.57, 0)               #Move down to box
  # Gripper_Close(JointState, pub)                                                                                                 #Grap box
  # Move_Arm(group, robot, display_trajectory_publisher, scene, bucket_x, bucket_y, bucket_z, 0, -1.57, 0)  #Move above bucket_x
  # Gripper_Open(JointState, pub)                                                                                                  #Open grapper
  ## REPEAT FOR EACH BOX:
  
  
 
 
 
 
  ## END_TUTORIAL
  print "============ STOPPING"
  R = rospy.Rate(2)
  while not rospy.is_shutdown():
    R.sleep()

def moveCubes(posArray):
  global Busy
 # if(not Busy):
 #   Busy = True
  global bucket_x; global bucket_y; global bucket_z
  global robot; global scene; global group
  for pose in posArray.poses:
    try:
      Move_Arm(group, robot, display_trajectory_publisher, scene, pose.position.x, pose.position.y, pose.position.z+0.2, 1,0)        #Move above box
    except Exception as e:
      print("Caught error", e)
      #continue
    Move_Arm(group, robot, display_trajectory_publisher, scene, pose.position.x, pose.position.y, pose.position.z, 1,0)               #Move down to box
    Gripper_Close(JointState, pub)                                                                                                 #Grap box
    Move_Arm(group, robot, display_trajectory_publisher, scene, pose.position.x, pose.position.y, pose.position.z+0.4, 0, -1.57) 
    print("Moving to bucket positioned at x:", bucket_x, ", y: ", bucket_y, ", z:", bucket_z)
    Move_Arm(group, robot, display_trajectory_publisher, scene, bucket_x, bucket_y, bucket_z + 0.4, 1, 0)  #Move above bucket_x
    Gripper_Open(JointState, pub) 
  #    Busy = False

    ## DONE - Shut down moveit_commander.
  moveit_commander.roscpp_shutdown()
def defineBucket(bucketPos):
  global bucket_x; global bucket_y; global bucket_z
  bucket_x = bucketPos.position.x
  bucket_y = bucketPos.position.y
  bucket_z = bucketPos.position.z
  global haveBucket
  if(not haveBucket):
    JP_Cube_Mover()
    haveBucket = True


def lookForBucket():
  rospy.init_node('CubeMover', anonymous=True) #No need to be anonymous I guess
  rospy.Subscriber("/BucketPos", Pose, defineBucket)
  R = rospy.Rate(2)
  while not rospy.is_shutdown():
    R.sleep()
  



if __name__=='__main__':
  #global haveBucket
  #haveBucket = False
  try:
     lookForBucket()
  except rospy.ROSInterruptException:
    pass


