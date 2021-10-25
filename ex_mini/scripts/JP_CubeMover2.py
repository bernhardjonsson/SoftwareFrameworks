#!/usr/bin/env python
import roslib
roslib.load_manifest('ex_mini')
 

import sys
import copy
import rospy
import tf_conversions
import moveit_commander
import moveit_msgs.msg
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
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

def Vizualize_Plan(plan):
  print "============ Waiting while RVIZ displays plan1..."
  rospy.sleep(0.5) #used to be 0.5
  ## You can ask RVIZ to visualize a plan (aka trajectory) for you.
  print "============ Visualizing plan 1"
  display_trajectory = moveit_msgs.msg.DisplayTrajectory()
  display_trajectory.trajectory_start = robot.get_current_state()
  display_trajectory.trajectory.append(plan)
  display_trajectory_publisher.publish(display_trajectory);
  print "============ Waiting while plan 2 is visualized (again)..."
  rospy.sleep(1)
  
  #If we're coming from another script we might want to remove the objects, we can probably remove this for tidiness
  if "table" in scene.get_known_object_names():
    print("Table got removed")
    scene.remove_world_object("table")
  if "table2" in scene.get_known_object_names():
    scene.remove_world_object("table2")
    print("Table2 got removed")
  if "groundplane" in scene.get_known_object_names():
    scene.remove_world_object("groundplane")
    print("GroundPlane got removed")

  
def Move_Arm(group, robot, display_trajectory_publisher, scene, pose_x, pose_y, pose_z, orientation_x, orientation_w):
  ## ======================
  ## Planning path
  ## ======================
  print "============ Generating plan"
  pose_goal = group.get_current_pose().pose
  print "===== Init pose: "
  print pose_goal
  waypoints = [] #Create an array for our waypoints
  waypoints.append(pose_goal) #Set our initial point
  pose_goal.position.x =pose_x
  pose_goal.position.y =pose_y
  pose_goal.position.z =pose_z
  pose_goal.orientation.x = 0.0 #Hard code our goal orientation to pointing directly downwards, should check cube orientation instad
  pose_goal.orientation.y = -0.707106781187
  pose_goal.orientation.z = 0.0
  pose_goal.orientation.w = 0.707106781187
  
  print pose_goal
 
  #Create waypoints
  waypoints.append(copy.deepcopy(pose_goal))
 
  #Create cartesian  plan
  attempts = 0; fraction = 0 #Create an attempts variable to break out of while loop eventually
  while fraction < 0.99 and attempts < 100:
    (plan, fraction) = group.compute_cartesian_path(
                                    waypoints,   # waypoints to follow
                                    0.01 - attempts/20000,        # eef_step, Try smaller step sizes as we go up in attemps,
                                    0.0)         # jump_threshold
    print("Found path fraction: ", fraction, "in attempt number", attempts)
    attempts += 1 
  
  #Visualizing Plan with RVIZ
  Vizualize_Plan(plan)
 
  print "============ Waiting while Executing Plan..."
  group.execute(plan,wait=True)
  print "============ Executed Plan..."
  rospy.sleep(2.)
 
def Gripper_Close(JointState, pub):
  currentJointState = rospy.wait_for_message("/joint_states",JointState)
  print 'Received!'
  currentJointState.header.stamp = rospy.get_rostime()
  tmp = 0.7
  currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp] + [tmp] + [tmp] + [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(6):
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
  currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  currentJointState.position = tuple(list(currentJointState.position[:6]) + [tmp] + [tmp]+ [tmp])
  rate = rospy.Rate(10) # 10hz
  for i in range(3):
    pub.publish(currentJointState)
    print 'Published!'
    rate.sleep()
  print 'Gripper Opened!'
  currentJointState.velocity = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
 
 
def Move_Arm_To_Init(group, robot, display_trajectory_publisher, scene):
    #Set tolerances to avoid error
    group.set_goal_orientation_tolerance(0.2)
    group.set_goal_tolerance(0.2)
    group.set_goal_joint_tolerance(0.2)
    Move_Arm(group, robot, display_trajectory_publisher, scene, 0.40, -0.1, 1.2, 1., 0.)
    #Move to init with higher precision
    group.set_goal_orientation_tolerance(0.02)
    group.set_goal_tolerance(0.02)
    group.set_goal_joint_tolerance(0.01)
    Move_Arm(group, robot, display_trajectory_publisher, scene, 0.40, -0.1, 1.2, 1., 0.)
  
 
 
def JP_Cube_Mover():
  ## Initialize moveit_commander and rospy.
  print "============ Move Box setup"
  moveit_commander.roscpp_initialize(sys.argv)
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
  group.set_goal_orientation_tolerance(0.02)
  group.set_goal_tolerance(0.02)
  group.set_goal_joint_tolerance(0.03)
  
  
  ## Grapper Setup:
  global pub
  pub = rospy.Publisher("/jaco/joint_control", JointState, queue_size=1)

  #Move arm to init pose
  Move_Arm_To_Init(group, robot, display_trajectory_publisher, scene)
  Gripper_Open(JointState, pub)


  ##Start Subscriber to listen for cube positons
  rospy.Subscriber("/CubePos", PoseArray, moveCubes)
  
  #print our initial Pose to check we have good movement
  init_pose = group.get_current_pose().pose
  print init_pose
  
  
 
 #Clean up our mess in Rviz
  cleanUp_RVIZ()
 
  ## END_TUTORIAL
  print "============ STOPPING"
  R = rospy.Rate(2)
  while not rospy.is_shutdown():
    R.sleep()

def cleanUp_RVIZ():
  models_in_scene = [s for s in scene.get_known_object_names() if "cube" or "bucket" in s]
  print("cleaning up models:")
  print(models_in_scene)
  for name in models_in_scene:
    scene.remove_world_object(name)

def moveCubes(posArray):
  global Busy
  global RunOnceFlag
  global bucket_x; global bucket_y; global bucket_z; global bucketPose
  global robot; global scene; global group
  
  #Add bucket to rviz
  p = PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  p.pose = bucketPose
  p.pose.position.z += 0.09

 # p.pose.orientation = 
  scene.add_box('Bucket', p, (0.2, 0.2, 0.18)) #Use sizes from blender
  #Try spawning cubes in RVIZ because the hint told us to?
  
  p = PoseStamped()
  p.header.frame_id = robot.get_planning_frame()
  cube_counter = 0
  for pose in posArray.poses:
    p.pose.position = pose.position
    p.pose.orientation = pose.orientation
    scene.add_box('cube'+str(cube_counter), p, (0.05, 0.05, 0.05))
    cube_counter += 1

  for pose in posArray.poses:
    print("Moving to box positioned at x:", pose.position.x, ", y: ", pose.position.y, ", z:", pose.position.z)
    try:
      Move_Arm(group, robot, display_trajectory_publisher, scene, pose.position.x, pose.position.y, pose.position.z+0.3, 1,0)        #Move above box
    except Exception as e:
      print("Caught error", e)
      #continue
    Move_Arm(group, robot, display_trajectory_publisher, scene, pose.position.x, pose.position.y, pose.position.z, 1,0)               #Move down to box
    Gripper_Close(JointState, pub)                                                                                                 #Grap box
    Move_Arm(group, robot, display_trajectory_publisher, scene, pose.position.x, pose.position.y, pose.position.z+0.4, 0, -1.57) 
    print("Moving to bucket positioned at x:", bucket_x, ", y: ", bucket_y, ", z:", bucket_z)
    Move_Arm(group, robot, display_trajectory_publisher, scene, bucket_x, bucket_y, bucket_z + 0.4, 1, 0)  #Move above bucket_x
    Gripper_Open(JointState, pub) 

    ## DONE - Shut down moveit_commander.
  moveit_commander.roscpp_shutdown()

def defineBucket(bucketPos):
  global bucket_x; global bucket_y; global bucket_z; global bucketPose
  bucket_x = bucketPos.position.x
  bucket_y = bucketPos.position.y
  bucket_z = bucketPos.position.z
  bucketPose = bucketPos
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
  print "Main of JP_CubeMover2 Started"
  try:
     lookForBucket()
  except rospy.ROSInterruptException:
    pass


