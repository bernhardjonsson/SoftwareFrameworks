#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import PoseArray, Pose


RandoPoints = [[0.45, -0.15, 1.2],[ 0.8, 0.15, 1.2], [-0.45, 0.15, 1.2], [0.15, 0.8, 1.2]]
def talker():
    pub = rospy.Publisher('poses', PoseArray, queue_size=1)
    rospy.init_node('fakePosePublisher', anonymous=True)
    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        poseArray = PoseArray()
        for points in RandoPoints:
            thisPoint = Pose()
            thisPoint.position.x = points[0]
            thisPoint.position.y = points[1]
            thisPoint.position.z = points[2]
            thisPoint.orientation.x = 0.0
            thisPoint.orientation.y = -0.707106781187
            thisPoint.orientation.z = 0.0
            thisPoint.orientation.w = 0.707106781187
            poseArray.poses.append(thisPoint)
        rospy.loginfo(poseArray)
        pub.publish(poseArray)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass