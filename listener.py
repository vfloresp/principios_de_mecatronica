#!/usr/bin/env python
# license removed for brevity
import rospy
from geometry_msgs.msg import Pose2D
from graphical_client.msg import Pose2D_Array

def callback(data):
    rospy.loginfo(data.poses[0])
    #pose = Pose2D()
    #pose.x = 0
    #pose.y = 0
    #pose.theta = 0

  
def listener():
  rospy.init_node('listener', anonymous=True)
  rospy.Subscriber("/trajectory", Pose2D_Array, callback)
  rospy.spin()

if __name__ == '__main__':
 listener()