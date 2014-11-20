#!/usr/bin/env python
import rospy
import roslib
roslib.load_manifest('rospid')
import rospidlib
from turtlesim.msg import Pose, Velocity
from math import sqrt

rospy.init_node('testpid')
steering_pid = rospidlib.Rospid(0.15,0.0,0.22,'~steering')
vel_pub = rospy.Publisher('turtle1/command_velocity', Velocity)

def pose_callback(data):
  # centre on 5,7
  dx = data.x - 5.0
  dy = data.y - 7.0
  # calculate radius
  radius = sqrt(dx*dx + dy*dy)
  # PID control for constant radius
  u = steering_pid.update(radius, 2.5, rospy.get_rostime().to_sec())
  # command
  v = Velocity()
  v.linear = 0.3
  v.angular = -u
  # send it
  vel_pub.publish(v)
  # tell the world
  rospy.loginfo('Got pos = (%f,%f) rad=%f ctrl=%f', dx, dy, radius, u)

pose_sub = rospy.Subscriber('turtle1/pose', Pose, pose_callback)
rospy.spin()