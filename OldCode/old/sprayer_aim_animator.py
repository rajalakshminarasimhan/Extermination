#!/usr/bin/python2.7
# Written by Adam Casey
# March 2021
# This code is meant to take in the angle vectors published by sprayer, and convert them to PoseStamped so that rviz will render them as arrows, showing the direction the robot think the sprayer is point in rviz
# TODO As of March 27, 2021, this doesn't work. The conversion from angles to quaternion is wrong because of a quirk in how ROS does quaternion_from_euler()

import tf2_ros
import rospy
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from geometry_msgs.msg import PoseStamped
from numpy import deg2rad
import tf_conversions

def leftSprayerAnimator(angles):

  sprayerAim = PoseStamped()

  sprayerAim.header.stamp = rospy.Time.now()
  sprayerAim.header.frame_id = "left_sprayer"

  #TODO this doesn't work because these aren't euler angles ya dummy
  #q = tf_conversions.transformations.quaternion_from_euler(deg2rad(angles.y), deg2rad(90 - angles.x), 0)
  q = tf_conversions.transformations.quaternion_from_euler(deg2rad(45), deg2rad(90 - 0), deg2rad(0))

  sprayerAim.pose.position.x = 0
  sprayerAim.pose.position.y = 0
  sprayerAim.pose.position.z = 0

  sprayerAim.pose.orientation = Quaternion(*q)

  global pubLeft
  pubLeft.publish(sprayerAim)

# TODO there should only be one function (just "sprayerAnimator", not leftSprayerAnimator and rightSprayerAnimator, 
# and initialize two of this node in the launch file, passing 'left' or 'right' as an argument, like we do in other nodes
#def rightSprayerAnimator(angles):
#
#  sprayerAim = PoseStamped()
#
#  sprayerAim.header.stamp = rospy.Time.now()
#  sprayerAim.header.frame_id = "right_sprayer"
#
#  q = tf_conversions.transformations.quaternion_from_euler(deg2rad * angles.y,deg2rad * angles.x,deg2rad * angles.z)
#
#  sprayerAim.pose.position.x = 0
#  sprayerAim.pose.position.y = 0
#  sprayerAim.pose.position.z = 0
#
#  sprayerAim.pose.orientation.x = q[0]
#  sprayerAim.pose.orientation.y = q[1]
#  sprayerAim.pose.orientation.z = q[2]
#  sprayerAim.pose.orientation.w = q[3]
#
#  global pubRight
#  pubRight.publish(sprayerAim)

def main():

  rospy.init_node("sprayer_aim_animator", anonymous=True)

  global pubRight
  global pubLeft

  # TODO these should all be wrapped with an "if", or have their topics created programatically, 
  # so that we can pass "left" or "right as an argument from the launch file.
  rospy.Subscriber("left_sprayer_angles", Vector3, leftSprayerAnimator)
  rospy.Subscriber("right_sprayer_angles", Vector3, rightSprayerAnimator)

  pubLeft = rospy.Publisher("left_sprayer_aim",   PoseStamped, queue_size=10)
  pubRight = rospy.Publisher("right_sprayer_aim", PoseStamped, queue_size=10)

  rospy.spin()

if __name__ == "__main__":
  main()
