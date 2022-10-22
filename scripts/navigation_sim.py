#!/usr/bin/python2.7

# Written by Adam Casey
# March 2021
# This scripts publishes fake navigation data, as if the chassis was driving in a straight line
# This can be useful for testing the extermination algorithms

import tf2_ros
import rospy
from geometry_msgs.msg import TransformStamped


def main():

  rospy.init_node("fake_chassis", anonymous=True)

  #chassisPub = rospy.Publisher("/chassis_pose", PoseStamped, queue_size = 10)
  br = tf2_ros.TransformBroadcaster()

  # TODO these should be pulled from the parameter server, not hard-coded in
  # Where is the "centre" of the chassis, relative to the "centre" of the world frame
  chassis_x_offset = 0
  chassis_y_offset = 0
  chassis_height = rospy.get_param("bot_height")

  # Run at 50 Hz / 50 times per second
  rate = rospy.Rate(50)

  while not rospy.is_shutdown():
    chassis_pose = TransformStamped()

    chassis_pose.header.stamp = rospy.Time.now()
    chassis_pose.header.frame_id = "world"
    chassis_pose.child_frame_id = "chassis"

    chassis_pose.transform.translation.x = chassis_x_offset
    chassis_pose.transform.translation.y = chassis_y_offset
    chassis_pose.transform.translation.z = chassis_height

    chassis_pose.transform.rotation.x = 0
    chassis_pose.transform.rotation.y = 0
    chassis_pose.transform.rotation.z = 0
    chassis_pose.transform.rotation.w = 1

    br.sendTransform(chassis_pose)

    # simulate the bot moving at ~0.5 m/s (approximately 1/2 walking speed)
    chassis_x_offset = chassis_x_offset + 0.01
    # Wait long enough for the loop to run at 20 Hz
    rate.sleep()

if __name__ == "__main__":
  main()
