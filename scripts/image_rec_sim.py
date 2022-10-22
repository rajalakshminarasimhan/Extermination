#!/usr/bin/python2.7

# Written by Adam Casey
# March 2021
# This script publishes fake weed data to the /targets topic so we can test how the rest of our code responds

import tf2_ros
import rospy
import geometry_msgs
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import String

import random

def main():

  rospy.init_node("fake_image_rec", anonymous=True)

  weedPub = rospy.Publisher("/targets", String, queue_size=100)

  tfBuffer = tf2_ros.Buffer()
  listener = tf2_ros.TransformListener(tfBuffer)

  rate = rospy.Rate(2)

  weed_index = 0

  stb = tf2_ros.StaticTransformBroadcaster()

  # Run the loop at 1 Hz (one time per second)
  rate = rospy.Rate(1)

  while not rospy.is_shutdown():
    try:
      # Find the chassis in the world frame
      chassis_pos = tfBuffer.lookup_transform("world", "chassis", rospy.Time(0))

      weed_pos = PointStamped()
      
      weed_pos.header.stamp = rospy.Time.now()
      weed_pos.header.frame_id = "world"

      # Generate a random location to 'find' the weed, 1+/-1 m in from of the chassis, and +/-0.75 m left or right of the chassis centre
      # TODO I picked those distance randomly, so once we know the field-of-view of the actual image rec system, we should change this to match
      weed_pos.point.x = chassis_pos.transform.translation.x + 1+random.random()
      weed_pos.point.y = chassis_pos.transform.translation.y + 1.5*(random.random() - 0.5)
      weed_pos.point.z = 0

      # Publishes the point so the rest of our code is tricked into thinking we actually found a weed
      #----weedPub.publish(weed_pos)

      #Also add to TF2 so we can see in RVIZ
      # TODO I think there's a way of setting up RVIZ to show the Points without making them frames like this
      weed_frame = tf2_ros.TransformStamped()

      weed_frame.header.stamp = rospy.Time.now()
      weed_frame.header.frame_id = "world"
      weed_frame.child_frame_id = "weed{}".format(weed_index)
      weedPub.publish("weed{}".format(weed_index))
      # Makes the weed_frame have the same location as the weed_pos we created earlier, and with no rotation relative to world
      weed_frame.transform.translation = weed_pos.point
      weed_frame.transform.rotation.x = 0
      weed_frame.transform.rotation.y = 0
      weed_frame.transform.rotation.z = 0
      weed_frame.transform.rotation.w = 1

      stb.sendTransform(weed_frame)

      weed_index = weed_index + 1

      # Waits the correct amount of time so that the loop runs at 2 Hz
      rate.sleep()

    except tf2_ros.LookupException as e:
      # This fails if this node starts up before the node that creates the chassis frame
      # because no transform is available yet between world and chassis
      # in this case the code pases and tries again.
      pass

    rate.sleep()
    

if __name__ == "__main__":
  main()
