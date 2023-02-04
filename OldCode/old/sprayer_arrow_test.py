#!/usr/bin/python2.7

# Written by Justin Lim
# May 2021
# This script publishes one weed on the left of the bot to test whether the arrow follows the weed correctly

import tf2_ros
import rospy
import geometry_msgs
from geometry_msgs.msg import TransformStamped

def main():
    #initialize node
    rospy.init_node("one_weed_test", anonymous=True)

    #create broadcaster and transform msg 
    sbr = tf2_ros.StaticTransformBroadcaster()
    weed_frame = geometry_msgs.msg.TransformStamped()

    weed_frame.header.stamp = rospy.Time.now();
    weed_frame.header.frame_id = "world"
    weed_frame.child_frame_id = "testweed"
    weed_frame.transform.translation.x = 5.0;
    weed_frame.transform.translation.y = 1.0;
    weed_frame.transform.translation.z = 0.0;
    weed_frame.transform.rotation.x = 0.0;
    weed_frame.transform.rotation.y = 0.0;
    weed_frame.transform.rotation.z = 0.0;
    weed_frame.transform.rotation.w = 1.0;

    sbr.sendTransform(weed_frame)
    rospy.spin()


if __name__ == '__main__':
    main()
