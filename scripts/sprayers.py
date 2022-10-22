#!/usr/bin/python2.7

# Written by Adam Fong
# December 2021
# This script is the placeholder for what will be the actuation of the array of sprayers
# Keep it simple, this script turns on a sprayer with a 1

# 9 total sprayers to spray

import tf2_ros
import rospy
from std_msgs.msg import String # String is how we receive the weed name

N_SPRAYERS = 9
sprayers_list = []

class Sprayer:
    def __init__(self, id):
        self.id = id

    # TODO: replace with connection to a node that handles spayer actuation with GPIO
    def actuate(self):
        rospy.loginfo("{id} Actuated for 0.5s".format(id = self.id))


def create_sprayers():
    # instantiate all sprayers in a list so we can index & actuate 
    for val in range(0, N_SPRAYERS, 1):
        id = "sprayer_{}".format(val)
        sprayers_list.append(Sprayer(id))

def sprayers(data):
    # iterate through string message and actuate if the indexed location != 0 
    for sprayer_idx in range(len(data.data)):
        ch = data.data[sprayer_idx]
        # 0 == do not actuate
        if int(ch) == 0:
            continue
        # not 0 == actuate
        else:
            sprayers_list[sprayer_idx].actuate()
                
def main():
    print('entered main')
    rospy.Subscriber(topic, String, sprayers)
    rospy.spin()

if __name__ == '__main__':
    # initialize node
    rospy.init_node("sprayers_listener", anonymous = True)
    topic = "/sprayers_actuate"
    rospy.loginfo('Intiated Node: sprayers_listener')

    create_sprayers()
    main()
