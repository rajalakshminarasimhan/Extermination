#!/usr/bin/python2.7

# Written by Adam Fong
# December 2021
# This script is a sorting algorithm that does not depend on calling transforms on weeds to sort. Transforms will only be called for each sprayer.

import tf2_ros
import rospy
from std_msgs.msg import String # String is how we receive the weed name
import geometry_msgs
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import TransformStamped
import heapq
import numpy as np

import threading
# to gain some momentum, let's just assume that there are two nozzles, one responsible for the left side of the chassis and one for the right
# The bot moves in the positive x direction, therefore the positive y axis is the left side of the bot when looking in the direction of travel 


current_weeds = [] # used as a heapq to sort all incoming weeds found from target_listener

N_SPRAYERS = 9
VELOCITY = rospy.get_param("bot_velocity") # currently 0.5 m/s 
SPRAYER_MAIN_FREQ = 1.0 / 4.0 # 4 Hz (means each sprayer must spray for 0.25s to exterminate the weed)
TARGET_LISTENER_FREQ = 1.0 / 100.0 # 100 Hz

def replace_char_at_index(org_str, index, replacement):
    ''' 
    Replace character at index in string org_str with thegiven replacement character.
    Used when editing the string to publish to sprayers GPIO
    '''
    new_str = org_str
    if index < len(org_str):
        new_str = org_str[0:index] + replacement + org_str[index + 1:]
    return new_str

class Weed:
    def __init__(self, id):
        self.id = id
        
    def get_pose(self): 
        """
        saves the position of a weed in relation to the world
        """
        weed = tfBuffer.lookup_transform("world", self.id, rospy.Time())

        self.x = weed.transform.translation.x
        self.y = weed.transform.translation.y
        self.z = weed.transform.translation.z
        self.pose = [self.x, self.y, self.z]

class ArrayofSprayers:
    def __init__(self):
        # instantiate all sprayer states to be '0' or off 
        self.state = "000000000"
        self.calculate_sprayer_positions()


    def calculate_sprayer_positions(self):
        """
        Uses the width of the bot to calculate the positions of equidistant sprayers. Assigns positions in np.array to self.positions. 
        Returns: void
        """
        width = rospy.get_param("bot_width")
        step = width / (N_SPRAYERS - 1)

        # accounts for range to end before the 'end' bound
        positions = np.arange(0, width + step, step) # each position in 'positions' is the offset from the left side of the frame which is parallel to travel 
        
        # because the tf location of the chassis is at the centerpoint, shift all positions in 'positions' to have the middle sprayer at location 0
        self.positions = positions - 0.325

        # positions == |  -0.325  |  -0.24375  |  -0.1625  |  -0.08125  |  0.  |  0.08125  |  0.1625  |  0.24375  |  0.325  |

    def actuate_sprayers(self):
        """
        Method to interact with GPIO for sprayers. Publishes to topic which GPIO listens to. 
        msg.data = "000000000" indicates all sprayers off
        msg.data = "111111111" indicates all sprayers on
        """
        sprayerPub = rospy.Publisher("sprayers_actuate", String, queue_size = 100)

        # send a string with N_SPRAYERS count of characters. 
        # to actuate an indexed sprayer, send a 1 at the specific index, otherwise, send a 0 at the index
        msg = String()
        
        msg.data = self.state

        sprayerPub.publish(msg)

    def close_all_sprayers(self):
        self.state = "000000000"
        self.actuate_sprayers()

    def assign_weed_to_sprayer(self, weed_y, chassis_y):
        """ 
        Sorts a weed to the correct sprayer. Assigns the sprayer closest to the weed to actuate. 
        """
        weed_wrt_chassis = chassis_y - weed_y # positive values are to the right side of the chassis, neg to the left, zero in the center 
        distances = np.abs(self.positions - weed_wrt_chassis) # find how close each sprayer is to the weed

        # find the index of the smallest distances (refer: https://stackoverflow.com/questions/34226400/find-the-index-of-the-k-smallest-values-of-a-numpy-array)
        k = 3 
        smallest_idxs = np.argpartition(distances, k)
        idx = smallest_idxs[0] # smallest index of sprayer 

        # change state of the sprayer nearest to weed to 1
        self.state = replace_char_at_index(self.state, idx, "1")


def sprayer_controller(event=None):
    """
    Acts as the main function to anything which involves sprayers in this script. Configured to be a callback for threading. 
    """
    print("entered sprayer main")
    rate = rospy.Rate(4) # 4 Hz

    distance_to_be_traveled = VELOCITY * SPRAYER_MAIN_FREQ

    sprayers = ArrayofSprayers()

    # get instantaneous chassis position 
    while not rospy.is_shutdown():
        try:
            chassis_pos = tfBuffer.lookup_transform("world", "chassis", rospy.Time(0))  

            # find the weeds that need to be immediately exterminated
            if len(current_weeds) != 0: # current_weeds[0][0] is effectively doing a 'peek' of the heap
                while (current_weeds[0][0] <= chassis_pos.transform.translation.x + distance_to_be_traveled):
                    _, weed = heapq.heappop(current_weeds)
                    print("Exterminating: {id}".format(id = weed.id))

                    sprayers.assign_weed_to_sprayer(weed.y, chassis_pos.transform.translation.y)
                    sprayers.actuate_sprayers()
                    # prevents calling the index of an empty list 
                    if(len(current_weeds) == 0): 
                        break
            
        except (tf2_ros.LookupException):
            rospy.logwarn("lookup")
        except (tf2_ros.ConnectivityException):
            rospy.logwarn("connectivity")
        except (tf2_ros.ExtrapolationException):
            rospy.logwarn("extrapolation")
    
        rate.sleep()
        sprayers.close_all_sprayers()

def add_weed(weed):

    try: # obtaining weed location from tf2
        rospy.loginfo(weed.data)	
        weedObj = Weed(weed.data)
        weedObj.get_pose()

        heapq.heappush(current_weeds, (weedObj.x, weedObj))

        #rospy.loginfo(current_weeds[0][0]) # y position

        rospy.loginfo("Current Weeds: {}".format(len(current_weeds)))

        #find the weed transform, relative to the world so we can continue referencing them as the chassis moves

    except (tf2_ros.LookupException):
        rospy.logwarn("lookup")
    except (tf2_ros.ConnectivityException):
        rospy.logwarn("connectivity")
    except (tf2_ros.ExtrapolationException):
        rospy.logwarn("extrapolation")

def target_listener(event=None):
    rospy.Subscriber("/targets", String, add_weed)
    rospy.loginfo('Target Listener Initiated')

if __name__ == "__main__":
    # use timers to call multiple callbacks in this node 
    rospy.init_node("target_manager")

    # initialize buffer and tf2 requirements
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    #stb = tf2_ros.StaticTransformBroadcaster() # I don't believe we need a broadcaster as we aren't updating frames 

    # define threads for the target listener and sprayer controller 
    target_listener_thread = threading.Thread(target = target_listener)
    sprayer_thread = threading.Thread(target = sprayer_controller)
    
    # start threads 
    target_listener_thread.start()
    sprayer_thread.start()

    rospy.spin()
