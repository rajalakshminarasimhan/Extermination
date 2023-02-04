#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool
import RPi.GPIO as GPIO

BUTTON_GPIO = 16

if __name__ == '__main__':
	rospy.init_node('button_state_publisher')
	# defining what topic to publish to 
	pub = rospy.Publisher('button_state', Bool, queue_size = 10)
	
	# setting up hardware to detect button presses
	GPIO.setmode(GPIO.BCM)
	GPIO.setup(BUTTON_GPIO, GPIO.IN, pull_up_down = GPIO.PUD_UP)
	
	# setting rospy rate on how often to publish the message
	rate_when_pos =  rospy.Rate(1)
	rate_when_neg = rospy.Rate(1)
	# forever loop to run this node during the entire running lifetime of pi
	while not rospy.is_shutdown():
		gpio_state = not GPIO.input(BUTTON_GPIO)
		# for send other datatypes through messages,
		#  look at std_msgs for other constructors
		msg = Bool()
		msg.data = gpio_state
		pub.publish(msg)
		if msg.data == 1:	
			rate_when_pos.sleep()
		else:
			rate_when_neg.sleep()

	GPIO.cleanup()
