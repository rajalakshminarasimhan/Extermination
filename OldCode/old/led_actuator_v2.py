#!/usr/bin/env python

import rospy
import RPi.GPIO as GPIO
from std_msgs.msg import Bool

LED_GPIO = 20
LED_MSG_OUTPUT = 26 
BUTTON = 19

def button_state_callback(msg):
	
	GPIO.output(LED_GPIO, msg.data)
	if msg.data == 1:
		q.addMessage(msg.data)

class MessageList:
	def __init__(self):
		self.list = []
		self.rate = rospy.Rate(1)
		self.counter = 0

	def addMessage(self, msg):
		self.list.append(msg)
		self.counter += 1		
		print("{c} messages received".format(c = self.counter))

	def dumpList(self):
		for val in self.list:
			GPIO.output(LED_MSG_OUTPUT, val)
			rate.sleep()
			GPIO.output(LED_MSG_OUTPUT, 0)
			rate.sleep()


# initialize node
if __name__ == '__main__':
	rospy.init_node('led_actuator')
	q = MessageList()

	GPIO.setmode(GPIO.BCM)
	GPIO.setup(LED_GPIO, GPIO.OUT)
	GPIO.setup(LED_MSG_OUTPUT, GPIO.OUT)
        GPIO.setup(BUTTON, GPIO.IN, pull_up_down = GPIO.PUD_UP)

	# subscribe to topic, write what message type is expecte from the topic
	# and what the callback function to handle a successfully read topic

	rospy.Subscriber('button_state', Bool, button_state_callback)

	# effectively creates a nice while loop that continues while the node is alive

	while not rospy.is_shutdown():
		gpio_state = not GPIO.input(BUTTON)
	
		if gpio_state == 0:
			q.rate.sleep()
		else:
			q.dumpList()

	GPIO.cleanup()


