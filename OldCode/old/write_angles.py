#!/usr/bin/python2.7
# This program takes two parameters, the angles for the
# two steppers to turn to, and writes them to the USB port
# We use it to tell the Arduino how to control the sprayer stepper motors / servos

import serial, sys
import rospy
from geometry_msgs.msg import Vector3

def writer(data):
  global ser
  command = "{:.2f},{:.2f}\n".format(data.x,data.y)
  ser.write(command.encode())

  print(command)

def listener():
  # Ensure this port is correct. On our RasPi, it's the top left USB port
  # The other ports might be called "/dev/ttyACM1" or something like that
  # You can check the /dev directory on the RasPi with a command like
  #
  #   ls /dev | grep ttyACM
  #
  # (run this ^^^ in a terminal on the RasPi)

  global ser
  ser = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

  # TODO we'll need one of these for each sprayer, so can't hard code in "sprayer1" and "angles1"
  # Should pass as an argument from the launch file
  rospy.init_node("sprayer1", anonymous=True)
  rospy.Subscriber("angles1", Vector3, writer)
  # TODO using a Vector3 ^^^ to hold two angle variables is terrible and we should probably change it

  rospy.spin() 

if __name__ == "__main__":
  listener()
