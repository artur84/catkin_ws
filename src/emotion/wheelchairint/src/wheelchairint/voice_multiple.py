#!/usr/bin/env python

"""
voice_cmd_vel.py is a simple demo of speech recognition.
  You can control a mobile base using commands found
  in the corpus file.
"""

import roslib
import rospy
import math

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from std_msgs.msg import Int32

class voice_cmd_vel:

	def __init__(self):
		rospy.on_shutdown(self.cleanup)
		self.speed = 0.1
		self.msg = Twist()
		self.headcontrol = Int32()
		self.headcontrol = 0
		self.forward = 1

        # publish to cmd_vel, subscribe to speech output
		self.pub_ = rospy.Publisher('voice_vel', Twist, queue_size=10)
		self.pub2_ = rospy.Publisher('signal', Int32, queue_size=10)
		rospy.Subscriber('recognizer/output', String, self.speechCb)

		r = rospy.Rate(10.0)
		while not rospy.is_shutdown():
			self.pub2_.publish(self.headcontrol)
			self.pub_.publish(self.msg)
			r.sleep()
        
	def speechCb(self, msg):
		rospy.loginfo(msg.data)
		if msg.data.find("go") > -1:
			self.forward = 1
			if self.msg.linear.x <= 0.0:
				self.msg.linear.x = 0.1
			self.msg.angular.z = 0
					
		if msg.data.find("stop") > -1:
			self.forward = 1
			self.msg.linear.x = 0
			self.msg.angular.z = 0
				
		if msg.data.find("reverse") > -1:
			self.forward = 0
			if self.msg.linear.x >= 0.0:
				self.msg.linear.x = -0.1
			self.msg.angular.z = 0.0
			
		if msg.data.find("faster") > -1:
			if self.forward == 1:
				self.msg.linear.x += 0.1
			else:
				self.msg.linear.x -= 0.1
			
		if msg.data.find("slower") > -1:
			if self.forward == 1:
				self.msg.linear.x -= 0.1
				if self.msg.linear.x < 0:
					self.msg.linear.x = 0
			else:
				self.msg.linear.x += 0.1
				if self.msg.linear.x > 0:
					self.msg.linear.x = 0
			
		if self.headcontrol == 0:
			if msg.data.find("left") > -1:
				if self.msg.angular.z >= 0:
					self.msg.angular.z += 0.1
				else:
					self.msg.angular.z = 0.1
			if msg.data.find("right") > -1:
				if self.msg.angular.z <= 0:
					self.msg.angular.z -= 0.1
				else:
					self.msg.angular.z = -0.1
			
		if msg.data.find("head control") > -1:
			self.msg.angular.z = 0
			self.headcontrol = 1
			
		if msg.data.find("voice control") > -1:
			self.headcontrol = 0

		self.pub_.publish(self.msg)
		self.pub2_.publish(self.headcontrol)

	def cleanup(self):
        # stop the robot!
		int = Int32()
		twist = Twist()
		self.pub_.publish(twist)
		self.pub2_.publish(int)

if __name__=="__main__":
    rospy.init_node('voice_multiple')
    try:
        voice_cmd_vel()
    except:
        pass

