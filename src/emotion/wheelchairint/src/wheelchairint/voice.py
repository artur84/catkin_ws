#!/usr/bin/env python

"""
voice.py is a simple demo of speech recognition.
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
        self.dir = Twist()
        self.vel = Twist()
        self.forward = 1

        # publish to cmd_vel, subscribe to speech output
        self.vel_pub = rospy.Publisher('voice_vel', Twist, queue_size=10)
        self.dir_pub = rospy.Publisher('voice_dir', Twist, queue_size=10)
        rospy.Subscriber('recognizer/output', String, self.speechCb)

        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            self.vel_pub.publish(self.vel)  # vel_command should be continuosly published
            r.sleep()
        
    def speechCb(self, msg):
        rospy.loginfo(msg.data)
        if msg.data.find("go") > -1:
            self.forward = 1
            if self.vel.linear.x <= 0.0:
                self.vel.linear.x = 0.1
            self.vel.angular.z = 0
            self.dir.linear.x = 1.0
            self.dir.angular.z = 0
            self.dir_pub.publish(self.dir)
                    
        elif msg.data.find("brake") > -1:
            self.forward = 1
            self.vel.linear.x = 0
            self.vel.angular.z = 0
            self.dir.linear.x = 0
            self.dir.angular.z = 0
            self.dir_pub.publish(self.dir)
                
        elif msg.data.find("back") > -1:
            self.forward = 0
            if self.vel.linear.x >= 0.0:
                self.vel.linear.x = -0.1
            self.vel.angular.z = 0.0
            self.dir.linear.x = 1.0
            self.dir.angular.z = 3.14
            self.dir_pub.publish(self.dir)
            
        elif msg.data.find("left") > -1:
            if self.vel.angular.z >= 0:
                self.vel.angular.z += 0.1
            else:
                self.vel.angular.z = 0.1
            self.dir.linear.x = 1.0
            self.dir.angular.z = 1.57
            self.dir_pub.publish(self.dir)

        elif msg.data.find("right") > -1:
            if self.vel.angular.z <= 0:
                self.vel.angular.z -= 0.1
            else:
                self.vel.angular.z = -0.1
            self.dir.linear.x = 1.0
            self.dir.angular.z = -1.57
            self.dir_pub.publish(self.dir)
        else:
            rospy.loginfo("Not a recognized command")
            

    def cleanup(self):
        # stop the robot!
        twist = Twist()
        self.vel_pub.publish(twist)
        self.dir_pub.publish(twist)
        

if __name__ == "__main__":
    rospy.init_node('voice')
    try:
        voice_cmd_vel()
    except:
        pass

