#! /usr/bin/env python
#
import roslib;
from wheelchairint._keywords_to_command import *
from std_srvs.srv._Empty import Empty
from geometry_msgs.msg._Twist import Twist
from std_msgs.msg import String
import numpy as np
import rospy
from copy import deepcopy
import tf
import math
from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray
#=============================================================================
#==============================================================================
class VoiceAndHead():
    """This module reads the \head origin transformation published by the kinect head pose estimator
        and publishes the direction as a topic called voice_and_head_dir.
        It also publishes a topic called voice_and_head_vel that maps the given angle to angular velocity of the wheelchair. The linear velocity is controlled by the voice (/recognizer/output).
        "go,brake,back,faster,slower"   -> self.mode = "moving" -> angular_vel = k*head_dir, linear_vel = selected by voice
        "brake"                         -> self.mode = "brake"  -> angular_vel = 0, linear_vel = 0
        "turn"                          -> self.mode = "turn"   -> angular_vel = k*head_dir, linear_vel = 0

    """
    def __init__(self):
        """
        Constructor.
        @param
        @type
        """

        """ constants
        """
        self.max_theta_vel = 0.4
        self.zero_zone = 0.05
        self.yaw_limit =  math.pi
        self.max_pitch = math.pi
        self.min_pitch = -math.pi
        self.max_roll = math.pi
        self.min_roll = -math.pi
        """ Parameters
        """
        self.tf_prefix = rospy.get_param('tf_prefix', '')  # Reads the tf_prefix from the ROS namespace
        if self.tf_prefix is not '':
            self.tf_prefix = '/' + self.tf_prefix

        """Members
        """
        self.voice_and_head_dir = Twist()
        self.voice_and_head_dir.linear.x = 1  # It has to be different to 0 to inform the destination inference module that it is a valid command
        self.voice_and_head_vel = Twist()
        self.current_command = 'brake'  # Received vocal command
        self.state = 'turn'
        self.vel_inc = 0.15
        self.min_linear_vel = 0.15
        self.keywords_to_command = KEYWORDS_TO_COMMAND
        self.__listener = tf.TransformListener()  # NOTE THIS: the listener should be declared in the class
        """ Subscribers
        """
        self.voice_sub = rospy.Subscriber("recognizer/output", String , self.__voice_callback__)
        """ Publishers
        """
        self.voice_and_head_vel_pub = rospy.Publisher("voice_and_head_vel", Twist, queue_size=10)
        self.voice_and_head_dir_pub = rospy.Publisher("voice_and_head_dir", Twist, queue_size=10)
        self.dir_marker_pub = rospy.Publisher("voice_and_head_dir_marker", Marker, queue_size=10)
        self.text_marker_pub = rospy.Publisher('voice_and_head_text', Marker, queue_size=10)
        rospy.on_shutdown(self.cleanup)
        """    Main
        """
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                now = rospy.Time(0)
                (trans, rot) = self.__listener.lookupTransform('/new_ref', '/head_origin', rospy.Time(0))
            except (tf.Exception):
                rospy.logwarn('tf.Exception in voice_and_head.py cannot read tf from /new_ref to /head_origin')
                self.cleanup()
                rospy.sleep(0.2)
                continue
            # compute head angles
            roll, pitch, yaw = tf.transformations.euler_from_quaternion(rot)
            if (yaw > self.yaw_limit):
                yaw = 0.0
            if (yaw < -self.yaw_limit):
                yaw = 0.0

            if (pitch > self.max_pitch):
                pitch = 0.0
            if (pitch < self.min_pitch):
                pitch = 0.0

            if (roll > self.max_roll):
                roll = 0.0
            if (roll < self.min_roll):
                roll = 0.0
            #Put the head direction as a twist
            self.voice_and_head_dir.angular.y = pitch
            self.voice_and_head_dir.angular.x = roll
            #yaw +0.5*yaw is to compensate the error from the kinect when turning the face
            #yaw = yaw + 0.5*yaw
            self.voice_and_head_dir.angular.z = yaw
            self.voice_and_head_dir.linear.x=1.0

            if self.state == "brake":
                self.stop()
            else:
                # Compute the speed according to the head's direction
                if (math.fabs(yaw) <= self.zero_zone):
                    self.voice_and_head_vel.angular.z = 0.0
                else:
                    if yaw > 0.0:
                        self.voice_and_head_vel.angular.z = (self.max_theta_vel * (yaw - self.zero_zone))
                    else:
                        self.voice_and_head_vel.angular.z = (self.max_theta_vel * (yaw + self.zero_zone))
                #If the user just want to turn around set linear velocity to 0
                if self.state == "turn":
                    self.voice_and_head_vel.linear.x = 0
            # publish the speed and direction of the head as Twist messages
            self.voice_and_head_vel_pub.publish(self.voice_and_head_vel)  # vel_command should be continuosly published
            self.voice_and_head_dir_pub.publish(self.voice_and_head_dir)  # dir_command should be continuosly published

            # Printing a marker
            marker = self.fill_command_marker(self.voice_and_head_dir, self.tf_prefix)
            self.dir_marker_pub.publish(marker)
            r.sleep()

    def __voice_callback__(self, command):
        self.check_action_to_do(command.data)

    def check_action_to_do(self, str):
        """ Selects the selected behavior for each received voice command when the wheelchair is in manual mode
            str: String, the received command in text format.
        """
        self.current_command = deepcopy(str)
        if str == "brake":
            self.display_text("brake")
            if self.state == "turn" or self.state == "brake":
                self.state = "brake"
                self.stop()
            else:
                self.state = "turn"
                self.voice_and_head_vel.linear.x = 0

        elif str == "face":
            self.state = "turn"

        elif str == "go" :
            self.display_text("go")
            self.state = "moving"
            if self.voice_and_head_vel.linear.x <= 0.0:
                self.voice_and_head_vel.linear.x = self.min_linear_vel

        elif str == "back" :
            self.display_text("back")
            self.state = "moving"
            if self.voice_and_head_vel.linear.x >= 0.0:
                self.voice_and_head_vel.linear.x = -self.min_linear_vel

        elif str == "slower":
            self.display_text("slower")
            if self.state == "moving":
                if self.voice_and_head_vel.linear.x >= self.min_linear_vel:
                    self.voice_and_head_vel.linear.x -= self.vel_inc
                elif self.voice_and_head_vel.linear.x <= -self.min_linear_vel:
                    self.voice_and_head_vel.linear.x += self.vel_inc
                else:
                    self.voice_and_head_vel.linear.x = 0.0

        elif str == "faster":
            self.display_text("faster")
            if self.state == "moving":
                if self.voice_and_head_vel.linear.x >= 0:
                    self.voice_and_head_vel.linear.x += self.vel_inc
                else:
                    self.voice_and_head_vel.linear.x -= self.vel_inc

        elif str == "turn":
            self.display_text("turn")
            self.state = "turn"
            self.voice_and_head_vel.linear.x = 0

        else:
            rospy.logwarn("voice_and_head.py: Not a recognized command")


    def stop(self):
        """Stop the robot, It is different from cleanup because just affects the velocity
        topic
        """
        self.voice_and_head_vel = Twist()
        self.voice_and_head_vel_pub.publish(self.voice_and_head_vel)


    def cleanup(self):
        """ Put all publishers to zero
        """
        self.voice_and_head_vel = Twist()
        self.voice_and_head_dir = Twist()
        self.voice_and_head_vel_pub.publish(self.voice_and_head_vel)
        self.voice_and_head_dir_pub.publish(self.voice_and_head_dir)
        # Deleting the marker
        marker = self.fill_command_marker(self.voice_and_head_dir, self.tf_prefix)
        self.dir_marker_pub.publish(marker)

    def fill_command_marker(self, command, tf_prefix):
        """
        command: Twist(), I contains the angular and linear speed of the command that we want to plot in rviz
        tf_prefix: String, The namespace to be added to the name of the base_link, ex. robot_0 or wheelchair
        """
        marker = Marker()
        orientation = tf.transformations.quaternion_from_euler(0, 0, command.angular.z)
        marker.header.frame_id = tf_prefix + '/base_link'
        marker.header.stamp = rospy.Time.now()
        # For zero command, display a ball
        if command.angular.z == 0 and command.linear.x == 0:
            marker.type = Marker.SPHERE
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3
        # For non zero command we display arrows
        else:
            marker.type = Marker.ARROW
            marker.scale.x = 2.0
            marker.scale.y = 0.3
            marker.scale.z = 0.3
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.lifetime = rospy.Duration.from_sec(0.3)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        return marker

    def display_text(self, text):
        marker = self.fill_text_marker(text)
        self.text_marker_pub.publish(marker)


    def fill_text_marker(self, text):
        marker = Marker()
        marker.header.frame_id = self.tf_prefix + '/base_link'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = text
        marker.scale.z = 1.0
        marker.scale.y = 1.0
        marker.scale.x = 1.0
        marker.lifetime = rospy.Duration.from_sec(1.0)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        marker.pose.position.x = 1.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 1.0
        return marker

#===============================================================================
if __name__ == '__main__':
    """Initializing"""
    rospy.init_node('voice_and_head')
    try:
        VoiceAndHead()
    except:
        rospy.logfatal("voice_and_head.py  died")
        pass


