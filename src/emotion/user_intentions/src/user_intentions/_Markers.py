#! /usr/bin/env python
""" @package ui

    This code is used to plot the necesary markers as, goals,
    probabilities etc.
        @author: Jesus Arturo Escobedo Cabello
        @contact: jesus.escobedo-cabello@inria.fr
        @organization: INRIA Grenoble, Emotion-team
"""
from copy import deepcopy

from geometry_msgs.msg._Quaternion import Quaternion
import rospy
import tf
from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray

import numpy as np


class UI_Markers():
    """
    Class to display user_intentions markers
    """


    def __init__(self):
        """ ROS Parameters
        """
        self.tf_prefix = rospy.get_param('tf_prefix', '')
        if self.tf_prefix is not '':
            self.tf_prefix = '/' + self.tf_prefix
        self.vel_marker_id = 0  # An int to keep a marker id for velocity
        self.dir_marker_id = 0  # An int to keep a marker id for dir
        self.text_marker_id = 0  # An int to keep a marker id for text

    def fill_goal_marker_array(self, goalPoseArray, proba_values, tags, lifetime=0.5):
        marker_array = MarkerArray()
        sphere_marker = Marker()
        text_marker = Marker()
        tag_marker = Marker()
        count = 0
        for goal in goalPoseArray:
            text_marker = self.fill_goal_id_text(count, goal, lifetime=lifetime)
            sphere_marker = self.fill_goal_marker(count, goal, proba_values[count], lifetime=lifetime)
            tag_marker = self.fill_goal_tag_text(count, goal, tags[count], lifetime=lifetime)
            marker_array.markers.append(deepcopy(sphere_marker))
            marker_array.markers.append(deepcopy(text_marker))
            marker_array.markers.append(deepcopy(tag_marker))
            count += 1
        return marker_array


    def fill_goal_id_text(self, marker_id, goal, text="i=", lifetime=0.5):
        marker = Marker()
        marker.ns = "index"
        marker.lifetime = rospy.Duration.from_sec(lifetime)
        marker.id = marker_id
        marker.header.frame_id = self.tf_prefix + '/map'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = text + str(marker_id + 1)  # index should start in one
        marker.pose.position.x = goal.pose.position.x
        marker.pose.position.y = goal.pose.position.y
        marker.pose.position.z = 1.0
        marker.scale.x = 0.6
        marker.scale.y = 0.6
        marker.scale.z = 0.6
        marker.color.a = 0.9
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        return marker

    def fill_goal_tag_text(self, marker_id, goal, tag_string="tag", lifetime=0.5):
        marker = Marker()
        marker.ns = "tag"
        marker.lifetime = rospy.Duration.from_sec(lifetime)
        marker.id = marker_id
        marker.header.frame_id = self.tf_prefix + '/map'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = tag_string
        marker.pose.position.x = goal.pose.position.x
        marker.pose.position.y = goal.pose.position.y + 0.6
        marker.pose.position.z = 2.0
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        marker.color.a = 0.9
        marker.color.r = 0.0
        marker.color.g = 0.3
        marker.color.b = 0.0
        return marker

    def fill_goal_marker(self, marker_id, goal, value, lifetime=0.5):
        marker = Marker()
        marker.ns = "sphere"
        marker.lifetime = rospy.Duration.from_sec(lifetime)
        marker.id = marker_id
        marker.header.frame_id = self.tf_prefix + '/map'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.SPHERE
        marker.text = str(round(value, 4))
        marker.pose = goal.pose
        marker.pose.position.z = 1.5 * value + 0.1
        marker.scale.x = 3.0 * value + 0.1
        marker.scale.y = 3.0 * value + 0.1
        marker.scale.z = 3.0 * value + 0.1
        marker.color.a = 0.5
        marker.color.r = value * 6
        marker.color.g = 0.0
        marker.color.b = 1 - value * 6
        return marker

    def fill_text_marker(self, text, lifetime=1.0, r=0.0, g=0.0, b=1.0):
        marker = Marker()

        marker.id = self.text_marker_id
        #self.text_marker_id += 1

        marker.header.frame_id = self.tf_prefix + '/base_link'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = text
        marker.scale.z = 1.0
        marker.lifetime = rospy.Duration.from_sec(lifetime)
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 1.0
        marker.pose.position.z = 2.0
        return marker


    def fill_vel_marker(self, orientation, vel):
        """ This function is different to fill_dir_marker because the size of the arrow changes accoring to
        the linear vel value
        vel: Twist(), It contains the angular and linear direction
        orientation: Quaternion()
        """
        marker = Marker()
        marker.header.frame_id = self.tf_prefix + '/base_link'
        marker.header.stamp = rospy.Time.now()
        marker.id = self.vel_marker_id
        # For zero command, display a ball
        if [orientation.x, orientation.y, orientation.z, orientation.w] == [0, 0, 0, 0]:
            marker.type = Marker.SPHERE
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 0.1
        # For non zero dir we display arrows
        else:
            marker.type = Marker.ARROW
            marker.scale.x = 1.5 * np.abs(vel.linear.x) + 0.4
            marker.scale.y = 0.2
            marker.scale.z = 0.01
            marker.color.a = 0.1
            marker.pose.orientation = orientation
        marker.lifetime = rospy.Duration.from_sec(5.0)
        marker.color.r = 0.0
        marker.color.g = 0.0
        marker.color.b = 1.0

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.8
        self.vel_marker_id += 1
        return marker

    def fill_dir_marker(self, orientation, discrete_mode_flag):
        """ This function is different to fill_vel_marker in which the size of the arrow is constant
        @param orientation: ROS Quaternion(), I contains the angular and linear direction
        @param discrete_mode_flag: bool, To change the markers to specifically show in discrete mode and continuous mode.
        """
        marker = Marker()
        marker.id = self.dir_marker_id
        marker.header.frame_id = self.tf_prefix + '/base_link'
        marker.header.stamp = rospy.Time.now()
        #In continuous mode we don't like to print the markers very strong.
        if discrete_mode_flag:
            marker.color.a = 0.8
            number_of_arrows_to_show = 1
        else:
            marker.color.a = 0.1
            number_of_arrows_to_show = 30
        # For zero command, display a ball
        if [orientation.x, orientation.y, orientation.z, orientation.w] == [0, 0, 0, 0]:
            marker.type = Marker.SPHERE
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.3

        # For non zero command we display arrows
        else:
            marker.type = Marker.ARROW
            marker.scale.x = 0.9
            marker.scale.y = 0.15
            marker.scale.z = 0.15

            marker.pose.orientation = orientation
        marker.lifetime = rospy.Duration.from_sec(60.0)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 1.7
        self.dir_marker_id += 1
        if self.dir_marker_id == number_of_arrows_to_show:  #To show up to this number of arrows
            self.dir_marker_id = 0
        return marker


    def fill_joy_marker(self, joy_input):
        """
        joy_input: type ROS   Odometry()
        """
        marker = Marker()
        orientation = tf.transformations.quaternion_from_euler(0, 0, joy_input.twist.twist.angular.z)
        marker.header.frame_id = joy_input.header.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.ARROW
        marker.pose.orientation.x = orientation[0]
        marker.pose.orientation.y = orientation[1]
        marker.pose.orientation.z = orientation[2]
        marker.pose.orientation.w = orientation[3]
        marker.scale.x = 2.0 * joy_input.twist.twist.linear.x
        marker.scale.y = 2.0
        marker.scale.z = 2.0
        marker.lifetime.from_sec(10.0)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        return marker

