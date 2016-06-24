#! /usr/bin/env python
#
import roslib
from copy import deepcopy
import random
from user_intentions._Markers import UI_Markers
from xlrd.formula import num2strg

import actionlib
from geometry_msgs.msg._Quaternion import Quaternion
from geometry_msgs.msg._Twist import Twist
from nav_msgs.msg._Odometry import Odometry
import rospy
from std_msgs.msg import String
import tf
from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray

import numpy as np


#=============================================================================
#==============================================================================
class WheelchairMarkerPublisher():

    def __init__(self):
        """Class printing the marker of the wheelchair in rviz
        """
        """ MEMBERS """
        self.markers = UI_Markers()
        self.user_dir = Quaternion()  # The user commanded direction, it can come from keyboard = key_cmd, voice = voice_cmd, or head =  head_cmd
        self.user_vel = Twist()  # The user commanded direction, it can come from keyboard = key_cmd, voice = voice_cmd, or head =  head_cmd

        """ROS PARAMETERS"""
        self.tf_prefix = rospy.get_param('tf_prefix', '')  # # Reads the tf_prefix from the ROS namespace
        if self.tf_prefix is not '':
            self.tf_prefix = '/' + self.tf_prefix
        self.discrete_interface = rospy.get_param('~discrete_interface', False)  ## To enable/disable the discrete mode Set true when using the keyboard
        """ ROS SUBSCRIBERS """
        self.user_dir_sub = rospy.Subscriber("user_dir", Quaternion , self.__user_dir_callback__, None, 1)
        self.user_vel_sub = rospy.Subscriber("user_vel", Twist , self.__user_vel_callback__, None, 1)
        self.vocal_command_sub = rospy.Subscriber("recognizer/output", String , self.__voice_callback__, None, 1)

        """ROS PUBLISHERS"""
        self.wheelchair_marker_pub = rospy.Publisher("wheelchair_marker", Marker, queue_size=1)
        self.humans_marker_array_pub = rospy.Publisher("humans_marker_array", MarkerArray, queue_size=1)
        self.dir_marker_pub = rospy.Publisher("dir_marker", Marker, queue_size=1)
        self.vel_marker_pub = rospy.Publisher("vel_marker", Marker, queue_size=1)
        self.text_marker_pub = rospy.Publisher('text_marker', Marker, queue_size=1)

        """Start
        """
        r = rospy.Rate(10.0)

        while not rospy.is_shutdown():

            wheelchair_marker = self.fill_wheelchair_marker()

            humans = self.fill_humans_marker_array()
            self.wheelchair_marker_pub.publish(wheelchair_marker)
            self.wheelchair_marker_pub.publish(self.fill_sit_human_marker())  # I send it in the same topic it works well.
            self.humans_marker_array_pub.publish(humans)
            self.print_vel()


            r.sleep()

    def __user_dir_callback__(self, msg):
        self.user_dir = msg
        self.print_dir()

    def __user_vel_callback__(self, msg):
        self.user_vel = msg



    def __voice_callback__(self, msg):
        """ It will receive the vocal command from the user, we assume that it will be perfect (already filtered)
        """
        marker = self.markers.fill_text_marker(msg.data, 10.0, 1.0, 0.0, 0.0)
        self.text_marker_pub.publish(marker)

    def print_dir(self):

        if self.discrete_interface:
            marker = self.markers.fill_dir_marker(self.user_dir, 1)
        else:
            marker = self.markers.fill_dir_marker(self.user_dir, 0)
        self.dir_marker_pub.publish(marker)

    def print_vel(self):
        marker = self.markers.fill_vel_marker(self.user_dir, self.user_vel)  # we suppose that dir and vel will have the same direction
        self.vel_marker_pub.publish(marker)

    def fill_people_marker(self, marker_id):
        """Fill a marker of a woman
            marker_id the number of the human
        """
        marker = Marker()
        frame = "/robot_" + num2strg(marker_id)
        marker.id = marker_id
        marker.header.frame_id = frame + '/base_link'
#        marker.header.frame_id = '/robot_0/base_link'
        marker.header.stamp = rospy.Time.now()

        marker.type = Marker.MESH_RESOURCE

        if marker_id % 2 == 0:
            marker.mesh_resource = "package://social_filter/meshes/man2/models/man1.dae"
            marker.scale.x = 40
            marker.scale.y = 35
            marker.scale.z = 50
            marker.pose.position.y += 0.04
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 1.82)
        else:
            marker.mesh_resource = "package://social_filter/meshes/female1/models/female1.dae"
            marker.scale.x = 0.45
            marker.scale.y = 0.35
            marker.scale.z = 0.5
            quaternion = tf.transformations.quaternion_from_euler(0, 0, 1.57)
        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]


        marker.mesh_use_embedded_materials = True;


        marker.lifetime = rospy.Duration.from_sec(1.0)
        return marker

    def fill_wheelchair_marker(self):
        marker = Marker()
        marker.header.frame_id = self.tf_prefix + '/base_link'
#        marker.header.frame_id = '/robot_0/base_link'
        marker.header.stamp = rospy.Time.now()
        marker.id = 1

        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://social_filter/meshes/wheelchair/Wheelchair1.dae";

        marker.mesh_use_embedded_materials = True;
        marker.pose.position.x = -0.2
        marker.scale.x = 1.7
        marker.scale.y = 1.1
        marker.scale.z = 1.2
        marker.lifetime = rospy.Duration.from_sec(1.0)
        return marker


    def fill_sit_human_marker(self):
        marker = Marker()
        marker.header.frame_id = self.tf_prefix + '/base_link'
        marker.header.stamp = rospy.Time.now()
        marker.id = 2

        marker.type = Marker.MESH_RESOURCE
        marker.mesh_resource = "package://social_filter/meshes/human/3D_Man_Seated.dae";

        marker.mesh_use_embedded_materials = True;

        marker.scale.x = .025;
        marker.scale.y = .025;
        marker.scale.z = .025;
        marker.pose.position.x -= 0.2;
        marker.pose.position.z += .3;
        marker.lifetime = rospy.Duration.from_sec(1.0)
        return marker

    def fill_humans_marker_array(self):
        marker_array = MarkerArray()
        marker = Marker()
        for n in range(1, 10):  # We will display maximum 10 markers
            marker = self.fill_people_marker(n)
            marker_array.markers.append(deepcopy(marker))
        return marker_array

#===============================================================================
if __name__ == '__main__':
    """Initializing ROS node"""
    rospy.init_node('wheelchair_marker_publisher')
    try:
        WheelchairMarkerPublisher()
    except:
        rospy.logfatal("WheelchairMarkerPublisher.py  died")
        pass



