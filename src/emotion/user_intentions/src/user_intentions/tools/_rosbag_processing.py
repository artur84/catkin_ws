#! /usr/bin/env python
""" @package user_intentions
    
    This program gets data from recorded bagfiles 
        @author: Jesus Arturo Escobedo Cabello
        @contact: jesus.escobedo-cabello@inria.fr
        @organization: INRIA Grenoble, Emotion-team
"""
import roslib; roslib.load_manifest('user_intentions')

import sys
import rospy
import numpy as np
import tf
import yaml
import math
from copy import deepcopy
from matplotlib.pylab import *
import matplotlib.animation as animation
import array

from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
import actionlib
from std_msgs.msg import String
from geometry_msgs.msg._PoseWithCovarianceStamped import PoseWithCovarianceStamped
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Pose import Pose
from nav_msgs.msg._Odometry import Odometry
from geometry_msgs.msg._Twist import Twist
from nav_msgs.msg._Path import Path
import rosbag
import genpy
from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray

    
    

#==============================================================================
class ROSBagProc():
    def __init__(self):
        """ MEMBERS """
        self.n = int
        self.n = 0
        self.dir = "/home/arturo/bagfiles/face_sensor/"
        self.bag_cero_time = 1357932261  # This is because the time in some bagfiles doesn't start in cero, maybe we can one solution more elegant here
        self.start_time = genpy.Time(self.bag_cero_time + 100)
        self.end_time = genpy.Time(self.bag_cero_time + 300)
        
        self.load_bag("head_vs_odom_1.bag")  # The bagfile where the data comes from

    def load_bag(self, bag_name):
        print "Current dir is:  " + self.dir
        print "Opening:" + bag_name + "File..."
        try:
            self.bag = rosbag.Bag(self.dir + bag_name)  # The bagfile where the data comes from
            print "Bagfile successfully opened"
        except (ValueError, rosbag.ROSBagException, rosbag.ROSBagFormatException):
           print 'The rosbagfile could not be opened'

    def write_var_to_text(self, topic):
        """ Write the values of var from topic and returns the vector of values
            @param var: string, name of the var 'it should be fully resolved e.g pose.pose.position.x
            @param topic: string, name of the desired ros topic to plot. 
            Topic="/topic" The slash is necessary! Without it the name can't be resolved.
        """
        for msg in self.bag.read_messages(topics=topic):  # the slash is necessary without it, the name can't be resolved.
            # msg[0] is the name of the topic e.g. /wheelchair/odom, msg[1] the content"""
            ros_quat = vars()['msg[1].' + var_name]
            euler = tf.transformations.euler_from_quaternion([ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w])
            yaw.append(deepcopy(euler[2]))   
            r.append(1)
        return [r, yaw]


    def get_odom_yaw(self, topic):
        """ Gets the yaw angle from the odom topic
            @param topic: string, name of the desired ros topic to plot. 
        """
        odom_vec = []
        t = []
        for msg in self.bag.read_messages(topics=topic):  # the slash is necessary without it, the name can't be resolved.
            print "odom"        
            odom_vec.append(msg[1].twist.twist.angular.z) 
            t.append(genpy.Time.to_nsec(msg[2]))
        return (odom_vec, t)
            
        

    def get_pose_angles(self, topic):  
        """ Gets a pose  and prints it in a cartesian plane
            @param topic: string, name of the desired ros topic to plot. 
            Topic="/topic" The slash is necessary! Without it the name can't be resolved.
        """
        x = []
        y = []
        z = []
        t = []
        for msg in self.bag.read_messages(topics=topic, start_time=self.start_time, end_time=self.end_time):  # the slash is necessary without it, the name can't be resolved.
            # msg[0] is the name of the topic e.g. /wheelchair/odom. msg[1] is the content
            ros_quat = msg[1].pose.orientation 
            euler = tf.transformations.euler_from_quaternion([ros_quat.x, ros_quat.y, ros_quat.z, ros_quat.w])
            x.append(deepcopy(euler[0])) 
            y.append(deepcopy(euler[1])) 
            z.append(deepcopy(-euler[2])) 
            t.append(genpy.Time.to_nsec(msg[2]))
        return (x, y, z, t)

    
    def print_pose_with_covariance_in_map(self, topic):
        """ Prints all the message in the robagfile topic as arrow markers in the position and orientation
            @param topic: string, name of the desired ros topic to plot. 
            Topic="/topic" The slash is necessary! Without it the name can't be resolved.
            @param type: string, type of message in the topic, PoseWithCovarianceStamped
        """
        pose_vec = []
        msg = PoseWithCovarianceStamped()
        for msg in self.bag.read_messages(topics=topic, start_time=self.start_time, end_time=self.end_time): 
            # msg[0] is the name of the topic e.g. /wheelchair/odom.  msg[1] is the content
            pose_vec.append(msg[1])
        markers = self.fill_arrow_marker_array(pose_vec)
        self.pose_marker_pub.publish(markers)
            
    def print_odom_in_map(self, topic):
        """ Prints all the message in the robagfile topic as arrow markers in the position and orientation
            @param topic: string, name of the desired ros topic to plot. 
            Topic="/topic" The slash is necessary! Without it the name can't be resolved.
            @param type: string, type of message in the topic, PoseWithCovarianceStamped
        """
        
        for msg in self.bag.read_messages(topics=['/wheelchair/amcl_pose', '/wheelchair/odom'], start_time=self.start_time, end_time=self.end_time,):  # start_time=self.start_time, end_time=self.end_time # the slash is necessary without it, the name can't be resolved.
            # msg[0] is the name of the topic e.g. /wheelchair/odom, msg[1] the content
            print msg[0]
#            pose = self.transform_odom_to_pose(msg[1])
#            pose_vec.append(msg[1])
#        markers = self.fill_arrow_marker_array(pose_vec)
#        self.pose_marker_pub.publish(markers)  
    
    
    def transform_odom_to_pose(self, odom_msg):  
        """ Transforms an odometry message into a pose in the /head_origin frame so that we can easily print it as an axis in rviz
        """
        yaw = odom_msg.twist.twist.angular.z
        py_quat = tf.transformations.quaternion_from_euler(0, 0, yaw, 'sxyz')
        odom_pose.header.frame_id = "/head_origin"
        odom_pose.pose.pose.orientation.x = py_quat[0]
        odom_pose.pose.pose.orientation.y = py_quat[1]
        odom_pose.pose.pose.orientation.z = py_quat[2]
        odom_pose.pose.pose.orientation.w = py_quat[3]
        
            

        

        
        
    

        
            

#===============================================================================
if __name__ == '__main__':
    """ Init the node
    """
    node = ROSBagProc()
    (odom_yaw, t_odom) = node.get_odom_yaw("/wheelchair/odom")
    (x, y, z, t_head) = node.get_pose_angles("/head_pose_filtered")
    figure(1)
    plot(t_odom, odom_yaw, 'g')
    plot(t_head, z, 'r')
    grid(True)
    title('Head direction vs odom')
    ylabel('angle (rad)')
    xlabel('time (ns)')
    show()




