#! /usr/bin/env python
""" @package user_intentions
    
    This program is used to transform the amcl_poses into a path 
        @author: Jesus Arturo Escobedo Cabello
        @contact: jesus.escobedo-cabello@inria.fr
        @organization: INRIA Grenoble, Emotion-team
"""
import roslib
import sys
import rospy
import numpy as np
import tf
import yaml
import math
from move_base_msgs.msg import MoveBaseGoal
from move_base_msgs.msg import MoveBaseAction
from std_msgs.msg import String
from geometry_msgs.msg._PoseWithCovarianceStamped import PoseWithCovarianceStamped
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Pose import Pose
from copy import deepcopy 
from geometry_msgs.msg._Twist import Twist

import array
import actionlib
from rospy import exceptions
from copy import deepcopy 

from nav_msgs.msg._Path import Path
import rosbag


#==============================================================================
class PlotPath():
    def __init__(self):
        self.n = int
        self.n = 0
        """ GETTING PARAMS """
        self.pose = rospy.get_param('~pose', 'amcl_pose')
        self.path = rospy.get_param('~path', 'path')
        self.bag_file_name = rospy.get_param('~bag_file_name', '/home/arturo/rosbag_files/test_interfaces/arturo_voice_1.bag')
        """MEMBERS"""
        self.path = Path()
        self.stamped_pose = PoseStamped()
        """ SUBSCRIBERS """
        self.amcl_pose_subscriber = rospy.Subscriber(self.pose, PoseWithCovarianceStamped , self.amcl_pose_callback)
        """ PUBLISHERS """
        self.path_publisher = rospy.Publisher(self.path, Path, queue_size=1)
   
    """ In this callback we read the incoming pose and add it to the path, then we refresh
        the path 
    """
    def amcl_pose_callback(self, PoseWithCovarianceStamped):
        print 'received amcl_pose' 
        self.transform_to_stamped_pose(PoseWithCovarianceStamped)
        self.push_to_path(self.stamped_pose)
        self.path_publisher.publish(self.path)
    
    def print_bag(self):
        bag = rosbag.Bag(self.bag_file_name)
        for msg in bag.read_messages(topics=self.pose):  # the slash is necessary without it, the name can't be resolved.
            self.transform_to_stamped_pose(msg[1])
            self.push_to_path(self.stamped_pose)
            print msg
        bag.close()
        self.path_publisher.publish(self.path)
    
    def print_vector(self, vector):
        """
        publish a given vector as a ros path
        @param vector: numpy vector in the form [[x0,y0],[x1,y1],[x2,y2]....[xn,yn]] 
        """
        self.vector = vector
        n = 0
        for pose in vector:
            self.stamped_pose.pose.position.x = deepcopy(pose[0])
            self.stamped_pose.pose.position.y = deepcopy(pose[1])
            self.stamped_pose.pose.position.z = 0.2
            self.stamped_pose.header.frame_id = "map"
            self.stamped_pose.header.seq = n
            self.stamped_pose.header.stamp = rospy.Time.now()
            self.push_to_path(self.stamped_pose)
        self.path_publisher.publish(self.path)

            
    
    
    def transform_to_stamped_pose(self, PoseWithCovarianceStamped):
        print 'transforming from PoseWithCovarianceStamped to PoseStamped' 
        self.stamped_pose.pose = PoseWithCovarianceStamped.pose.pose
        self.stamped_pose.pose.position.z = 0.2
        self.stamped_pose.header = PoseWithCovarianceStamped.header
        
    
        
    def push_to_path(self, PoseStamped):  
        print 'pushing to path'
        self.path.header.seq = 1
        self.path.header.frame_id = PoseStamped.header.frame_id
        self.path.header.stamp = rospy.get_rostime()
        self.path.poses.append(deepcopy(PoseStamped))  # Deepcopy avoids pointing to the same element for all the path
        self.path.header.stamp = rospy.get_rostime()
        
            

#===============================================================================
if __name__ == '__main__':
    """ Init the node
    """
    rospy.init
    rospy.init_node('amcl_poses_to_path')
    rospy.loginfo("amcl_poses_to_path started")
    """ Setting ROS parameters
    """
    rospy.set_param('pose', '/amcl_pose')
    rospy.set_param('path', '/plot_path')
    rospy.set_param('bag_file_name', '/home/arturo/rosbag_files/test_interfaces/gregoire2.bag')
   
   # rospy.get_param('/bag_file_name', default='arturo_voice_2.bag')
   
    node = PlotPath()
    node.print_bag()
    rospy.spin()



