#! /usr/bin/env python
#
import sys
import rospy
import numpy as np
import yaml
import math
from copy import deepcopy
from geometry_msgs.msg import PoseStamped
import tf

#==============================================================================
#==============================================================================
class GoalsFileCreator:
    def __init__(self):
        """Class for computing of current intended destination of a user driving the
            wheelchair.
        """
        """ROS PARAMETERS
        """
        self.tf_prefix = rospy.get_param('tf_prefix', '')  ## Reads the tf_prefix from the ROS namespace
        if self.tf_prefix is not '':
            self.tf_prefix = '/' + self.tf_prefix
        rospy.loginfo('UI: tf_prefix: %s', self.tf_prefix)
        rospy.on_shutdown(self.ending)
        self.goals_file = rospy.get_param('~BN_goals_file', '/home/artur/Documents/videos_wheelchair/gerhome_dataset_videos/2008_03_27/test.yaml')
        self.pose_vector = []
        self.data_dictionary = {'p_g_xc0':[]}#'tags':["stair", "corridor"], 'goals:[[x,y,theta],[x,y,theta]]
        """MEMBERS
        """
       
        """ROS SUBSCRIBERS"""
        self.user_dir_sub = rospy.Subscriber("/robot_0/move_base_simple/goal", PoseStamped , self.__goal_callback__, None, 1)
        r = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            
            r.sleep()

    def read_data(self, path):
        """ Writes the data of typical destinations in the selected scenario
        """
        try:
            rospy.loginfo("Trying to read File: %s", path)
            with open(path) as opened_file:
                self.data = yaml.load(opened_file)
                rospy.loginfo("File loaded from: %s", path)
        except IOError as exc:
            rospy.logerr("The file CAN'T BE READ: %s", path)
        self.static_goals_X_Y_THETA_array = np.asfarray(self.data['goals'])#The matrix containing [x,y,theta] for each goal
        self.goals_X_Y_THETA_array = deepcopy(self.static_goals_X_Y_THETA_array)
        self.p_g_xc0 = np.asfarray(self.data['p_g_xc0'])#Prior probability
        self.static_goals_prior = deepcopy(self.p_g_xc0) 
        self.static_tag_list = self.data['tags']#The tag for each goal
        self.p_desired_measured = np.asfarray(self.data['p_desired_measured'])
        self.number_of_static_goals = deepcopy(np.shape(self.static_goals_X_Y_THETA_array)[0])
        #print self.p_desired_measured
        opened_file.close()
        
    def write_data(self, path, data):
        """ Writes the data of typical destinations in the selected scenario
        """
        print data
        print path
        try:
            rospy.loginfo("Trying write data to File: %s", path)
            with open(path,"w+") as opened_file:
                yaml.safe_dump(data, opened_file)
                rospy.loginfo("Data written to %s", path)
        except IOError as exc:
            rospy.logerr("The file CAN'T BE ACCESSED: %s", path)
        opened_file.close()
        
    def __goal_callback__(self, msg):
        orientation = [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w]
        euler = tf.transformations.euler_from_quaternion(orientation)
        pose = [msg.pose.position.x, msg.pose.position.y, euler[2]]
        self.pose_vector.append(pose)
        print msg
        
    def ending(self):
        print "ending"
        #Create probability matrix with size according to the number of goals
        self.write_data(self.goals_file, self.pose_vector)
#===============================================================================
if __name__ == '__main__':
    """Initializing ROS node"""
    rospy.init_node('goals_file_creator')
    try:
        GoalsFileCreator()
    except:
        rospy.logfatal("goals_file_creator.py  died")
        pass
