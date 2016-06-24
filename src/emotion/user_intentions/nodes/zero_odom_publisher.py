#! /usr/bin/env python
#
import roslib
import rospy
from geometry_msgs.msg._PoseWithCovarianceStamped import PoseWithCovarianceStamped
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Twist import Twist
from nav_msgs.msg._Odometry import Odometry
import tf
from geometry_msgs.msg._TransformStamped import TransformStamped
from copy import deepcopy
import numpy as np
#=============================================================================
#==============================================================================
class ZeroOdomPub():

    def __init__(self):
        """Publishes a zero odometry
        """
        print "init#####################################################"
        """ROS PARAMETERS"""
        self.tf_prefix = rospy.get_param('~tf_prefix', '')  # Reads the tf_prefix from the ROS namespace
        self.x = rospy.get_param('~x', 0)  # Reads the tf_prefix from the ROS namespace
        self.y = rospy.get_param('~y', 0)  # Reads the tf_prefix from the ROS namespace
        self.theta = rospy.get_param('~theta', 0)  # Reads the tf_prefix from the ROS namespace
        print self.tf_prefix
        if self.tf_prefix is not '':
            self.tf_prefix = '/' + self.tf_prefix
        print self.tf_prefix
        self.odom_frame_name = self.tf_prefix+'/odom'
        self.baselink_frame_name = self.tf_prefix + '/base_link'
        """MEMBERS"""
        self.odom = Odometry()
        """ROS PUBLISHERS"""
        self.odom_pub = rospy.Publisher(self.odom_frame_name, Odometry, queue_size=1)
        current_time = rospy.Time.now()
        last_time = rospy.Time.now()
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
#            print self.baselink_frame_name
#            print self.odom_frame_name
#            print self.tf_prefix
            r.sleep()
            last_time = deepcopy(current_time)
            current_time = rospy.Time.now()
            dt = (current_time.to_sec() - last_time.to_sec())

            #next, we'll publish the odometry message over ROS
            quat_tf = tf.transformations.quaternion_from_euler(0, 0, self.theta)
            self.odom.header.stamp = current_time
            self.odom.header.frame_id = self.odom_frame_name
            #set the position
            self.odom.pose.pose.position.x = self.x
            self.odom.pose.pose.position.y = self.y
            self.odom.pose.pose.position.z = 0
            self.odom.pose.pose.orientation.x = quat_tf[0]
            self.odom.pose.pose.orientation.y = quat_tf[1]
            self.odom.pose.pose.orientation.z = quat_tf[2]
            self.odom.pose.pose.orientation.w = quat_tf[3]
            #set the velocity
            self.odom.child_frame_id = self.baselink_frame_name
            self.odom.twist.twist.linear.x = 0
            self.odom.twist.twist.linear.y = 0
            self.odom.twist.twist.angular.z = 0
            #publish the message
            self.odom_pub.publish(self.odom)

#===============================================================================
if __name__ == '__main__':
    """Initializing ROS node"""
    rospy.init_node('zero_odom_publisher')
    try:
        ZeroOdomPub()
    except Exception as error:
        rospy.logfatal("zero_odom_publisher.py  died: %s", error)




