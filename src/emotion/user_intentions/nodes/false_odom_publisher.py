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
class FalseOdomPub():

    def __init__(self):
        """Takes cmd_vel topic and position of the wheelchair and resend it as a false odometry topic
        """
        print "init"
        """ROS PARAMETERS"""
        self.tf_prefix = rospy.get_param('~tf_prefix', '')  # Reads the tf_prefix from the ROS namespace
        print self.tf_prefix
        if self.tf_prefix is not '':
            self.tf_prefix = '/' + self.tf_prefix
        print self.tf_prefix
        self.odom_frame_name = self.tf_prefix+'/odom'
        self.baselink_frame_name = self.tf_prefix+'/base_link'
        """MEMBERS"""
        self.x = 0
        self.y = 0
        self.theta = 0
        self.prev_vx =0
        self.prev_vtheta =0
        self.alpha =0.8
        self.beta = 0.9
        self.count =10
        self.vx = 0
        self.vy = 0
        self.vtheta = 0
        self.cmd_vel = Twist()  # The current velocity of the wheelchair,
        self.odom = Odometry()
        """ROS SUBSCRIBERS"""
        self.cmd_vel_sub = rospy.Subscriber("cmd_vel", Twist, self.__cmd_vel_callback__, None, 1)
        """ROS PUBLISHERS"""
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        """tf"""
        tf_broadcaster = tf.TransformBroadcaster()
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
            delta_x = (self.vx * np.cos(self.theta)-self.vy*np.sin(self.theta))*dt
            delta_y = (self.vx * np.sin(self.theta)+self.vy*np.cos(self.theta))*dt
            delta_theta = self.vtheta * dt
            self.x = self.x+delta_x
            self.y = self.y+delta_y
            self.theta = self.theta + delta_theta
            #first, we'll publish the transform over tf
            quat_tf = tf.transformations.quaternion_from_euler(0, 0, self.theta)
            tf_broadcaster.sendTransform((self.x, self.y, 0), quat_tf,
                                         current_time,self.baselink_frame_name, self.odom_frame_name)
            #next, we'll publish the odometry message over ROS
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
            self.odom.twist.twist.linear.x = self.vx
            self.odom.twist.twist.linear.y = self.vy
            self.odom.twist.twist.angular.z = self.vtheta
            #publish the message
            self.odom_pub.publish(self.odom)



    def __cmd_vel_callback__(self, cmd_vel):

        self.vx = self.alpha * self.prev_vx + (1-self.alpha)* cmd_vel.linear.x
        self.vtheta = self.beta * self.prev_vtheta + (1-self.beta) * cmd_vel.angular.z
        self.vy = 0
        self.prev_vx =self.vx
        self.prev_vtheta =self.vtheta



#===============================================================================
if __name__ == '__main__':
    """Initializing ROS node"""
    rospy.init_node('false_odom_publisher')
    try:
        FalseOdomPub()
    except:
        rospy.logfatal("false_odom_publisher.py  died")
        pass



