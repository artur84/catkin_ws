#! /usr/bin/env python
import roslib
import rospy
from sensor_msgs.msg import LaserScan
import yaml

class handler1():

    def __init__(self, client):
        self.client = client
        self.key_command = int
        self.n = int
        self.n = 0
        
        
        #Initial value of data
        self.laser_data = LaserScan()


        #Setting subscriber to necessary topics
        self.laserSubscriber = rospy.Subscriber("/base_scan", LaserScan , self.laser_callback)
        self.f = open('/home/arturo/test/laser', 'w')

    def laser_callback(self, LaserScan):
        #Gets pos and orientation data from the message
        #self.robot_pos = PoseWithCovarianceStamped.pose.pose.position
        #self.orient = PoseWithCovarianceStamped.pose.pose.orientation

        #Transforms the ...pose.orientation in a python list which is the format
        #required as input by the "euler_from_quaternion" function
        #self.quat = [self.orient.x, self.orient.y, self.orient.z, self.orient.w]

        #Gets the robot orientation as euler angles
        #self.robot_orient_euler = tf.transformations.euler_from_quaternion(self.quat, 'sxyz')
        #self.robot_theta = self.robot_orient_euler[2] #theta goes from -pi to pi (rad)
        #print 'theta:  ' + str(self.robot_theta)

        self.laser_data = LaserScan
        print self.laser_data
    
    

        





#===============================================================================
if __name__ == '__main__':
    #Init Node
    rospy.init
    rospy.init_node('laser_saver')

    #Transforms the ...pose.orientation in a python list which is the format
    #required as input by the "euler_from_quaternion" function
    

        #Gets the robot orientation as euler angles
        #self.robot_orient_euler = tf.transformations.euler_from_quaternion(self.quat, 'sxyz')
        #self.robot_theta = self.robot_orient_euler[2] #theta goes from -pi to pi (rad)
        #print 'theta:  ' + str(self.robot_theta)
    handler1()
    rospy.spin()
        
    
   


    




