#!/usr/bin/env python
"""
servo_following.py
 This program will read the position of a user given by openni_tracker
 then it will move the kinect by using the servo motor connected to an arduino board
 It relies on openni_tracker and serial_node.py nodes, so be sure you 
 launch them first.
maintainer: arturoescobedo.iq@gmail.com
"""
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import UInt16
from copy import deepcopy 
import numpy as np

class TrackedPerson():
    def __init__( self ):
        """ Constants
        """
        
        """Members 
        """ 
        self.last_angle_from_kinect = 0
        self.current_angle_from_kinect = 1
        """ Publishers
        """
        
        """ Subscribers
        """
        self.__listener = tf.TransformListener() #NOTE THIS: the listener should be declared in the class

    def get_angle_from_kinect(self):
        """ Get the position (angle) of the tracked user with respect to the kinect """
        try:
            now = rospy.Time(0)
            #wait transformation for 500 msec = 500 000 000 nsec )
            #It caused the node to die
            #self.__listener.waitForTransform( '/new_ref', '/head_origin', now, rospy.Duration( 1, 500000000 )) 
            (trans, rot) = self.__listener.lookupTransform( 'openni_depth_frame', 'torso_1',  now)
        except ( tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException ):
            pass
        self.last_angle_from_kinect = self.current_angle_from_kinect
        self.current_angle_from_kinect = np.rad2deg(np.arctan2( trans[1] , trans[0] )) #Angle of the user with respect to the kinect
        return self.current_angle_from_kinect
    
    def loop_exists(self):
        if self.current_angle_from_kinect == self.last_angle_from_kinect:
            return True
        else:
            return False
        
        



class Servo():
    def __init__( self ):
        """ Constants
        """
        self.C_TIME_FOR_ANGLE_INCREMENT = 0.15 #time in seconds
        self.C_ANGLE_INCREMENT= 3 #Value of desired angle steps to move the servo (It is related to the resolution of the servo) mine is 10 degrees.
        self.C_INIT_ANGLE = 90 #Servo starting position 90 -> to the front, 0->right, 180 left.
        """For PID controller """
        self.dt = 0.1 #Period of time to change the min resolution angle of the servo
        self.previous_angle_error = 0.0
        self.integral = 0
        self.derivative = 0
        self.Kp = 0.13  #Best tuned params I have found are kp 0.13, ki 0.17, and kd 0.025
        self.Ki = 0.17
        self.Kd = 0.025
        """Members 
        """ 
        self.angle = UInt16() #Values 0 "right"  - 180 "left", The angle where we want the serve moves to 
        """Publishers
        """
        self.angle_pub = rospy.Publisher( '/servo', UInt16, queue_size=10 ) #Should have values between 0 and 180
        self.servo_tf_br = tf.TransformBroadcaster()
        """Subscribers
        """
        
    def move_angle(self, angle):
#        if (angle > self.C_ANGLE_INCREMENT):
#                self.move_left()
#        if (angle < -self.C_ANGLE_INCREMENT):
#                self.move_right()
#        rospy.sleep(1)
        if (np.abs(angle) > self.C_ANGLE_INCREMENT):
            next_angle = self.angle + self.C_ANGLE_INCREMENT
            if next_angle >= 0 and next_angle <= 180:
#                if angle > 0:
#                    self.move_left()
#                else:
#                    self.move_right()
#
#                rospy.sleep(rospy.Duration.from_sec(self.C_TIME_FOR_ANGLE_INCREMENT))
                self.pid_control(angle)
            else:
                rospy.loginfo("Servo can't move more")
    
    def pid_control(self, angle_error):
        self.angle_error = angle_error
        rospy.sleep(rospy.Duration.from_sec(self.dt))
        self.integral = self.integral + (self.angle_error*self.dt)
        self.derivative = (self.angle_error - self.previous_angle_error)/self.dt
        self.pid_output = (self.Kp*self.angle_error) + (self.Ki*self.integral) + (self.Kd*self.derivative)
        self.previous_angle_error = self.angle_error
        self.angle += self.pid_output
        self.angle_pub.publish(self.angle)
        #TODO: remap this output to 0-180 scale
        
        
        
    def move_right(self):
            self.angle -= self.C_ANGLE_INCREMENT
            self.angle_pub.publish(self.angle)

    
    def move_left(self):   
            self.angle += self.C_ANGLE_INCREMENT
            self.angle_pub.publish(self.angle)


    def go_home(self):
        self.angle_pub.publish(self.C_INIT_ANGLE)
        self.angle = self.C_INIT_ANGLE
        
        
class ServoFollowing():
    def __init__( self ):
        """ Constants
        """
        
        """Members 
        """ 
        self.servo = Servo()
        self.user = TrackedPerson()
        """ Publishers
        """
 
        rospy.on_shutdown( self.cleanup )
        self.cleanup()
        r = rospy.Rate( 10.0 )
        
        while not rospy.is_shutdown():
            try:
                angle = self.user.get_angle_from_kinect()  
                print "Angle between tracked person and kinect:"                
                print angle
            except:
                print "Cannot find the tracked person"
                rospy.sleep(1)
                continue
            if self.user.loop_exists():
                print "Loop Exists"
                rospy.sleep(1)
                continue
            self.servo.move_angle(angle)
            r.sleep()
            
    def cleanup(self):
        self.servo.go_home()
    
        
    

if __name__ == "__main__":
    rospy.init_node( 'servo_following' )
    try:
        ServoFollowing()
    except:
        rospy.logfatal("servo_following.py controller died")
        pass
