#!/usr/bin/env python
import roslib;
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from copy import deepcopy
from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray
import tf
import sys, select, termios, tty

class DriverCoyote:
    """This class performs the management of sensors and actuators of the robot
    """
    def __init__(self):
        """ The Constructor of the class
        """
        """ Constants
        """
        rospy.on_shutdown(self.cleanup)
        
        """Members
        """
        self.robot_vel = Twist()
        self.__listener = tf.TransformListener()  # NOTE THIS: the listener should be declared in the class
        self.timer = rospy.Time()  # To check if the head has sent the same velocity command for a while. It can be an error.
        self.timer_duration = rospy.Duration()
        
        """Publishers
        """
        self.odom_pub = rospy.Publisher('odometry', String, queue_size=10)
        self.odom = Odometry()
        r = rospy.Rate(10.0)
        
        """ Subscribers
        """
        rospy.Subscriber('arduino/lwheel', Twist, self.headCb)

        while not rospy.is_shutdown():
            print "I'm alive"
            r.sleep()


    def headCb(self, msg):
        self.filtered_head_pose = msg
        
    def save_speech(self, data, p):
        """ The recorded speech phrase @param data, is saved as a wav file.

        @param data: The data to be saved.
        @param p: The pyaudio member used to do the sound processing.
        """
        filename = 'output_' + str(int(time.time()))
        # write data to WAVE file
        self.data = ''.join(data)
        wf = wave.open(filename + '.wav', 'wb')
        wf.setnchannels(1)
        wf.setsampwidth(p.get_sample_size(pyaudio.paInt16))
        wf.setframerate(16000)
        wf.writeframes(self.data)
        wf.close()
        return filename

if __name__ == "__main__":
    rospy.init_node('driver_coyote')
    try:
        DriverCoyote()
    except:
        pass
