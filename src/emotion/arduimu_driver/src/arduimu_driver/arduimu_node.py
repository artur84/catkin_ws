#! /usr/bin/env python
import roslib; 
import rospy
import math
import numpy
import tf
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_about_axis
   
class ArduIMUNode():
    
    def __init__(self, port='/dev/ttyUSB0'):
        """This class reads the serial port where the Arduimu board is connected, and puts the result in a ros message.
        
        I wrote this code considering only the arduimu V3, For this driver to work you should make sure that the arduimu is 
        publishing the data as strings and no as a binary number, In the Arduimu.ino file verify the following code.
                // *** NOTE!   To use ArduIMU with ArduPilot you must select binary output messages (change to 1 here)
                #define PRINT_BINARY 0  //Will print binary message and suppress ASCII messages (above)
        @param port: string, name of the port where arduimu is connected. ex. '/dev/ttyUSB0'
        
        """
        """Initializing ROS node"""
        rospy.init_node('arduimu_node')
        """ROS PARAMETERS"""
        self.port = rospy.get_param('~port', port)
        rospy.loginfo("serial port: %s" % (self.port))
        """MEMBERS"""
        self.pitch = 0
        self.yaw = 0
        self.roll = 0
        self.imu_data = Imu()
        self.imu_data = Imu(header=rospy.Header(frame_id="imu_link")) 
        """ Sets covariance matrix """
        self.imu_data.orientation_covariance = [1e6, 0, 0,
                                                0, 1e6, 0,
                                                0, 0, 1e-6]
        self.imu_data.angular_velocity_covariance = [1e6, 0, 0,
                                                     0, 1e6, 0,
                                                     0, 0, 1e-6]
        self.imu_data.linear_acceleration_covariance = [1e6, 0, 0,
                                                        0, 1e6, 0,
                                                        0, 0, 1e-6]
        self.imu_pub = rospy.Publisher('imu/data', Imu, queue_size=10)
        """Open the port to read the data"""
        try:
            self.serial_port = open(self.port)  
            print "arduimu connection initialized"
        except:
            print "could not connect to arduimu"
            self.__close__()
        r = rospy.Rate(30.0)
        while not rospy.is_shutdown():
            received_string = ''
            listc = []
            count = 0
            correct_elements = 0  # To count up how many correct data we received
            """Read the serial port till it finds the end of the string marked with three stars (***)"""
            for char in self.serial_port.read():
                    listc += [char]  # Gets the input from the serial port as a list of chars
                    if char == '*':  # Arduimu sends three stars  (***) to mark the end of a line 
                        count += 1
                    if count == 3:
                        count = 0
                        received_string = ''.join(listc)  # Join all the chars in a single string 
            tuples_from_serial = received_string.split(',')  # Splits the message in strings separated by ','
            members = [ tuple.split(':') for tuple in tuples_from_serial]  # Then each of the previously separated messages is separated again when a ':' is found in order to get each member.
            """Decodes the information contained in string of the form '!!!VER:1.9:RLL:-0.456:PCH:-0.65....***"""
            for element in members:
                if element[0] == 'RLL':             
                    self.roll = float(element[1])
                    correct_elements += 1
                elif element[0] == 'PCH':             
                    self.pitch = float(element[1])
                    correct_elements += 1
                elif element[0] == 'YAW':
                    self.yaw = float(element[1])
                    correct_elements += 1
                else:
                    not_found = True
            """Check if we received the 3 angles roll,pitch and yaw correctly"""
            if correct_elements == 3: 
                self.publish_ros_data(self.roll, self.pitch, self.yaw)
            r.sleep()
                    
    def publish_ros_data(self, roll, pitch, yaw):
        self.imu_data.header.seq += 1
        self.imu_data.header.stamp = rospy.Time.now()
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw, 'sxyz')
        """Puts the computed quaternion in the ROS message"""
        self.imu_data.orientation.x = q[0] 
        self.imu_data.orientation.y = q[1]
        self.imu_data.orientation.z = q[2]
        self.imu_data.orientation.w = q[3] 
        """This should be changed to read the angular velocity"""
        self.imu_data.angular_velocity.x = 0
        self.imu_data.angular_velocity.y = 0
        self.imu_data.angular_velocity.z = 0
        """This should be changed to read the acceleration"""
        self.imu_data.linear_acceleration.x = 0
        self.imu_data.linear_acceleration.y = 0
        self.imu_data.linear_acceleration.z = 0
        """Publish the data to a ros topic"""
        print self.imu_data
        self.imu_pub.publish(self.imu_data)
    
    def __close__(self):
        self.serial_port.fclose()
        print "closed"

if __name__ == '__main__':
    try:
        ArduIMUNode()
    except:
        rospy.logfatal("arduimu_node.py  died")
        pass
