#! /usr/bin/env python
#
import roslib
from copy import deepcopy
from user_intentions._Markers import UI_Markers
from user_intentions._conversions import pose_quat_to_euler, pose_euler_to_quat

import actionlib
from controller.srv._EnableControl import EnableControl
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._PoseWithCovarianceStamped import PoseWithCovarianceStamped
from geometry_msgs.msg._Twist import Twist
from move_base_msgs.msg._MoveBaseActionResult import MoveBaseActionResult
import rospy
from sound_play.libsoundplay import SoundClient
from std_msgs.msg import String
from std_msgs.msg._Int32 import Int32
from std_srvs.srv._Empty import Empty
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray

import numpy as np


#from std_msgs.msg._Float32 import Float32
#=============================================================================
#==============================================================================
class ModeSelector():
    """
    This node  connects to different cmd_vel topics published by different controllers
    for example: move_base and head controller then selects one of them to send it to the wheelchair.
    The selection is made according to the voice command given by the user.
    """
    def __init__(self):
        """
        Constructor.
        @param
        @type
        """
        rospy.on_shutdown(self.cleanup)

        """ ROS parameters
        """
        self.mode = rospy.get_param('~mode' , 'in_goal')  ## The current mode of the wheelchair,
                                                            # It can be "autonomous, follow, face, in_goal, joystick
        self.last_mode = deepcopy(self.mode)
        rospy.loginfo("mode param set to %s", self.mode)
        self.tf_prefix = rospy.get_param('tf_prefix', '')  # The tf_prefix from the ROS namespace
        if self.tf_prefix is not '':
            self.tf_prefix = '/' + self.tf_prefix

        """ Members
        """
        #self.max_proba_value = 0.0
        self.markers = UI_Markers()
        self.user_vel = Twist()
        self.robot_vel = Twist()
        self.follower_vel = Twist()
        self.cmd_vel = Twist()
        #self.head_pose = PoseStamped()
        self.pitch_refference = 0.4  # Todo: Adjust this value according to typicall values of the kinect head pose estimator when the head is looking to the front

        """ Subscribers
        """
        #This topic sends a message if the move_base goal was reached or cancelled
        self.move_base_result_sub = rospy.Subscriber("move_base/result", MoveBaseActionResult, self.__move_base_result_callback__)
        #self.head_pose_sub = rospy.Subscriber("user_dir", Twist, self.__head_pose_callback__)
        self.voice_sub = rospy.Subscriber("recognizer/output", String , self.__voice_callback__)
        self.user_vel_sub = rospy.Subscriber("user_vel", Twist , self.__user_vel_callback__)
        self.robot_vel_sub = rospy.Subscriber("robot_vel", Twist , self.__robot_vel_callback__)  # velocity comming from move base
        self.follower_vel_sub = rospy.Subscriber("follower_vel", Twist , self.__follower_vel_callback__)  # velocity comming from move base
        self.riskrrt_finished_sub = rospy.Subscriber("rosplanner_static/status_rrt", Int32, self.__riskrrt_finished_callback__, None, 1)
        #self.follower_vel_sub = rospy.Subscriber("max_proba_value", Float32 , self.__max_proba_value_callback__)
        """ Publishers
        """
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)
        self.text_marker_pub = rospy.Publisher('mode_selector_text', Marker, queue_size=1)
        # Init the wheelchair in the required mode Manual/Autonomous
        self.display_text("Autonomous mode")
        self.wheelchair_autonomous_mode_enable()
        r = rospy.Rate(30)

        """ Main loop
        """
        while not rospy.is_shutdown():
            r.sleep()

    def wheelchair_manual_mode_enable(self):
        """ Puts the wheelchair in joystick mode so that it can be controlled using the
        Joystick
        """
        rospy.loginfo('mode_selector: calling bb_robot/manual_enable service ')
        # rospy.wait_for_service('voice_and_head/disable')
        try:
            self.wheelchair_manual_mode_enable_client = rospy.ServiceProxy('/wheelchair/bb_robot/manual_enable', Empty)
            self.wheelchair_manual_mode_enable_client()
        except rospy.ServiceException, e:
            rospy.logwarn("mode_selector: Service call to bb_robot/manual_enable failed (this is probably ok if you are not using this node) : %s", e)


    def wheelchair_autonomous_mode_enable(self):
        """ Puts the wheelchair in autonomous mode so that it can be controlled by move_base
        """
        rospy.loginfo('mode_selector: calling bb_robot/autonomous_enable service ')
        try:
            self.wheelchair_autonomous_mode_enable_client = rospy.ServiceProxy('/wheelchair/bb_robot/autonomous_enable', Empty)
            self.wheelchair_autonomous_mode_enable_client()
        except rospy.ServiceException, e:
            rospy.logwarn("mode_selector: Service call to bb_robot/autonomous_enable failed (this is probably ok if you are not using this node) : %s", e)

    def voice_head_disable(self):
        rospy.loginfo('mode_selector: executing voice_head_disable service ')
        try:
            self.voice_head_disable_client = rospy.ServiceProxy('voice_and_head/disable', Empty)
            self.voice_head_disable_client()
        except rospy.ServiceException, e:
            rospy.logwarn("mode_selector: Service call failed (this is probably ok if you are not using this node) : %s", e)

    def voice_head_enable(self):
        rospy.loginfo('mode_selector: executing voice_head_enable service ')
        try:
            self.voice_head_enable_client = rospy.ServiceProxy('voice_and_head/enable', Empty)
            self.voice_head_enable_client()
        except rospy.ServiceException, e:
            rospy.logwarn("mode_selector: Service call failed (this is probably ok if you are not using this node) : %s", e)

    def follower_enable(self, bool_value):
        rospy.loginfo('mode_selector: executing follower_enable service %i', bool_value)
        try:
            self.follower_enable_client = rospy.ServiceProxy('/ctrl_node/enable_control', EnableControl)  # We have to send true or false to enable/disable the follower controlle
            succes = self.follower_enable_client(bool_value)
            rospy.loginfo('mode_selector: succes(%s)', succes)
        except rospy.ServiceException, e:
            rospy.logwarn("mode_selector: Service call failed (this is probably ok if you are not using this node) : %s", e)

    def __voice_callback__(self, command):
        data = command.data
        rospy.loginfo("mode_selector: received recognizer_input= %s", data)

        if data == "joystick" or data == "manual":
            self.last_mode = deepcopy(self.mode)
            self.mode = "joystick"
            rospy.loginfo("mode_selector: Set to manual mode")
            self.display_text("Using joystick!!")
            self.wheelchair_manual_mode_enable()

        elif data == "go" or data == "join":
            if self.mode == "in_goal":
                self.mode = "autonomous"

        elif data == "autonomous":
            self.last_mode = deepcopy(self.mode)
            self.mode = "autonomous"
            rospy.loginfo("mode_selector: Set to manual mode")
            self.display_text("Semi-aut mode!")
            self.wheelchair_autonomous_mode_enable()


        elif data == "follow":
            self.last_mode = deepcopy(self.mode)
            self.mode = "follow"
            self.display_text("Follow mode!!")
            rospy.loginfo("mode_selector: Set to follow mode")
            self.wheelchair_autonomous_mode_enable()
#            self.voice_head_disable()
#            self.follower_enable(True)

        elif data == "face":
            self.last_mode = deepcopy(self.mode)
            self.cleanup()
            self.mode = "face"
            self.display_text("Face mode!!")
            rospy.loginfo("mode_selector: Set to face mode")
            self.wheelchair_autonomous_mode_enable()
#            self.voice_head_disable()
#            self.follower_enable(False)

        elif data == "brake":
            self.last_mode = deepcopy(self.mode)
            if self.mode == "autonomous":
                self.mode = "in_goal"
            rospy.loginfo("mode_selector: The wheelchair was stopped")
            self.display_text("Stopping!!")
            self.cleanup()
#            self.voice_head_disable()
#            self.follower_enable(False)

        else:
            rospy.loginfo("mode_selector: Not a recognized command")


    def display_text(self, str):
        marker = self.fill_text_marker(str)
        self.text_marker_pub.publish(marker)


    def fill_text_marker(self, text):
        marker = Marker()
        marker.header.frame_id = self.tf_prefix + '/base_link'
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.TEXT_VIEW_FACING
        marker.text = text
        marker.scale.z = 0.7
        marker.scale.y = 0.7
        marker.scale.x = 0.7
        marker.lifetime = rospy.Duration.from_sec(1.5)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0
        marker.pose.position.x = 0.0
        marker.pose.position.y = 0.0
        marker.pose.position.z = 0.0
        return marker

    def __robot_vel_callback__(self, vel):
        if self.user_vel.linear.x < 0.0:  #If the user is giving a back command is because the robot is crashing so its better to take it into account.
            pass
        else:
            if self.mode == "autonomous":
                if self.user_vel.linear.x < 0.0:  #If the user gave a back command take it instead of robot's velocity.
                    self.cmd_vel == deepcopy(self.user_vel)
                else:
                    self.cmd_vel == deepcopy(vel)

                self.cmd_vel_pub.publish(vel)
            else:
                pass

    def __move_base_result_callback__(self, msg):
        result = MoveBaseActionResult()
        result = msg
        if msg.status.status == msg.status.SUCCEEDED:  #Goal was reached
            self.mode = "in_goal"
        else:
            pass

    def __riskrrt_finished_callback__(self, msg):
        """ This callback happens only when the riskrrt arrives to the given goal
        """
        self.mode = "in_goal"

    def __user_vel_callback__(self, vel):
        self.user_vel = deepcopy(vel)
        if self.user_vel.linear.x < 0.0:  #If the user is giving a back command is because the robot is crashing so its better to take it into account.
            self.cmd_vel == deepcopy(self.user_vel)
            self.cmd_vel_pub.publish(self.user_vel)
        else:
            if self.mode == "face":
               self.cmd_vel == deepcopy(vel)
               self.cmd_vel_pub.publish(vel)
            elif self.mode == "in_goal":  #Just use the angular velocity
                self.cmd_vel.linear.x = 0.0
                self.cmd_vel.angular.z = vel.angular.z
                self.cmd_vel_pub.publish(self.cmd_vel)
            else:
                pass



    def __follower_vel_callback__(self, vel):
        if self.mode == "follow":
           self.cmd_vel == deepcopy(vel)
           self.cmd_vel_pub.publish(vel)
        else:
            pass

#    def __max_proba_value__(self, msg):
#        msg=Float32()
#        self.max_proba_value = deepcopy(msg.data)


#    def __head_pose_callback__(self, msg):
#        self.head_pose = msg


    def cleanup(self):
        # stop the robot!
        vel = Twist()
        self.cmd_vel_pub.publish(vel)


#    def set_user_attention(self):
#        """ Check if the user is focused on driving
#        It will assume that the user is more likely to be paying attention to the driving if he is looking to the front.
#        """
#        orientation = self.head_pose.pose.orientation
#        euler = euler_from_quaternion([orientation.x, orientation.y, orientation.z, orientation.w])
#        pitch = euler[1]
#        roll = euler[0]
#        yaw = euler[2]
#        rospy.loginfo("pitch: %f", pitch)
#        if pitch >= self.pitch_refference:
#            self.user_attention = True
#        else:
#            self.user_attention = False

#===============================================================================
if __name__ == '__main__':
    """Initializing"""
    rospy.init_node('mode_selector')
    try:
        ModeSelector()
    except:
        rospy.logfatal("mode_selector.py  died")
        pass


