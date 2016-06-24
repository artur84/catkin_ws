#! /usr/bin/env python
#
import roslib
from copy import deepcopy
from social_filter.msg._humanPose import humanPose
from social_filter.msg._humanPoses import humanPoses
from social_filter.msg._int_data import int_data
from social_filter.msg._int_list import int_list
import threading
from user_intentions._BN import BN
from user_intentions._Enumerations import *
from user_intentions._Markers import UI_Markers
from user_intentions._Meeting_points import FormationType
from user_intentions.cfg import UserIntentionsConfig
from user_intentions.msg._LocalGoal import LocalGoal
from user_intentions.msg._LocalGoalArray import LocalGoalArray

import actionlib
from actionlib_msgs.msg._GoalStatus import GoalStatus
from dynamic_reconfigure.server import Server
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._PoseWithCovarianceStamped import PoseWithCovarianceStamped
from geometry_msgs.msg._Quaternion import Quaternion
from geometry_msgs.msg._Twist import Twist
from move_base_msgs.msg import MoveBaseAction
from move_base_msgs.msg import MoveBaseGoal
import rospy
from std_msgs.msg import String
from std_msgs.msg._Float64 import Float64
from std_msgs.msg._Int32 import Int32
from std_msgs.msg._UInt8 import UInt8
from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray

import numpy as np
import tf  as tf


# Give ourselves the ability to run a dynamic reconfigure server.
# Import custom message data and dynamic reconfigure variables.
#=============================================================================
#=============================================================================
class DestinationInference():

    def __init__(self):
        """Class for computing of current intended destination of a user driving the
            wheelchair.

        """
        """ROS PARAMETERS
        """
        self.movebase_behavior = UIMoveBaseBehavior.AUTONOMOUS
        self.movebase_state = UIMoveBaseState.BRAKE  ## The current mode of the wheelchair,
                                                           # It can be " in_goal", and "moving"
        self.tf_prefix = rospy.get_param('tf_prefix', '')  ## Reads the tf_prefix from the ROS namespace
        if self.tf_prefix is not '':
            self.tf_prefix = '/' + self.tf_prefix
        self.planner = rospy.get_param("~planner", 'ros_planner')
        goals_file = rospy.get_param('~BN_goals_file', '/home/artur/ros_works/pal/teams/emotion/wheelchairconf/world/hall_inria_goals.yaml')
        self.discrete_interface = rospy.get_param('~discrete_interface', False)  ## To enable/disable the discrete mode Set true when using the keyboard
        if self.discrete_interface:
            p_g_g_factor = rospy.get_param('~p_g_g_factor', 2000.0)  ## How important the last computed value will be when updating probabilities
            p_c_gx_sigma = rospy.get_param('~p_c_gx_sigma', 0.4)  ## Standard deviation in p_g_xc formula.
            self.proba_threshold = rospy.get_param("~proba_threshold", 0.99)  # Probability value that a goal should have to be accepted
        else:
            p_g_g_factor = rospy.get_param('~p_g_g_factor', 2000.0)  ## How important the last computed value will be when updating probabilities
            p_c_gx_sigma = rospy.get_param('~p_c_gx_sigma', 1.6)  ## Standard deviation in p_g_xc formula.
            self.proba_threshold = rospy.get_param("~proba_threshold", 0.99)  # Probability value that a goal should have to be accepted
        rospy.loginfo('UI: discrete_interface: %s', self.discrete_interface)
        rospy.loginfo('UI: proba_threshold param: %f' , self.proba_threshold)
        rospy.loginfo('UI: planner param: %s', self.planner)
        rospy.loginfo('UI: tf_prefix: %s', self.tf_prefix)

        """ROS TF
        """
        # NOTE THIS: the listener should be declared in the class not in the function, otherwise it produces errors.
        self.__listener = tf.TransformListener()

        """MEMBERS
        """
        self.lock = threading.Lock()  # Lock used to avoid messing up things with shared resources, because we  have different callbacks that sometimes do the same thing.
        self.bn = BN(goals_file, p_g_g_factor, p_c_gx_sigma)
        self.markers = UI_Markers()
        self.goal_ROS_poses_list = []  # It is necessary to keep the list of ROS poses for the goals in the environment.
        self.meeting_points_goal_array = LocalGoalArray()
        self.last_sent_goal_index = 3  # The +1 is just to make it initially different to goal_index
        self.n = 0  # Index of the current sent goal, for ROS header
        self.goal = MoveBaseGoal(); self.goal_index = 1
        self.roll = 0; self.pitch = 0; self.yaw = 0
        self.user_dir = Quaternion()  # The user commanded direction, it can come from keyboard = key_cmd, voice = voice_cmd, or head =  head_cmd
        self.last_meet_point_size = 0  # Int, number of meeting points at last computation, this number gets refreshed when the meeting points are updated
        self.ui_goal = LocalGoal()  #To send to in the ui_goal topic, will contain the goal with highest probability
        self.meeting_points_dictionary = {}  #A dictionary to store the meeting points as local goals
        self.interaction_list = int_list()  ##It contains the list of meeting points published by the social filter.
        self.interaction_msg = Float64()  ## Controls how big the interaction space will be
        self.interaction_msg.data = 0.40
        self.br = tf.TransformBroadcaster()
        """Creating Move base client"""
        if self.planner != 'riskrrt':
            # This will wait until movebase is available
            self.__subscribe_to_goal_server__()
        """Wait for necessary tfs to be available"""
        while not rospy.is_shutdown():
            try:
                self.__listener.waitForTransform(self.tf_prefix + '/base_link' , self.tf_prefix + '/map', rospy.Time(0), rospy.Duration(10.0))
                break
            except (tf.Exception):
                rospy.logwarn('DEST INF: At init.. no TF  %s/base_link and %s/map was found', self.tf_prefix, self.tf_prefix)
                continue

        # Only when everything is ready we will create the subscribers to avoid bad callbacks
        """ROS SUBSCRIBERS"""
        self.user_dir_sub = rospy.Subscriber("user_dir", Quaternion , self.__user_dir_callback__, None, 1)
        self.vocal_command_sub = rospy.Subscriber("recognizer/output", String , self.__voice_callback__, None, 1)
        self.interaction_list_sub = rospy.Subscriber("interaction_list", int_list , self.__interaction_list_callback__, None, 1)
        self.human_poses_sub = rospy.Subscriber("human_poses", humanPoses , self.__human_poses_callback__, None, 1)
        self.riskrrt_finished_sub = rospy.Subscriber("rosplanner_static/status_rrt", Int32, self.__riskrrt_finished_callback__, None, 1)

        # self.meeting_points_sub = rospy.Subscriber("meeting_points_goal_array", LocalGoalArray, self.__meeting_points_callback__, None, 1 )
        """ROS PUBLISHERS"""
        self.rrrt_goal_pub = rospy.Publisher("goal", PoseStamped, queue_size=10)
        self.dir_marker_pub = rospy.Publisher("dest_inference_dir_marker", Marker, queue_size=10)
        self.goal_marker_pub = rospy.Publisher('goal_marker_array', MarkerArray, queue_size=10)
        self.goal_array_pub = rospy.Publisher('goal_array', LocalGoalArray, queue_size=1)
        self.ui_goal_pub = rospy.Publisher('ui_goal', LocalGoal, queue_size=1)
        self.interaction_msg_pub = rospy.Publisher("interaction_msg", Float64, queue_size=1)
        """ Dynamic Reconfigure Client
        """
#         # Create a dynamic reconfigure server.
#         self.first_reconfigure_callback = True
#         self.dyn_rec_server = Server(UserIntentionsConfig, self.__reconfigure_callback__)
        """Start
        """
        self.init_proba_values_array("go")  # At the begining there is not vocal command but we will init by default with 'go'
        # Print the markers at the begining
        self.lock.acquire()
        try:
            self.print_goal_array(60.0)  # is necessary to do it differently here or in the callback depending on if this is a discrete or continuous interface.
            self.broadcast_static_goals_tf()
        finally:
            self.lock.release()

        r = rospy.Rate(20.0)
        while not rospy.is_shutdown():
            self.lock.acquire()
            #Publish necessary messages to control the social filter.
            self.interaction_msg_pub.publish(self.interaction_msg)
            try:
                self.compute_goals_from_meeting_points()
                self.broadcast_static_goals_tf()
            finally:
                self.lock.release()

            if not self.discrete_interface:
                if self.movebase_state != UIMoveBaseState.BRAKE:
                    self.compute_proba_values_array()
                    self.select_goal()
                    if self.check_goal():
                        #if self.movebase_behavior == UIMoveBaseBehavior.AUTONOMOUS:
                         #   self.send_goal()
                    #else:
                        rospy.logdebug("destination inference: Current goal same as last goal; i will not send it again")
                self.lock.acquire()
                try:
                    self.print_goal_array(0.5)  # is necessary to do it differently here or in the callback depending on if this is a discrete or continuous interface.
                    self.publish_goal_array()
                finally:
                    self.lock.release()
            r.sleep()

    """ subscribes to the goal action server
    """
    def __subscribe_to_goal_server__(self):
        rospy.logwarn("waiting for move_base_server to come up...")
        self.goal_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        # Init ROS Action Server to send the goals to Move_Base
        self.goal_client.wait_for_server()
        rospy.loginfo("move_base_server is up")


    def __user_dir_callback__(self, msg):
        self.lock.acquire()
        try:
            if [msg.x, msg.y, msg.z, msg.w] == [0, 0, 0, 0]:
                rospy.logdebug("Not valid quaternion in destination_inf dir callback will not be considered, %f,%f,%f,%f", msg.x, msg.y, msg.z, msg.w)
                return
            # Get Euler Angles
            self.roll, self.pitch, self.yaw = tf.transformations.euler_from_quaternion([msg.x, msg.y, msg.z, msg.w])
            self.user_dir = msg
            rospy.logdebug("got roll,pitch,yaw: %f,%f,%f", self.roll, self.pitch, self.yaw)
            if (self.discrete_interface and (self.movebase_behavior == UIMoveBaseBehavior.AUTONOMOUS)):
                self.compute_proba_values_array()
                self.select_goal()
                if self.movebase_state != UIMoveBaseState.BRAKE:
                    if self.check_goal():
                        self.send_goal()
                    else:
                        rospy.loginfo("goal didn't pass the check")
                # print the goals in the map
                self.print_goal_array(60.0)
        finally:
            self.lock.release()



#     def __meeting_points_callback__(self, msg):
#         #self.meeting_points_goal_array = LocalGoalArray()
#         self.meeting_points_goal_array = msg
#         self.lock.acquire()
#         try:
#             self.update_meeting_points()
#         finally:
#             self.lock.release()

    def update_meeting_points(self):
        """Check if there are new meeting points in the list, then it will update the values
        for each goal.
        """
        current_size = deepcopy(len(self.meeting_points_goal_array.goals))  #Current meeting points array size
        if current_size > self.last_meet_point_size:
            rospy.loginfo("Meetings points added")
            self.bn.add_goals(self.meeting_points_goal_array)
            self.reset_prior_matrix()
        elif current_size < self.last_meet_point_size:
            # TODO: print "DO something here erase goals"
            rospy.loginfo("Meetings points erased")
            self.bn.delete_goals(self.meeting_points_goal_array)
            self.reset_prior_matrix()
        else:
            # if we have the same number of goals we then just update the positions
            for n in range(0, current_size):
                x = self.meeting_points_goal_array.goals[n].pose.x
                y = self.meeting_points_goal_array.goals[n].pose.y
                theta = self.meeting_points_goal_array.goals[n].pose.theta
                self.bn.change_goal_position(n, x , y, theta)
        self.last_meet_point_size = deepcopy(current_size)
        self.print_goal_array(0.4)

    def publish_goal_array(self):
        """ Publishes the list of goals as a LocalGoalArray msg
        """
        goal_poses_array = self.bn.get_list_of_goals()
        tags = self.bn.get_list_of_tags()
        goal_array = LocalGoalArray()
        goal = LocalGoal()
        goal_array.header.frame_id = self.tf_prefix + '/map'
        for i in range(np.shape(goal_poses_array)[0]):
            goal.id = i
            goal.pose.x = goal_poses_array[i][0]
            goal.pose.y = goal_poses_array[i][1]
            goal.pose.theta = goal_poses_array[i][2]
            goal.proba = self.proba_values_array[i]
            goal.tag = tags[i]
            goal_array.goals.append(deepcopy(goal))
        goal_array.header.stamp = rospy.Time.now()
        self.goal_array_pub.publish(goal_array)

    def __voice_callback__(self, msg):
        """ It will receive the vocal command from the user, we assume that it will be perfect (already filtered)
        """
        rospy.sleep(rospy.Duration(0.1))
        self.vocal_command = deepcopy(msg.data)
        self.lock.acquire()
        try:
            self.check_action_to_do(msg.data)
        finally:
            self.lock.release()


    def check_action_to_do(self, data):
        """ Selects the adequate action to do according to the vocal command, we assume that the vocal command was perfect
        """
        if data == "brake":
            self.movebase_state = UIMoveBaseState.BRAKE
            self.stop_goal()
            self.print_goal_array(60)

        elif data == "go" :
            if (self.movebase_state == UIMoveBaseState.BACK) or (self.movebase_state == UIMoveBaseState.BRAKE):
                self.movebase_behavior = UIMoveBaseBehavior.AUTONOMOUS
            self.movebase_state = UIMoveBaseState.CONTROLLING
            #self.init_proba_values_array('go')  # one to consider prior
            self.compute_proba_values_array()  # other to consider head
            self.print_goal_array(60)
            self.select_goal()
            self.send_goal()

        elif data == "back" :
            self.movebase_behavior = UIMoveBaseBehavior.FACE
            self.movebase_state = UIMoveBaseState.BACK

        elif data == "join" :
            self.movebase_state = UIMoveBaseState.CONTROLLING
            self.init_proba_values_array('join')  # one to consider prior
            self.compute_proba_values_array()  # other to consider head
            self.print_goal_array(60)
            self.select_goal()
            self.send_goal()

        elif data == "autonomous":
            self.movebase_behavior = UIMoveBaseBehavior.AUTONOMOUS
            self.init_proba_values_array('go')  # one to consider prior
            self.print_goal_array(60)

        elif data == "face" :
            self.movebase_behavior = UIMoveBaseBehavior.FACE
            self.movebase_state = UIMoveBaseState.BRAKE
            self.stop_goal()
            self.print_goal_array(60)

        else:
            rospy.logwarn("destination_inference.py: Not a valid command")


    def compute_proba_values_array(self):
        goal_yaw_array, goal_pitch_array, goal_dist_array = self.compute_angle_to_goals()
        self.bn.compute_p_g_xcv(self.yaw, goal_yaw_array)
        self.proba_values_array = self.bn.get_p_g_xc()  # proba_values_array: numpy array containing [p0, p1, p2, ....]

    def init_proba_values_array(self, vocal_command):
        """ This function will init the P(G|X0,V0) considering X=X0  where X0 is the closest
            POI to the current destination, V0 in {'go','join'} is the initial command (by default "go" is considered).
            It sets the self.proba_values_array member with the result.
        """
        if vocal_command == 'go' or vocal_command == 'join':
            goal_yaw_array, goal_pitch_array, goal_dist_array = self.compute_angle_to_goals()
            # The starting position is taken as the one closer to the robot position.
            pos = goal_dist_array.argmin()
            self.starting_location_index = deepcopy(pos)
            self.bn.init_p_g_xcv(pos, vocal_command)
            self.proba_values_array = self.bn.get_p_g_xc()  # proba_values_array: numpy array containing [p0, p1, p2, ....]
        else:
            rospy.logfatal("self.vocal_command has not a valid value, it has to be {'go','join'}")
            raise ValueError()

    def reset_prior_matrix(self):
        """ This resets the prior proba matrix when new meeting points are deleted or added
        """
        # TODO: FIX THIS FUNCTION.
        goal_yaw_array, goal_pitch_array, goal_dist_array = self.compute_angle_to_goals()
        # The starting position is taken as the one closer to the robot position.
        pos = goal_dist_array.argmin()
        self.starting_location_index = deepcopy(pos)
        self.bn.init_p_g_xcv(pos, "go")  # TODO: I will send go for the moment but I will have to fix it later
        self.proba_values_array = self.bn.get_p_g_xc()

    def select_goal(self):
        if self.proba_values_array is not None:
            self.goal_index = np.where(self.proba_values_array == self.proba_values_array.max())[0][0]  # The [0] [0] is to use the element 0 of tuple 0 from the answer
        else:
            rospy.logerror("self.proba_values_array is  None, first call compute_proba_values_array()")

    def check_goal(self):
        """Check if the current estimated goal can be accepted to be sent to the autonomous navigation system.
        This will depend on its probability value, last computed goal
        @return: goal_ok : bool, True if goal is good.
        """
        if self.proba_values_array is None:
            rospy.logerror("self.proba_values_array is  None, first call compute_proba_values_array()")
            return
        goal_ok = False  # Bool, indicate if current computed goal is good enough to be sent
        """If probability is high enough"""
        if (self.proba_values_array[self.goal_index] >= self.proba_threshold):
            """If last sent goal was the same don't send it again"""
            if (self.goal_index != self.last_sent_goal_index):
                goal_ok = True
            else:
                rospy.logdebug("The goal is the same as last one, so i will not send it again!!")
                goal_ok = False
        return goal_ok

    def send_goal(self):
        if self.movebase_behavior == UIMoveBaseBehavior.AUTONOMOUS:
            self.set_goal_ROS_poses_list()
            goal_pose = self.goal_ROS_poses_list[self.goal_index]
            goal_tag = self.bn.get_list_of_tags()[self.goal_index]
            self.n += self.n
            self.goal.target_pose.header.frame_id = "/map"  # Is important to keep this in /map DON'T CHANGE to /robot_0/map
            self.goal.target_pose.header.stamp = rospy.Time.now()
            self.goal.target_pose.header.seq = self.n
            self.goal.target_pose.pose.position = deepcopy(goal_pose.pose.position)

            """Fill ui goal """
            self.ui_goal.pose.x = goal_pose.pose.position.x
            self.ui_goal.pose.y = goal_pose.pose.position.y
            self.ui_goal.tag = goal_tag
            self.ui_goal.id = self.goal_index
            self.ui_goal.proba = self.proba_values_array[self.goal_index]
            self.ui_goal_pub.publish(self.ui_goal)
            # It was hard for me to discover that I needed to use this temporal_quat instead of
            # direct assignment to the self...orientation
            self.goal.target_pose.pose.orientation = deepcopy(goal_pose.pose.orientation)
            if self.planner == "riskrrt":  # This topic should only be published for riskrrt (never send it when using a goal action)
                self.rrrt_goal_pub.publish(goal_pose)
            # Send the goal with highest probability
            else:
                self.goal_client.send_goal(self.goal, self.__goal_finished_callback__, None , None)
            rospy.loginfo("last_sent_goal_index: %f", self.goal_index)
            self.last_sent_goal_index = deepcopy(self.goal_index)
        else:
            rospy.loginfo("I will not send the goal because we are not in autonomous mode")


    def __goal_finished_callback__(self, terminal_state, result):
        """ It can be in Status = 3: succeeded or status = 2 : replaced with other goal
        """
        rospy.loginfo("goal finished callback")
        rospy.loginfo("terminal state:%d", terminal_state)
        # self.last_sent_goal_index = deepcopy(self.goal_index)
        if terminal_state == GoalStatus.SUCCEEDED:  # When we arrive to a destination reinit bayesian network
            self.movebase_state = UIMoveBaseState.BRAKE
            self.lock.acquire()
            try:
                self.init_proba_values_array("go")
                self.print_goal_array(60.0)
            finally:
                self.lock.release()

    def __riskrrt_finished_callback__(self, msg):
        """ This callback happens only when the riskrrt arrives to the given goal
        """
        rospy.loginfo("Riskrrt goal finished callback")
        self.movebase_state = UIMoveBaseState.BRAKE
        self.lock.acquire()
        try:
            self.init_proba_values_array("go")
            self.print_goal_array(60.0)
        finally:
            self.lock.release()

#    def __goal_active_callback__(self):
#        """ This is called when we just sent a new goal. Notice that this is not called when the
#        same goal is sent twice
#        """
#        print "############################"
#        print "active callback"
#        print ":::::::::::::::::::::::::::::"

#    def __goal_feedback_callback__(self, base_position):
#        goal_status = self.goal_client.get_state()
#        duration = 5.0

    def set_goal_ROS_poses_list(self):
        goal_poses_array = self.bn.get_list_of_goals()  # goal_poses_array: numpy array containing [[x0,y0,theta0],.....]
        self.goal_ROS_poses_list = self.transform_goals_to_stamped_poses(goal_poses_array)

    def print_goal_array(self, lifetime):
        # Publish the MarkerArray
        marker_array = MarkerArray()
        tags = self.bn.get_list_of_tags()
        self.set_goal_ROS_poses_list()
        if self.proba_values_array is None:
            rospy.logerror("self.proba_values_array is  None, first call compute_prob_values_array()")
            return
        marker_array = self.markers.fill_goal_marker_array(self.goal_ROS_poses_list, self.proba_values_array, tags, lifetime = lifetime)
        self.goal_marker_pub.publish(marker_array)

    def stop_goal(self):
        rospy.loginfo("UI: Stop goal")
        if self.planner != "riskrrt":
            self.goal_client.cancel_all_goals()  # cancell the goal
        else:
            rospy.logerr("UI: you have to add a function to stop riskrrt planner here")
        self.init_proba_values_array("go")  # it sets the self.proba_values_array with initial values


    def compute_angle_to_goals(self):
        """ Transform the position of the goals in the environment to the local frame (base_link)
            @return:  goal_yaw_array, goal_pitch_array, goal_dist_array: np.arrays, containing the angles and distance to goals from current position.
        """
        self.set_goal_ROS_poses_list()
        goal_yaw_array = []; goal_dist_array = []; goal_pitch_array = []
        """ I had several problems to get working this listener.waitForTransform one of the main problems was that I didn't use the
        first call self.__listener.waitForTransform before the loop, then in many ROS tutorials they use  rospy.Time() where rospy.Time.now() is
        necessary. Sometimes COPY-PASTE is really bad :)
        In REAL I don't fcking need this!!!
        """
        r = rospy.Rate(10.0)
        while not rospy.is_shutdown():
            try:
                # This returns a tuple like this one ((x, y, z), quat=(x, y, z, w))
                now = rospy.Time.now()
                self.__listener.waitForTransform(self.tf_prefix + '/base_link' , self.tf_prefix + '/map', now, rospy.Duration(0.5))
                break
            except (tf.Exception):
                rospy.logwarn('DEST INF: waiting for TF between %s/base_link and %s/map to become available', self.tf_prefix, self.tf_prefix)
                continue
        # Finds the angle between the command and each goal in the map.
        for goal_global in self.goal_ROS_poses_list:
            goal_relative = self.__listener.transformPose(self.tf_prefix + '/base_link' , goal_global)
            # distance to goal
            goal_dist = np.sqrt(np.power(goal_relative.pose.position.x, 2) + np.power(goal_relative.pose.position.y, 2))  ## Vector with the distances from the current robot position to each of the possible destinations.
            goal_dist_array = np.append(goal_dist_array, goal_dist)
            # yaw to goal
            goal_yaw = np.arctan2(goal_relative.pose.position.y , goal_relative.pose.position.x)
            goal_yaw_array = np.append(goal_yaw_array, goal_yaw)
            # pitch to goal
            goal_pitch = np.arctan2(goal_relative.pose.position.z, goal_dist)
            goal_pitch_array = np.append(goal_pitch_array, goal_pitch)
        return goal_yaw_array, goal_pitch_array, goal_dist_array

    def transform_goals_to_stamped_poses(self, goal_list):
        """ @param goal_list: python list [], is the list of [X, Y, THETA] goals
        """
        goal_pose_list = []
        goalPose = PoseStamped()
        for i in range(len(goal_list)):
            goalPose.pose.position.x = goal_list[i][0]
            goalPose.pose.position.y = goal_list[i][1]
            try:
                orientation = tf.transformations.quaternion_from_euler(0, 0, float(goal_list[i][2]))
            except:
                rospy.logerr("destination_inference: tf transform euler to quaternion failed")
            goalPose.pose.orientation.x = orientation[0]
            goalPose.pose.orientation.y = orientation[1]
            goalPose.pose.orientation.z = orientation[2]
            goalPose.pose.orientation.w = orientation[3]
            goalPose.header.frame_id = 'map'
            goal_pose_list.append(deepcopy(goalPose))
        return goal_pose_list

    def __reconfigure_callback__(self, config, level):
        """This callback is activated when the parameters are changed by rqt_reconfigure or dynamic reconfigure"
        """
#        print "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"
#        if (self.first_reconfigure_callback):
#            rospy.logdebug("I will not use the first reconfigure callback")
#            self.first_reconfigure_callback=False
#        else:
#            rospy.loginfo("Updating destination_inference parameters")
#            p_g_g_factor = config["p_g_g_factor"]
#            p_c_gx_sigma = config["p_c_gx_sigma"]
#            self.bn.set_param_values(p_g_g_factor, p_c_gx_sigma)
#            self.proba_threshold = config["proba_threshold"]
#            self.discrete_interface  = config["discrete_interface"]
        return config  # This callback has to return a possible configuration otherwise it fails with no warning
    def __interaction_list_callback__(self, msg):
        """ It will receive interaction list from fformdetect node
        """
        self.lock.acquire()
        try:
            self.interaction_list = deepcopy(msg)
        finally:
            self.lock.release()


    def __human_poses_callback__(self, msg):
        """ It will receive human poses from human_proc node
        """
        self.human_poses_list = deepcopy(msg)

    def broadcast_static_goals_tf(self):
        tag_list = self.bn.get_list_of_static_tags()
        goal_list = self.bn.get_list_of_static_goals()
        n = 0
        for tag in tag_list:
            try:
               self.br.sendTransform((goal_list[n][0], goal_list[n][1], 0),
                     tf.transformations.quaternion_from_euler(0, 0, goal_list[n][2]),
                     rospy.Time.now(),
                     tag + str(n),
                     self.tf_prefix + '/map')
               n += 1
            except (tf.Exception):
                rospy.logwarn('I could not broadcast the goal transformation')
                return

#    def listen_goals_tf(self):
#        """ This will read the TF server to check if there exist any /group_n or /person_n frame
#            which comes from an interaction or personal meeting point
#        """
#        self.meeting_points_goal_array = LocalGoalArray()
#        goal = LocalGoal()
#        for n in range(1, 5):  #Persons go from 1 to n
#            try:
#                # This returns a tuple like this one ((x, y, z), quat=(x, y, z, w))
#                now = rospy.Time.now()
#                (trans, rot) = self.__listener.lookupTransform(self.tf_prefix + '/map', 'person' + str(n), rospy.Time(0))
#            except tf.Exception as error:
#                rospy.logdebug('No tf person_%f', n)
#                continue
#            goal.pose.x = trans[0]
#            goal.pose.y = trans[1]
#            goal.id = n
#            goal.tag = 'person'
#            goal.proba = 0
#            self.meeting_points_goal_array.goals.append(deepcopy(goal))
#
#        for n in range(0, 5):  #groups goes from 0 to n
#            try:
#                # This returns a tuple like this one ((x, y, z), quat=(x, y, z, w))
#                now = rospy.Time.now()
#                (trans, rot) = self.__listener.lookupTransform(self.tf_prefix + '/map', 'group' + str(n), rospy.Time(0))
#            except tf.Exception as error:
#                rospy.logdebug('No tf group %f', n)
#                continue
#            goal.id = n
#            goal.pose.x = trans[0]
#            goal.pose.y = trans[1]
#            goal.tag = 'group'
#            goal.proba = 0
#            self.meeting_points_goal_array.goals.append(deepcopy(goal))
#
#        try:
#            self.update_meeting_points()
#        except Exception as error:
#            print error

    def compute_goals_from_meeting_points(self):
        """ This will read the interaction_list published by social_filter and extract the
        meeting points information
        """
        user_intentions_goal = LocalGoal()
        user_intentions_goal.tag = "group"
        user_intentions_goal.proba = 0.0
        self.meeting_points_goal_array = LocalGoalArray()
        interaction = int_data()
        for interaction in self.interaction_list.formation:
            if interaction.type != FormationType.CIRCULAR and interaction.type != FormationType.HOI:  # IF not circular nor HOI then there are meeting points
                if interaction.type == FormationType.VIS_VIS or interaction.type == FormationType.V_FORM:  # IF VIS_VIS there are 2 meeting points"
                    # Append first meeting point
                    user_intentions_goal.id = ord(interaction.id_members[0])  # XXX: It was hard to figure that uint8 ROS structure was needed to be translated using ord()
                    user_intentions_goal.pose.x = interaction.meet_points[0]
                    user_intentions_goal.pose.y = interaction.meet_points[1]
                    user_intentions_goal.pose.theta = interaction.meet_points[2]
                    self.meeting_points_goal_array.goals.append(deepcopy(user_intentions_goal))
                    # Append second meeting point
                    user_intentions_goal.id = ord(interaction.id_members[1])  # XXX: It was hard to figure that uint8 ROS structure was needed to be translated using ord()
                    user_intentions_goal.pose.x = interaction.meet_points[3]
                    user_intentions_goal.pose.y = interaction.meet_points[4]
                    user_intentions_goal.pose.theta = interaction.meet_points[5]
                    self.meeting_points_goal_array.goals.append(deepcopy(user_intentions_goal))
                else:  # if not vis vis nor v_form
                    # Append first meeting point
                    user_intentions_goal.id = ord(interaction.id_members[0])  # XXX: It was hard to figure that uint8 ROS structure was needed to be translated using ord()
                    user_intentions_goal.pose.x = interaction.meet_points[0]
                    user_intentions_goal.pose.y = interaction.meet_points[1]
                    user_intentions_goal.pose.theta = interaction.meet_points[2]
                    self.meeting_points_goal_array.goals.append(deepcopy(user_intentions_goal))
            else:
                #rospy.logerr("destination_inference.py, No meeting points received")
                return

        try:
            self.update_meeting_points()
        except Exception as error:
            rospy.logerr("destination_inference.py, Could not update_meeting_points() because of: %s", error)


#===============================================================================
if __name__ == '__main__':
    """Initializing ROS node"""
    rospy.init_node('destination_inference')
    try:
        DestinationInference()
    except Exception as detail:
        rospy.logfatal("destination_inference.py  died because of: %s", detail)




