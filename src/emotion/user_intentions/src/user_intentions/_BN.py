""" @package ui

    This code is used to manage the computing of the Bayesian Network to infer the
   desired goal in user_intentions
        @author: Jesus Arturo Escobedo Cabello
        @contact: jesus.escobedo-cabello@inria.fr
        @organization: INRIA Grenoble, Emotion-team
"""
#! /usr/bin/env python
#
from __builtin__ import exit
from copy import deepcopy
import math
import sys
from user_intentions.msg._LocalGoal import LocalGoal
from user_intentions.msg._LocalGoalArray import LocalGoalArray
import yaml

import rospy

import numpy as np


#==============================================================================
#==============================================================================
class BN:
    def __init__(self, goals_file, p_g_g_factor, p_c_gx_sigma):
        """Members
        """
        self.__p_g_xc_init_flag___ = False  # To indicate if p_g_xc was init
        self.__init_data_flag__ = False  # To indicate that data was read
        self.p_g_g_factor = p_g_g_factor
        self.p_c_gx_sigma = p_c_gx_sigma
        # BLA_BLA_array -> it means that it is a numpy array
        # BLA_BLA_list -> it means it is a python list
        self.local_X_array = np.array([])
        self.local_Y_array = np.array([])
        self.tag_list = []  # The list containing the type of each goal dynamic or static
        """Init data
        """
        self.read_data(goals_file)
        self.__compute_p_g_g__()

    def set_param_values(self, p_g_g_factor, p_c_gx_sigma):
        self.p_g_g_factor = p_g_g_factor
        self.p_c_gx_sigma = p_c_gx_sigma


    def read_data(self, path):
        """ Reads the data of typical destinations in the selected scenario
        """
        try:
            rospy.loginfo("Trying to read File: %s", path)
            with open(path) as opened_file:
                self.data = yaml.load(opened_file)
                rospy.loginfo("File loaded from: %s", path)
        except IOError as exc:
            rospy.logerr("The file CAN'T BE READ: %s", path)
        self.static_goals_X_Y_THETA_array = np.asfarray(self.data["goals"])  # The matrix containing [x,y,theta] for each goal
        self.goals_X_Y_THETA_array = deepcopy(self.static_goals_X_Y_THETA_array)
        self.__p_g_xc0__ = np.asfarray(self.data["p_g_xc0"])  # Prior probability
        self.static_goals_prior = deepcopy(self.__p_g_xc0__)
        self.static_tag_list = self.data['tags']  # The tag for each goal
        self.tag_list = deepcopy(self.static_tag_list)
        self.p_desired_measured = np.asfarray(self.data["p_desired_measured"])
        self.number_of_static_goals = deepcopy(np.shape(self.static_goals_X_Y_THETA_array)[0])
        # print self.p_desired_measured
        opened_file.close()
        self.__init_data_flag__ = True

    def add_goals(self, user_intention_local_goal_array):
        """ This function will add the received goals to the current self.goals_X_Y_THETA_array
            It will also update the probabilities so that they are re normalized
            @param user_intention_local_goal_array: LocalGoalArray() The datatype is defined in user_intentions package
        """
        # Copy the static goals
        self.goals_X_Y_THETA_array = deepcopy(self.static_goals_X_Y_THETA_array)
        self.tag_list = deepcopy(self.static_tag_list)
        for goal in user_intention_local_goal_array.goals:
            goal_pose = deepcopy(np.asarray([goal.pose.x, goal.pose.y, 0.0]))
            concatenated_goals = np.vstack((self.goals_X_Y_THETA_array, goal_pose))
            self.goals_X_Y_THETA_array = deepcopy(concatenated_goals)
            self.tag_list.append(deepcopy(goal.tag))
        self.__reset_p_g_xc0__()
        """As the matrix grows so p_g_g"""
        self.__compute_p_g_g__()
        """ As the matrix grows we also need to update p_g_x_c"""
        self.init_p_g_xc(0)  # I put 0, actually as the probability distribution is uniform it doesn't care which one to use

    def delete_goals(self, user_intention_local_goal_array):
        """ This function will delete the received goals in the current self.goals_X_Y_THETA_array
            @param user_intention_local_goal_array: LocalGoalArray() The datatype is defined in user_intentions package
        """
        # Copy the static goals
        self.goals_X_Y_THETA_array = deepcopy(self.static_goals_X_Y_THETA_array)
        self.tag_list = deepcopy(self.static_tag_list)
        for goal in user_intention_local_goal_array.goals:
            goal_pose = deepcopy(np.asarray([goal.pose.x, goal.pose.y, 0.0]))
            concatenated_goals = np.vstack((self.goals_X_Y_THETA_array, goal_pose))
            self.goals_X_Y_THETA_array = deepcopy(concatenated_goals)
            self.tag_list.append(deepcopy(goal.tag))
        # TODO:It should work only when all the meeting points disapear at once, so I have to change it to consider more cases
        self.__reset_p_g_xc0__()
        """As the matrix grows so p_g_g"""
        self.__compute_p_g_g__()
        """ As the matrix grows we also need to update p_g_x_c"""
        self.init_p_g_xc(0)  # I put 0, actually as the probability distribution is uniform it doesn't care which one to use


    def change_goal_position(self, index, x, y, theta):
        """ This will just update de position of the goal defined by index, with x, y, and theta
        """
        global_index = self.number_of_static_goals + index
        self.goals_X_Y_THETA_array[global_index][0] = x
        self.goals_X_Y_THETA_array[global_index][1] = y
        self.goals_X_Y_THETA_array[global_index][2] = theta

    def __reset_p_g_xc0__(self):
        """ This function will update self.__p_g_xc0__ each time new goals are added to the list
            For the moment it will reset everything as a uniform distribution but it should be changed if
            I have more time.
        """

        n = self.goals_X_Y_THETA_array.shape[0]  # the number of rows
        epsilon = 0.000001
        x = np.ones((n, n))
        y = epsilon * x
        # copy static goals prior in y
        y[0:self.static_goals_prior.shape[0], 0:self.static_goals_prior.shape[1]] = self.static_goals_prior
        x = self.normalize(y)
        self.__p_g_xc0__ = deepcopy(x)

    def normalize(self, y):
        """Normalizes a numpy matrix
            y: the matrix whose values will be normalized
            @return: x, the normalized matrix
        """
        x = np.ones((y.shape[0], y.shape[0]))
        normalizer = np.sum(y, 1)
        i = 0
        for element in normalizer:
            x[i] = y[i] / element
            i = i + 1
        return x


    def __compute_p_g_g__(self):
        """ Function to compute P(Gt|Gt-1) which is the probability that the current Goal "Gt" is different from the previous one "Gt-1"
        This probability is set at the beginning and remains constant.
        """
        n = self.goals_X_Y_THETA_array.shape[0]
        normalizer = 1.0 / (n + self.p_g_g_factor - 1.0)
        # Creates an squared matrix of nXn where n is the number of goals
        A = np.ones((n, n))
        # Creates a diagonal matrix of n x n where the elements of the diagonal
        # are "self.p_g_g_factor" times bigger than elements in matrix A
        x = np.ones((1, n))
        # It was hard to know how to use it. x[0] is necessary
        diag = (self.p_g_g_factor - 1.0) * np.diag(x[0])
        self.p_g_g = normalizer * (A + diag)  # #  @param self.p_g_g: Is a matrix of n x n, where n = length of goals_X_Y_THETA_array.\
                                                # #  The elements of its diagonal are "self.p_g_g_factor times bigger than the other elements and each row/column adds-up = 1

    def __compute_p_c_gx__(self, command_angle, angle_to_goals_array):
        """    Computes the P(C|G,X) probability which is the probability that the user is pointing to a given goal. Closer goals to the commanded direction have a bigger probability
        given the direction of the command.

        """
        # distance_to_goals_array = np.sqrt(self.local_X_array ** 2 + self.local_Y_array ** 2)         ## Vector of distances from the current robot position to each of the possible destinations..
        # Compute the difference between the command and the angle to each of the goals.
        ai = command_angle - angle_to_goals_array
        # Compute the non normalized probability p_c_gx for each goal.
        pp = (1.0 / (self.p_c_gx_sigma * np.sqrt(2.0 * np.pi))) * np.exp(-(1.0 / 2.0) * (ai / self.p_c_gx_sigma) ** 2)
        # Compute the normalized probability
        self.p_c_gx = np.abs((1 / sum(pp)) * pp)  # # Is a numpy array of probabilities,this probability decreases with the angle between the command and each goal
        return  self.p_c_gx

    def compute_p_g_xc(self, command_angle, angle_to_goals_array):
        """   Computes the posterior intention probability for each Goal in the environment.
        It is expressed as P(Gt|X0:t,C0:t)
            @param command_angle: Float, angle of the command
            @param angle_to_goals_array: numpy array, An array containing the angle from the current position to each of the goals in the scenario
            @param last_goal_idx: Int, the index of the last sucsessful goal.Which should not be take into account in calcualtions.
        """
        if not self.__p_g_xc_init_flag___:
            rospy.logerr("p_g_xc has not been initialized yet, I can not compute_p_g_xc")
            return
        # rospy.loginfo( 'COMPUTE P(G|C,P) STARTED' )
        self.__p_g_xc_previous__ = self.__p_g_xc__
        # Get [(P__command|G0,__pose),(P_command|G1,__pose),(P__command|G2,__pose)]
        self.__compute_p_c_gx__(command_angle, angle_to_goals_array)
        # Computing the non-normalized posterior
        p_g_xc = self.p_c_gx * (np.dot(self.p_g_g, np.transpose(self.__p_g_xc_previous__)))
        # Avoid taking into account as possible goal the current location.
        # Normalizing the probabilities
        normalizer = 1.0 / sum(p_g_xc)
        self.__p_g_xc__ = normalizer * p_g_xc

    def compute_p_g_xcv(self, command_angle, angle_to_goals_array):
        """
        @return: False if there is an error.
        @return: self.__p_g_xc__ numpy array, with the values for each goal
        """
#         #Security checks
#         if not self.__p_g_xc_init_flag___:
#             rospy.logerr("self.__p_g_xc__ has not been initialized yet, I can not compute_p_g_xcv")
#             return False
        # rospy.loginfo( 'COMPUTE P(G|C,P) STARTED' )
        self.__p_g_xc_previous__ = self.__p_g_xc__
        # Get [(P__command|G0,__pose),(P_command|G1,__pose),(P__command|G2,__pose)]
        self.__compute_p_c_gx__(command_angle, angle_to_goals_array)
        # Computing the non-normalized posterior
        p_g_xc = self.p_c_gx * (np.dot(self.p_g_g, np.transpose(self.__p_g_xc_previous__)))
        # Avoid taking into account as possible goal the current location.
        # p_g_xc[starting_location_index] = 0.0000001
        # Normalizing the probabilities
        normalizer = 1.0 / sum(p_g_xc)
        self.__p_g_xc__ = normalizer * p_g_xc
        return self.__p_g_xc__

    def init_p_g_xcv(self, starting_location_index, voice_command):
        """This is necessary to init the values when no vocal command has arrived
            @param starting_location_index: int, the goal closer to the current wheelchair position.
        """
        if voice_command == "join":
            p_g_v = []
            i = 0
            for tag in self.tag_list:
                if tag == "group" or tag == "person":
                    p_g_v.append(10.0)
                else:
                    p_g_v.append(1.0)
                i = i + 1
            # Normalizing the probabilities
            pgv_array = np.asarray(p_g_v)
            normalizer = 1.0 / sum(pgv_array)
            self.__p_g_xc__ = normalizer * pgv_array

        elif voice_command == "go":  # voice_command == "go":
            self.init_p_g_xc(starting_location_index)
        else:
            rospy.logfatal("Not valid voice_command received when executing init_p_g_xcv function")
            raise ValueError("Not valid voice_command received when executing init_p_g_xcv function")


    def init_p_g_xc(self, starting_location_index):
        """This is necessary to init the values when no vocal command has arrived
            @param starting_location_index: int, the goal closer to the current wheelchair position.
        """
        self.__p_g_xc_init_flag___ = True
        self.__p_g_xc__ = np.array(self.__p_g_xc0__[starting_location_index])  # # @var self.p_g_x_c: np.array, posterior probability

    def get_p_g_xc(self):
        if self.__p_g_xc_init_flag___ == True:
            return self.__p_g_xc__
        else:
            rospy.logerr("BN: self.__p_g_xc__ is not initialized")
            return None

    def get_list_of_goals(self):
        if self.__init_data_flag__ == True:
            return self.goals_X_Y_THETA_array
        else:
            rospy.logerr("BN:  self.goals_X_Y_THETA_array is not initialized")
            return None

    def get_list_of_static_goals(self):
        if self.__init_data_flag__ == True:
            return self.static_goals_X_Y_THETA_array
        else:
            rospy.logerr("BN:   self.static_goals_X_Y_THETA_array is not initialized")
            return None


    def get_list_of_tags(self):
        if self.__init_data_flag__ == True:
            return self.tag_list
        else:
            rospy.logerr("BN:  self.tag_list is not initialized")
            return None

    def get_list_of_static_tags(self):
        if self.__init_data_flag__ == True:
            return self.static_tag_list
        else:
            rospy.logerr("BN: self.static_tag_list")
            return None



