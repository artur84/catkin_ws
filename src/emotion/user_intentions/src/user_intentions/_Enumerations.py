#! /usr/bin/env python
""" @package ui

    Some useful definitions used in ui_move_base.h  that i neet to use in user_intentions.py
        @author: Jesus Arturo Escobedo Cabello
        @contact: jesus.escobedo-cabello@inria.fr
        @organization: INRIA Grenoble, Emotion-team
"""
import rospy
import numpy as np
from copy import deepcopy
from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray
from geometry_msgs.msg._Quaternion import Quaternion
import tf

class UIMoveBaseState():
    PLANNING = 1
    CONTROLLING = 2
    CLEARING = 3
    BACK = 4  # Moving back enabled by the user
    TURN = 5  # Turning enabled by the user
    BRAKE = 6  # Stopped
    FORWARD = 7  # Moving forward


class RecoveryTrigger():
    PLANNING_R = 1
    CONTROLLING_R = 2
    OSCILLATION_R = 3


class UIMoveBaseBehavior():
    AUTONOMOUS = 1  # Wheelchair moving according to the ROS navigation system
    FOLLOW = 2  # The wheelchair is following a person
    FACE = 3  # The wheelchair is operated using commands from the user


class WheelchairMode():
    COMPUTER = 1  # wheelchair receiving commands from computer
    JOYSTICK = 2  # wheelchair using joystick


#===============================================================================
# If this code is executed instead of imported we will do this
#===============================================================================
if __name__ == "__main__":
    print "_ui_enumerations.py called as main"
