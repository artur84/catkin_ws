#! /usr/bin/env python
#
import roslib
from copy import deepcopy
import rospy
from std_msgs.msg._Float64 import Float64
from std_msgs.msg._UInt8 import UInt8


#=============================================================================
#==============================================================================
class SocialFilterHandler():

    def __init__(self):
        """Class for sending some necessary topics to control the social_filter.
        """



        """MEMBERS """
        #self.meeting_points_goal_array = LocalGoalArray()
        self.interaction_msg = Float64()  ## Controls how big the interaction space will be
        self.interaction_msg.data = 0.40
        self.pspace_msg = Float64()  ## Controls how big the personal space will be
        self.pspace_msg.data = 0.40

        """ROS PUBLISHERS"""
        self.pspace_msg_pub = rospy.Publisher("/robot_0/rosplanner_static/pspace_msg", Float64, queue_size=1)  ##Publishes a number to control how wide should the personal space be.
        self.interaction_msg_pub = rospy.Publisher("/robot_0/rosplanner_static/interaction_msg", Float64, queue_size=1)

        """Start
        """
        r = rospy.Rate(10.0)


        while not rospy.is_shutdown():
            self.interaction_msg_pub.publish(self.interaction_msg)
            self.pspace_msg_pub.publish(self.pspace_msg)
            r.sleep()

if __name__ == '__main__':
    """Initializing ROS node"""
    rospy.init_node('SocialFilterHandler')
    try:
        SocialFilterHandler()
    except Exception as error:
        rospy.logfatal("SocialFilterHandler  died: %s" , error)
        pass
