#!/usr/bin/env python
import roslib
import rospy
import math
import tf
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from copy import deepcopy

class HeadCmd():


    def __init__(self):
        """
        head.py is a node in charge to get the position of the face published by
        pi_face_tracker, and send this info to command the wheelchair or as input for the user_intentions
        algorithm
        """
        rospy.on_shutdown(self.cleanup)
        """ Constants
        """
        self.angular_velocity = 1.2
        self.linear_velocity = 0.0
        self.zero_zone = 0.1
        self.time_threshold = 1.0  # to interrupt velocity commands if it has been the same during this period of time.
                                    # normally indicating that something went wrong with the kinect
        """Members
        """
        self.head_dir = Twist()
        self.head_vel = Twist()
        self.rotation_from_tf = 0.0  # The value of rotation without filtering
        self.rotation = 0.0  # The value of head rotation that we really want to publish
        self.rotation_filtered = 0.0  # The value of head's rotation coming from filter
        self.last_rotation = 0.1  # Just to be different to self.rotation
        self.filtered_head_pose = PoseStamped()
        self.__listener = tf.TransformListener()  # NOTE THIS: the listener should be declared in the class
        self.timer = rospy.Time()  # To check if the head has sent the same velocity command for a while. It can be an error.
        self.timer_duration = rospy.Duration()
        self.first_time = 1  # flag to indicate if it is the first time the timer is called

        """ Publishers
        """
        self.head_vel_pub = rospy.Publisher('head_vel', Twist, queue_size=10)
        self.head_dir_pub = rospy.Publisher('head_dir', Twist, queue_size=10)

        """ Subscribers
        """
        rospy.Subscriber('head_pose_filtered', PoseStamped, self.headCb)

        self.cleanup()
        r = rospy.Rate(4.0)
        while not rospy.is_shutdown():
            try:
                now = rospy.Time(0)
                # wait transformation for 500 msec = 500 000 000 nsec )
                # It caused the node to die
                # self.__listener.waitForTransform( '/new_ref', '/head_origin', now, rospy.Duration( 1, 500000000 ))
                # self.__listener.waitForTransform('/camera_depth_frame', '/new_frame', rospy.Time(0), rospy.Duration(1.0))
                (trans, rot) = self.__listener.lookupTransform('/new_ref', '/head_origin', rospy.Time(0))
            except (tf.Exception):
                rospy.logwarn('tf.Exception in head node: Restart loop.')
                self.cleanup()
                rospy.sleep(0.1)
                continue
            #===================================================================
            # self.transformed_pose = self.__listener.transformPose('/new_ref', self.filtered_head_pose)
            #
            # quaternion = [self.transformed_pose.pose.orientation.x, self.transformed_pose.pose.orientation.y,
            #          self.transformed_pose.pose.orientation.z, self.transformed_pose.pose.orientation.w]
            # roll, pitch, self.rotation_filtered = tf.transformations.euler_from_quaternion(quaternion)
            #===================================================================
            roll, pitch, self.rotation_from_tf = tf.transformations.euler_from_quaternion(rot)
            # which of the results to use (filtered: self.rotation= self.rotation_filtered or not filtered)
            self.rotation = self.rotation_from_tf

            if (self.rotation > math.pi / 2.0):
                self.rotation = 0.0
            if (self.rotation < -math.pi / 2.0):
                self.rotation = 0.0
            # print self.rotation

            self.head_dir.angular.z = self.rotation
            if (math.fabs(self.rotation) <= self.zero_zone):
                self.head_vel.angular.z = 0.0
            else:
                if self.rotation > 0.0:
                    self.head_vel.angular.z = (self.angular_velocity * (self.rotation - self.zero_zone))
                else:
                    self.head_vel.angular.z = (self.angular_velocity * (self.rotation + self.zero_zone))

            if self.check_loop_exists():
                rospy.logerror("Abnormal loop detected in head control. Setting head_vel to 0")
                self.cleanup()
                continue
            self.head_dir.linear.x = 1  # It has to be different to 0 to inform the destination inference module that it is a valid command
            self.head_vel_pub.publish(self.head_vel)  # vel_command should be continuosly published
            self.head_dir_pub.publish(self.head_dir)  # dir_command should be continuosly published

            r.sleep()


    def check_loop_exists(self):
        flag = 0
        if self.first_time == 1:
            if self.rotation == self.last_rotation:
                self.init_timer()
                self.first_time = 0
            else:
                pass
        else:
            if self.rotation == self.last_rotation:
                flag = self.check_timer()
            else:
                self.first_time = 1
        self.last_rotation = deepcopy(self.rotation)

        return flag



    def init_timer(self):
        self.timer = rospy.Time.now()
    def check_timer(self):
        self.timer_duration = rospy.Time.now() - self.timer
        if self.timer_duration.secs >= self.time_threshold:
            self.timer_finished = 1
        else:
            self.timer_finished = 0
        return self.timer_finished


    def headCb(self, msg):
        self.filtered_head_pose = msg




    def cleanup(self):
        twist = Twist()
        self.head_vel_pub.publish(twist)
        self.head_dir_pub.publish(twist)


if __name__ == "__main__":
    rospy.init_node('head')
    try:
        rospy.logerr("init")
        HeadCmd()
    except:
        rospy.logfatal("head.py controller died")
        pass

