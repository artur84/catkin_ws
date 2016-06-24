#! /usr/bin/env python
import roslib
from trajectory_simulator.msg import TrajectoryObservation
import numpy as np
import rospy
import cv2.cv as cv
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg._PoseStamped import PoseStamped
import tf
from copy import deepcopy
from copy import copy
from rospy.numpy_msg import numpy_msg
from user_intentions._Interest_grid import InterestGrid
from nav_msgs.msg._OccupancyGrid import OccupancyGrid
from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray

#==============================================================================
#==============================================================================
class ROSInterestGridHandler:
    """This class creates an histogram of how often some place is in the field of view of the user.

        @author: Jesus Arturo Escobedo Cabello
        @contact: jesus.escobedo-cabello@inria.fr
        @organization: INRIA Grenoble, Emotion-team
    """
    def __init__(self):
        """ Inits ROS interfaces as publishers, subscribers etc.

            More details:
        """

        rospy.on_shutdown(self.cleanup)
        """ Getting ROS parameters
        """
        self.height = rospy.get_param('~height', 250)  # Number of cells over 'x' axe in the grid
        self.width = rospy.get_param('~width', 250)  # Number of cells over 'y' direction in the grid
        self.frame_id = rospy.get_param('~frame_id', 'map')  # The grid will be displayed with respect to this frame.
        self.tf_prefix = rospy.get_param('tf_prefix', '')
        if self.tf_prefix is not '':
            self.tf_prefix = "/" + self.tf_prefix
        self.__tf_listener = tf.TransformListener()  # NOTE THIS: the listener should always be declared in the class
        self.__tf_br__ = tf.TransformBroadcaster()  # To broadcast the new frame
        """ Members
        """
        self.interest_grid = InterestGrid()
        self.bridge = CvBridge()  # To transform between ROS and opencv.
        self.head_pose = PoseStamped()
        self.ros_map = OccupancyGrid()
        self.ros_map_received = False  # To indicate if a map has been already received
        self.joy_input = Odometry()  # The input from the joystick will be read as an odometry message
        self.ros_img = Image()
        self.grid_marker_array = MarkerArray()
        self.ghmm_obs = TrajectoryObservation()
        self.whc_trans = (0, 0, 0)
        """ ROS Publishers
        """
        self.ros_grid_pub = rospy.Publisher('grid', OccupancyGrid, queue_size=1)
        self.ros_img_pub = rospy.Publisher('ros_img', Image, queue_size=1)
        self.grid_marker_array_pub = rospy.Publisher('grid_marker_array', MarkerArray, queue_size=1)
        self.ghmm_obs_pub = rospy.Publisher('dynamic_objects', TrajectoryObservation, queue_size=1)
        """ ROS Subscribers
        """
        rospy.Subscriber('head_pose_filtered', PoseStamped, self.headPoseCb)
        rospy.Subscriber('joy_odometry', Odometry, self.joyCb)
        rospy.Subscriber('map', OccupancyGrid, self.rosMapCb)

        r = rospy.Rate(10.0)
        """ Waits for the map in order to create a grid of the same dimensions """
        print "waiting for map"
        while not rospy.is_shutdown():
            if self.ros_map_received:
                self.init_interest_grid()
                break
            r.sleep()
        print "map was received"
        """Main loop"""
        visualize_time = 0  # To control when to display the markers so that it's not so expensive.
        while not rospy.is_shutdown():
            try:
                """Get the global position of the wheelchair and the user's head"""
                (whc_trans, whc_quat) = self.__tf_listener.lookupTransform("/map", "/wheelchair/base_link", rospy.Time(0))
                (head_trans, head_quat) = self.__tf_listener.lookupTransform("/map", "/head_origin", rospy.Time(0))
            except (tf.LookupException, tf.ConnectivityException, tf.Exception, tf.ExtrapolationException):
                rospy.logerr('TF error in interest_grid')
                rospy.sleep(0.5)
                continue
            # self.set_head_global_pose_params()
            """Publish dynamic object frame"""
            self.whc_trans = deepcopy(whc_trans)
            self.whc_angles = tf.transformations.euler_from_quaternion(deepcopy(whc_quat))
            self.set_trajectory_type(whc_trans[0], whc_trans[1])
            self.publish_trajectory_point()
            self.__tf_br__.sendTransform((0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0), rospy.Time.now(), self.dyn_obj_frame, "/wheelchair/base_link")
            (head_roll, head_pitch, head_yaw) = tf.transformations.euler_from_quaternion(head_quat)
            # print "head_yaw: ", head_yaw, "head_pose: ", head_trans
            # self.interest_grid.set_line(head_trans[0], head_trans[1], head_yaw, 80, 1)
            # self.interest_grid.set_scaled_line(head_trans[0], head_trans[1], head_yaw, head_pitch, 1)
            self.interest_grid.set_point(whc_trans[0], whc_trans[1], 3)
            print "here"
            """Visualization in rviz"""
            if visualize_time == 30:
                visualize_time = 0
                """Get corners"""
                self.interest_grid.set_typical_destinations()
                corners = self.interest_grid.get_corners_img()
                tmp = np.asarray(corners)
                self.fill_cell_marker_array(tmp)
                self.grid_marker_array_pub.publish(self.grid_marker_array)
            visualize_time += 1
            """Visualization as ROS image"""
            # self.ros_img_pub.publish(self.bridge.cv_to_imgmsg(cv.fromarray(tmp.astype(np.uint8))))
            r.sleep()

    def set_trajectory_type(self, dyn_obj_pose_x, dyn_obj_pose_y):
        """
        Starts a new trajectory to be sent to ghmm whenever the wheelchair passes near the precomputed
        typicall destinations

        """
        typical_destinations = deepcopy(self.interest_grid.typical_dest_list)
        x = dyn_obj_pose_x
        y = dyn_obj_pose_y
        r = 1.0  # radius min to one typical goal to reset the trajectory
        print "dyn_obj_pose_x: ", x, "dyn_obj_pose_y: ", y
        is_inside = False
        for pose in typical_destinations:
            d = np.sqrt(np.square(x - pose[0]) + np.square(y - pose[1]))
            if d <= r:
                print "d= ", d
                is_inside = True
        r = rospy.Rate(1.0)
        if is_inside:
            if self.ghmm_obs.type == 0:
                self.ghmm_obs.type = self.ghmm_obs.LAST
            else:
                if self.ghmm_obs.type == self.ghmm_obs.LAST:
                    self.ghmm_obs.type = self.ghmm_obs.FIRST
                    """We only icrease index (id of the current trajectory) when starting a new one"""
                    self.ghmm_obs.object_id = self.ghmm_obs.object_id + 1
            r.sleep()
        else:
            self.ghmm_obs.type = 0
        print "index: ", self.ghmm_obs.object_id
        print "type: ", self.ghmm_obs.type

    def publish_trajectory_point(self):
        """ I had several problems to get working this listener.waitForTransform one of the main problems was that I didn't use the
        first call self.__tf_listener__.waitForTransform before the loop, then in many ROS tutorials they use  rospy.Time() where rospy.Time.now() is
        necessary. Sometimes COPY-PASTE is really bad :)
        """
        self.dyn_obj_frame = "/dynamic_object_pose_%s" % self.ghmm_obs.object_id
        print self.dyn_obj_frame
        self.ghmm_obs.pose.x = self.whc_trans[0]
        self.ghmm_obs.pose.y = self.whc_trans[1]
        self.ghmm_obs.pose.theta = self.whc_angles[2]
        self.ghmm_obs.header.stamp = rospy.Time.now()
        self.ghmm_obs.header.frame_id = "1"  # It was like that in messages sent by trajectory_simulator
        self.ghmm_obs_pub.publish(self.ghmm_obs)


    def init_interest_grid(self):
        if self.ros_map_received:
            resolution = 0.5  # our grid can have a different resolution from that of the ros_map
            height = (self.ros_map.info.height) * self.ros_map.info.resolution + resolution
            width = (self.ros_map.info.width) * self.ros_map.info.resolution + resolution
            x_origin = self.ros_map.info.origin.position.x
            y_origin = self.ros_map.info.origin.position.y
            self.interest_grid = InterestGrid(width , height, (x_origin, y_origin), resolution)
            print "interest grid was initialized"
            print "height: ", height, " width: ", width, " origin: ", (x_origin, y_origin), " resolution: ", resolution
        else:
            rospy.logerr("The map has not been received. Interest grid cannot be initialized")

    def set_head_global_pose_params(self):
        """
            Sets the necessary values to draw the line that goes out from the head's position in it's direction.
        """
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion([temp_quat.x, temp_quat.y, temp_quat.z, temp_quat.w])
        head_x0 = self.head_pose.pose.position.x  # Current position of the user's face in the map
        head_y0 = self.head_pose.pose.position.y  # Current position of the user's face in the map
        head_yaw = yaw
        head_roll = roll
        head_pitch = pitch

    def transform_to_map(self):
        pass

    def draw_joyline(self):
        """ Draw the line in the direction of the joystick
        """
        temp_quat = self.joystick_odom.pose.pose.orientation
        x0 = self.head_pose.pose.position.x
        y0 = self.head_pose.pose.position.y
        (pitch, roll, yaw) = tf.transformations.euler_from_quaternion([temp_quat.x, temp_quat.y, temp_quat.z, temp_quat.w])

    def ocv_to_grid(self):
        """ Transform an opencv image to a ros grid
        """
        try:
            self.ros_img = self.bridge.cv_to_imgmsg(self.interest_grid.grid, "mono8")
        except CvBridgeError, e:
            print e
        """Transform from cv to numpy array"""
        a = np.asmatrix(self.cv_observation_map)
        """Fill OccGrid metadata """
        self.ros_grid = OccupancyGrid()  # Clean the Occupancy grid
        self.ros_grid.info.height = self.height
        self.ros_grid.info.width = self.width
        self.ros_grid.info.resolution = self.resolution
        self.ros_grid.header.frame_id = self.frame_id
        self.ros_grid.header.stamp = rospy.Time.now()
        """ Fill OccGrid orientation """
        self.ros_grid.info.origin.orientation.x = tf.transformations.quaternion_from_euler(0, 0, 0, 'sxyz')[0]
        self.ros_grid.info.origin.orientation.y = tf.transformations.quaternion_from_euler(0, 0, 0, 'sxyz')[1]
        self.ros_grid.info.origin.orientation.z = tf.transformations.quaternion_from_euler(0, 0, 0, 'sxyz')[2]
        self.ros_grid.info.origin.orientation.w = tf.transformations.quaternion_from_euler(0, 0, 0, 'sxyz')[3]
        """ Fill OccGrid position """
        self.ros_grid.info.origin.position.x = 0  # where the upper left corner of the image will be located referent to the frame
        self.ros_grid.info.origin.position.y = 0
        # self.ros_grid.info.origin.position.y = -(self.width/2)*self.resolution #Te frame will be at the middle of the Grid in y coordinate.
        for r in range(np.shape(a)[0]):
            for c in range(np.shape(a)[1]):
                self.ros_grid.data.append(copy(a[r, c]))

    def headPoseCb(self, msg):
        self.head_pose = msg

    def joyCb(self, msg):
        self.joy_input = msg

    def rosMapCb(self, msg):
        self.ros_map_received = True
        self.ros_map = msg

    def fill_cell_marker_array(self, numpy_array_grid):
        """Fill a marker array to display the grid in rviz.

        """
        self.grid_marker_array = MarkerArray()
        marker = Marker()
        resolution = self.interest_grid.resolution
        a = numpy_array_grid
        marker.header.frame_id = '/map'  # it should always be "/map" I had some troubles with rviz if i try to use with respect to any other frame
        marker.header.stamp = rospy.Time.now()
        marker.type = Marker.CUBE
        marker.scale.x = resolution
        marker.scale.y = resolution

          # The z dimension will be much smaller than x and y dimensions
        """Color"""

        marker.color.g = 0.0
        """Maybe I can erase this"""
        marker.pose.orientation.x = 0
        marker.pose.orientation.y = 0
        marker.pose.orientation.z = 0
        marker.pose.orientation.w = 1
        count = 0
        maximo = 1.0 * np.max(a) + 0.1
        offset = resolution / 2.0
        for r in range(np.shape(a)[0]):
            for c in range(np.shape(a)[1]):
                marker.id = count
                (xmap, ymap) = self.interest_grid.tfocv2map(c, r)
                marker.pose.position.x = copy(xmap) + offset
                marker.pose.position.y = copy(ymap) - offset  # the resolution/2 is to make the square to be consistent with the ros map
                marker.scale.z = 10.0 * a[r, c] / maximo
                marker.pose.position.z = marker.scale.z / 2
                marker.color.r = a[r, c] / maximo
                marker.color.b = 1.0 - marker.color.r
                marker.color.a = a[r, c] / maximo + 0.3
                self.grid_marker_array.markers.append(deepcopy(marker))
                count += 1

    def cleanup(self):
        """ To cleanup memory before leaving the program
        """
        cv.Save("interest_grid.xml", self.interest_grid.grid)
        cv.Save("interesting_places_img.xml", self.interest_grid.get_corners_img())
        rospy.loginfo("cleaning up cut_kinect_image node")





if __name__ == '__main__':
    rospy.init_node('interest_grid')  # Is inportant to put this line here and not in the class if you want the rospy.on_sutdown command to work properly
    try:
        ROSInterestGridHandler()
    except:
        rospy.logfatal("ROSInterestGridHandler died")
        pass










