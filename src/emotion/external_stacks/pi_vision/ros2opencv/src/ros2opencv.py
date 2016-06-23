#!/usr/bin/env python

""" ros2opencv.py - Version 0.2 2011-11-09

    A ROS-to-OpenCV node that uses cv_bridge to map a ROS image topic and optionally a ROS
    depth image topic to the equivalent OpenCV image stream(s).
    
    Includes variables and helper functions to store detection and tracking information and display
    markers on the image.
    
    Creates an ROI publisher to publish the region of interest on the /roi topic.
    
    Publishes a 3D PointStamped message on the /target_point topic.
    
    Created for the Pi Robot Project: http://www.pirobot.org
    Copyright (c) 2011 Patrick Goebel.  All rights reserved.

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.
    
    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details at:
    
    http://www.gnu.org/licenses/gpl.html
      
"""

import roslib
roslib.load_manifest('ros2opencv')
import rospy
import cv
import sys
from std_msgs.msg import String
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge, CvBridgeError
import time

class ROS2OpenCV:
    def __init__(self, node_name):
        rospy.init_node(node_name)
        
        rospy.on_shutdown(self.cleanup)
        
        self.node_name = node_name
        self.input_rgb_image = "input_rgb_image"
        self.input_depth_image = "input_depth_image"
        self.output_image = "output_image"
        self.show_text = rospy.get_param("~show_text", True)
        self.show_features = rospy.get_param("~show_features", True)

        """ Initialize the Region of Interest and its publisher """
        self.ROI = RegionOfInterest()
        self.pubROI = rospy.Publisher("/roi", RegionOfInterest)

        """ Do the same for the point cluster publisher """
        self.cluster3d = PointStamped()
        self.pub_cluster3d = rospy.Publisher("/target_point", PointStamped)
        
        """ Initialize a number of global variables """
        self.image = None
        self.image_size = None
        self.depth_image = None
        self.grey = None
        self.selected_point = None
        self.selection = None
        self.drag_start = None
        self.keystroke = None
        self.key_command = None
        self.detect_box = None
        self.track_box = None
        self.display_box = None
        self.keep_marker_history = False
        self.night_mode = False
        self.auto_face_tracking = True
        self.cps = 0 # Cycles per second = number of processing loops per second.
        self.cps_values = list()
        self.cps_n_values = 20
        self.flip_image = False

        """ Create the display window """
        self.cv_window_name = self.node_name
        cv.NamedWindow(self.cv_window_name, cv.CV_NORMAL)
        cv.ResizeWindow(self.cv_window_name, 640, 480)
        
        """ Create the cv_bridge object """
        self.bridge = CvBridge()
        
        """ Set a call back on mouse clicks on the image window """
        cv.SetMouseCallback (self.node_name, self.on_mouse_click, None)
        
        """ A publisher to output the display image back to a ROS topic """
        self.output_image_pub = rospy.Publisher(self.output_image, Image)
        
        """ Subscribe to the raw camera image topic and set the image processing callback """
        self.image_sub = rospy.Subscriber(self.input_rgb_image, Image, self.image_callback, queue_size=1)
        self.depth_sub = rospy.Subscriber(self.input_depth_image, Image, self.depth_callback, queue_size=1)
        
        rospy.loginfo("Starting " + self.node_name)
        
    def on_mouse_click(self, event, x, y, flags, param):
        """ We will usually use the mouse to select points to track or to draw a rectangle
            around a region of interest. """
        if not self.image:
            return
        
        if self.image.origin:
            y = self.image.height - y
            
        if event == cv.CV_EVENT_LBUTTONDOWN and not self.drag_start:
            self.detect_box = None
            self.selected_point = (x, y)
            self.drag_start = (x, y)
            
        if event == cv.CV_EVENT_LBUTTONUP:
            self.drag_start = None
            self.detect_box = self.selection
            
        if self.drag_start:
            xmin = max(0, min(x, self.drag_start[0]))
            ymin = max(0, min(y, self.drag_start[1]))
            xmax = min(self.image.width, max(x, self.drag_start[0]))
            ymax = min(self.image.height, max(y, self.drag_start[1]))
            self.selection = (xmin, ymin, xmax - xmin, ymax - ymin)
            
    def depth_callback(self, data):
        depth_image = self.convert_depth_image(data)
        
        if self.flip_image:    
            cv.Flip(depth_image)
            
        if not self.depth_image:
            (cols, rows) = cv.GetSize(depth_image)
            self.depth_image = cv.CreateMat(rows, cols, cv.CV_32FC1)
            
        cv.Copy(depth_image, self.depth_image)

    def image_callback(self, data):
        """ Time this loop to get cycles per second """
        start = rospy.Time.now()
        
        """ Convert the raw image to OpenCV format using the convert_image() helper function """
        cv_image = self.convert_image(data)
        
        """ Some webcams invert the image """
        if self.flip_image:
            cv.Flip(cv_image)
                    
        """ Create a few images we will use for display """
        if not self.image:
            self.image_size = cv.GetSize(cv_image)
            self.image = cv.CreateImage(self.image_size, 8, 3)
            self.marker_image = cv.CreateImage(self.image_size, 8, 3)
            self.display_image = cv.CreateImage(self.image_size, 8, 3)
            self.processed_image = cv.CreateImage(self.image_size, 8, 3)
            cv.Zero(self.marker_image)

        """ Copy the current frame to the global image in case we need it elsewhere"""
        cv.Copy(cv_image, self.image)
        
        if not self.keep_marker_history:
            cv.Zero(self.marker_image)
        
        """ Process the image to detect and track objects or features """
        processed_image = self.process_image(cv_image)
        
        """ If the result is a greyscale image, convert to 3-channel for display purposes """
        if processed_image.channels == 1:
            cv.CvtColor(processed_image, self.processed_image, cv.CV_GRAY2BGR)
        else:
            cv.Copy(processed_image, self.processed_image)
        
        """ Display the user-selection rectangle or point."""
        self.display_markers()
        
        if self.night_mode:
            """ Night mode: only display the markers """
            cv.SetZero(self.processed_image)
        
        """ Merge the processed image and the marker image """
        cv.Or(self.processed_image, self.marker_image, self.display_image)
        
        if self.track_box:
            if self.auto_face_tracking:
                cv.EllipseBox(self.display_image, self.track_box, cv.CV_RGB(255, 0, 0), 2)
            else:
                (center, size, angle) = self.track_box
                pt1 = (int(center[0] - size[0] / 2), int(center[1] - size[1] / 2))
                pt2 = (int(center[0] + size[0] / 2), int(center[1] + size[1] / 2))

                cv.Rectangle(self.display_image, pt1, pt2, cv.RGB(255, 0, 0), 2, 8, 0)
            
        elif self.detect_box:
            (pt1_x, pt1_y, w, h) = self.detect_box
            cv.Rectangle(self.display_image, (pt1_x, pt1_y), (pt1_x + w, pt1_y + h), cv.RGB(255, 0, 0), 2, 8, 0)
        
        """ Handle keyboard events """
        self.keystroke = cv.WaitKey(5)
            
        duration = rospy.Time.now() - start
        duration = duration.to_sec()
        fps = int(1.0 / duration)
        self.cps_values.append(fps)
        if len(self.cps_values) > self.cps_n_values:
            self.cps_values.pop(0)
        self.cps = int(sum(self.cps_values) / len(self.cps_values))
        
        if self.show_text:
            hscale = 0.2 * self.image_size[0] / 160. + 0.1
            vscale = 0.2 * self.image_size[1] / 120. + 0.1
            text_font = cv.InitFont(cv.CV_FONT_VECTOR0, hscale, vscale, 0, 1, 8)
            """ Print cycles per second (CPS) and resolution (RES) at top of the image """
            if self.image_size[0] >= 640:
                vstart = 25
                voffset = int(50 + self.image_size[1] / 120.)
            elif self.image_size[0] == 320:
                vstart = 15
                voffset = int(35 + self.image_size[1] / 120.)
            else:
                vstart = 10
                voffset = int(20 + self.image_size[1] / 120.)
            cv.PutText(self.display_image, "CPS: " + str(self.cps), (10, vstart), text_font, cv.RGB(255, 255, 0))
            cv.PutText(self.display_image, "RES: " + str(self.image_size[0]) + "X" + str(self.image_size[1]), (10, voffset), text_font, cv.RGB(255, 255, 0))
        # Now display the image.
        cv.ShowImage(self.node_name, self.display_image)
        
        """ Publish the display image back to ROS """
        try:
            self.output_image_pub.publish(self.bridge.cv_to_imgmsg(self.display_image, "bgr8"))
        except CvBridgeError, e:
            print e
        
        """ Process any keyboard commands or command sent via the key_command service """
        if self.key_command:
            self.keystroke = ord(self.key_command)
            self.key_command = None
        if 32 <= self.keystroke and self.keystroke < 128:
            cc = chr(self.keystroke).lower()
            if cc == 'c':
                self.features = []
                self.track_box = None
                self.detect_box = None
            elif cc == 'n':
                self.night_mode = not self.night_mode
            elif cc == 'f':
                self.show_features = not self.show_features
            elif cc == 't':
                self.show_text = not self.show_text
            elif cc == 'a':
                self.auto_face_tracking = not self.auto_face_tracking
                if self.auto_face_tracking:
                    self.features = []
                    self.track_box = None
                    self.detect_box = None
            elif cc == 'q':
                """ user has press the q key, so exit """
                rospy.signal_shutdown("User hit q key to quit.")      

          
    def convert_image(self, ros_image):
        try:
            cv_image = self.bridge.imgmsg_to_cv(ros_image, "bgr8")
            return cv_image
        except CvBridgeError, e:
          print e
          
    def convert_depth_image(self, ros_image):
        try:
            depth_image = self.bridge.imgmsg_to_cv(ros_image, "32FC1")
            return depth_image
    
        except CvBridgeError, e:
            print e
          
    def process_image(self, cv_image):
        if not self.grey:
            """ Allocate temporary images """      
            self.grey = cv.CreateImage(self.image_size, 8, 1)
            
        """ Convert color input image to grayscale """
        cv.CvtColor(cv_image, self.grey, cv.CV_BGR2GRAY)
        cv.EqualizeHist(self.grey, self.grey)
        
        # Since we aren't applying any filters in this base class, set the ROI to the selected region, if any.
        if not self.drag_start and not self.detect_box is None:         
            self.ROI = RegionOfInterest()
            self.ROI.x_offset = self.detect_box[0]
            self.ROI.y_offset = self.detect_box[1]
            self.ROI.width = self.detect_box[2]
            self.ROI.height = self.detect_box[3]
            
        self.pubROI.publish(self.ROI)
        
        return self.grey
    
    def display_markers(self):
        # If the user is selecting a region with the mouse, display the corresponding rectangle for feedback.
        if self.drag_start and self.is_rect_nonzero(self.selection):
            x,y,w,h = self.selection
            cv.Rectangle(self.marker_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
            self.selected_point = None

        # Else if the user has clicked on a point on the image, display it as a small circle.            
        elif not self.selected_point is None:
            x = self.selected_point[0]
            y = self.selected_point[1]
            cv.Circle(self.marker_image, (x, y), 3, (0, 255, 255), 2)
        
    def is_rect_nonzero(self, r):
        # First assume a simple CvRect type
        try:
            (_,_,w,h) = r
            return (w > 0) and (h > 0)
        except:
            try:
                # Otherwise, assume a CvBox2D type
                ((_,_),(w,h),a) = r
                return (w > 0) and (h > 0)
            except:
                return False
        
    def cleanup(self):
        print "Shutting down vision node."
        cv.DestroyAllWindows()       

def main(args):
    # Display a help message if appropriate.
    help_message = ""
          
    print help_message

    try:   
        # Fire up the node.
        ROS2OpenCV("ros2opencv")
        # Spin so our services will work
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down vision node."
        cv.DestroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)
