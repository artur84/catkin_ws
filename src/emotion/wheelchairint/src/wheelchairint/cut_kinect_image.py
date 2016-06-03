#!/usr/bin/env python

""" cut_kinect_image.py is a node that reads the rgb image published by the kinect camera 
used to drive the wheelchair and puts a black frame around a predefined Region of interest 
to avoid areas in the image that are not interesting because the face of the user will never be located there.      
"""

import roslib
import rospy
import cv2.cv as cv
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class CutKinectImage():
    def __init__( self ):
        """Parameters
        """
        self.ROI_x_offset = rospy.get_param("~ROI_x_offset", 10)   # origin of the ROI in the original image
        self.ROI_y_offset = rospy.get_param("~ROI_y_offset", 10)    #Upper corner (x0,y0)-------> x
                                                                #                |
                                                                #                |
                                                                #                V y
        self.ROI_height = rospy.get_param("~ROI_height", 240)     # The height of the ROI
        self.ROI_width = rospy.get_param("~ROI_width", 230)      # The width of the ROI
        self.cols = rospy.get_param("~cols", 640)   #Resolution of the output image (Should be equal to kinect's resolution)
        self.rows = rospy.get_param("~rows", 480)
        """ Publishers
        """
        self.out_img_pub = rospy.Publisher( 'out_img', Image, queue_size=10 )
        
        """ Subscribers
        """
        rospy.Subscriber( 'in_img', Image, self.in_img_callback )
        """ Constants
        """

        """Members 
        """
        self.bridge = CvBridge() # To transform between ROS and opencv.
        rospy.on_shutdown( self.cleanup )
        self.cv_out_img = cv.CreateImage((self.cols,self.rows), cv.IPL_DEPTH_8U, 3)
        self.ROI = (self.ROI_x_offset, self.ROI_y_offset, self.ROI_width, self.ROI_height)
        #cv.NamedWindow("Image window", 1)
        rospy.loginfo('cut_kinect_image node started')
        rospy.spin()
        
    
    """Kinect camera input image callback
    """
    def in_img_callback( self, msg ):
        try:
            cv_in_img = self.bridge.imgmsg_to_cv(msg, "bgr8")
        except CvBridgeError, e:
            print e
          
        (cols,rows) = cv.GetSize(cv_in_img)
        #thumbnail = cv.CreateMat(self.ROI_height , self.ROI_width, cv.CV_8UC3)
        
        """ Cut the image and past it in a larger one with a border
        """
        cv.CopyMakeBorder(cv.GetSubRect(cv_in_img, self.ROI), self.cv_out_img, (self.ROI_x_offset, self.ROI_y_offset),0,(0,0,0,0))
        #cv.ShowImage("Image window", self.cv_out_img)
        #cv.WaitKey(2)
    
        try:
            self.out_img_pub.publish(self.bridge.cv_to_imgmsg(self.cv_out_img, "bgr8"))
        except CvBridgeError, e:
            print e
          
               
    """ To cleanup memory before leaving the program
    """
    def cleanup( self ):
        rospy.loginfo("cleaning up cut_kinect_image node")
       # cv.DestroyAllWindows()
        

if __name__ == "__main__":
    rospy.init_node( 'cut_kinect_image' )
    try:
        CutKinectImage()
    except:
        rospy.logfatal("cut_kinect_image node died at start")
        pass

