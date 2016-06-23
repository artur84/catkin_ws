#! /usr/bin/env python
#
import roslib; roslib.load_manifest( 'user_intentions' )
import cv
import rospy
from move_base_msgs.msg import MoveBaseGoal

from move_base_msgs.msg import MoveBaseAction
from geometry_msgs.msg._PoseWithCovarianceStamped import PoseWithCovarianceStamped
from geometry_msgs.msg._PoseStamped import PoseStamped
from geometry_msgs.msg._Twist import Twist
from std_msgs.msg import String
import numpy as np
import actionlib
from actionlib_msgs.msg._GoalStatus import GoalStatus
#=============================================================================
#==============================================================================
class UIWindow():
    """
    Class for handling the computing of current computed goal
    """
    
    def __init__( self ):
        """
        Constructor.
        """
        rospy.init_node('UIWindow')
    
        """ Getting ROS parameters
        """
    
        """ Pseudo constants 
        """

        """Members 
        """     

        """ Subscribers
        """

        
        """ Publishers
        """

        
        """Creating Move base client
        """
        print "creating the window"
        cv.NamedWindow("ui_kinect_image", cv.CV_WINDOW_AUTOSIZE)
        cv.SetMouseCallback("ui_kinect_image", self.onmouse)
        import cv, numpy
        
        """Create a simple matrix to display """
        a = numpy.ones((480, 640))
        mat = cv.fromarray(a)
        print mat.rows
        print mat.cols
        cv.ShowImage("ui_kinect_image", mat)
        
        def onmouse(event, x, y, flags, param):
            if event == cv.EVENT_LBUTTONDOWN:
                drag_start = x, y
                sel = 0,0,0,0
            elif event == cv.EVENT_LBUTTONUP:
                if sel[2] > sel[0] and sel[3] > sel[1]:
                    print "sel[0]=" + sel[0]
                    print "sel[1]=" + sel[1]
                    print "sel[2]=" + sel[2]
                    print "sel[3]=" + sel[3]
                    drag_start = None
                elif drag_start:
                    #print flags
                    if flags & cv.EVENT_FLAG_LBUTTON:
                        minpos = min(drag_start[0], x), min(drag_start[1], y)
                        maxpos = max(drag_start[0], x), max(drag_start[1], y)
                        sel = minpos[0], minpos[1], maxpos[0], maxpos[1]
                        print "Dragging"
            else:
                print "selection is complete"
                drag_start = None
   
            

#===============================================================================
if __name__ == '__main__':
    """ Initializing
    """
    window = UIWindow()
        
    """Start
    """
    while not rospy.is_shutdown():                
        rospy.sleep( -1 )



