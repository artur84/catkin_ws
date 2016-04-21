#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy

import tf
import turtlesim.msg

def handle_turtle_pose(msg, turtlename):
    br = tf.TransformBroadcaster()
    br.sendTransform((msg.x, msg.y, 0), 
                     tf.transformations.quaternion_from_euler(0, 0, msg.theta),
                     rospy.Time.now(), turtlename, "world")
    print 'I received a message:'
    print msg
    print 'turtlename:'
    print turtlename

if __name__ == '__main__':
    rospy.init_node('turtle_tf_broadcaster')
    #rospy.get_param('~nombre_parametro', 'valor por default')
    turtlename = rospy.get_param('~turtle', 'turtle1')
    print 'The node started'
    rospy.Subscriber('/%s/pose' % turtlename, turtlesim.msg.Pose, handle_turtle_pose, turtlename)
    rospy.spin()
    
