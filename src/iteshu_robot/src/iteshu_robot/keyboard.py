#!/usr/bin/env python
import roslib;
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from copy import deepcopy
from visualization_msgs.msg._Marker import Marker
from visualization_msgs.msg._MarkerArray import MarkerArray
import tf
import sys, select, termios, tty

msg = """
Reading from the keyboard  and Publishing to Twist!
---------------------------
Moving around:
   u    i    o
   j    k    l
   m    ,    .

+/- : increase/decrease max speeds by 10%

anything else : stop

CTRL-C to quit
"""

moveBindings = {
    'i':( 1,  0, 0),
    'o':( 1, -1, -0.75),
    'l':( 0, -1, -1.57),
    '.':(-1,  1, -2.32),
    ',':(-1,  0, 3.14),
    'm':(-1,  1, 2.32),
    'j':( 0,  1, 1.57),
    'u':( 1,  1, 0.75),
    'k':( 0,  0, 0 ),
       }

dirBindings = {  # for any other key it will send (-1,0)
    'e':(1, 0),
    'r':(1, -0.75),
    's':(1, 1.57),
    'f':(1, -1.57),
    'c':(1, 3.14),
    'w':(1, 0.75),
    'v':(1, -2.32),
    'x':(1, 2.32),

       }

speedBindings = {#To change the speed
    '+':(1.1, 1.1),
    '-':(.9, .9),
      }

voiceCommandBindings = {
    'z':"face",
    'h':"brake",
    't':"turn",
    'g':"go",
    'b':"back",
    'a':"autonomous",
      }



def fill_text_marker(text, tf_prefix):
    marker = Marker()
    marker.header.frame_id = tf_prefix + '/base_link'
    marker.header.stamp = rospy.Time.now()
    marker.type = Marker.TEXT_VIEW_FACING
    marker.text = text
    marker.scale.z = 1.0
    marker.scale.y = 1.0
    marker.scale.x = 1.0
    marker.lifetime = rospy.Duration.from_sec(1.0)
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.pose.position.x = 1.0
    marker.pose.position.y = 1.0
    marker.pose.position.z = 1.0
    return marker


def fill_command_marker(command, theta, tf_prefix):
    """
    command: Twist(), I contains the angular and linear speed of the command that we want to plot in rviz
    tf_prefix: String(), The namespace to be added to the name of the base_link, ex. robot_0 or wheelchair
    """
    marker = Marker()
    orientation = tf.transformations.quaternion_from_euler(0, 0, theta)
    marker.header.frame_id = tf_prefix + '/base_link'
    marker.header.stamp = rospy.Time.now()
    # For zero command, display a ball
    if  command.linear.x == 0 and command.angular.z == 0:
        marker.type = Marker.SPHERE
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
#    elif command.linear.x == -1: #not valid key will not be displayed
#        marker.type = Marker.SPHERE
#        marker.scale.x = 0.01
#        marker.scale.y = 0.01
#        marker.scale.z = 0.01
    # For non zero command we display arrows
    else:
        marker.type = Marker.ARROW
        marker.scale.x = 2.0
        marker.scale.y = 0.3
        marker.scale.z = 0.3
    marker.pose.orientation.x = orientation[0]
    marker.pose.orientation.y = orientation[1]
    marker.pose.orientation.z = orientation[2]
    marker.pose.orientation.w = orientation[3]
    marker.lifetime = rospy.Duration.from_sec(0.4)
    marker.color.r = 1.0
    marker.color.g = 0.0
    marker.color.a = 1.0
    marker.pose.position.x = 0.0
    marker.pose.position.y = 0.0
    marker.pose.position.z = 0.0
    return marker

def getKey():
    tty.setraw(sys.stdin.fileno())
    select.select([sys.stdin], [], [], 0)
    key = sys.stdin.read(1)
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

speed = 0.1
turn = 0.4

def vels(speed, turn):
    return "currently:\tspeed %s\tturn %s " % (speed, turn)

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('teleop_twist_keyboard')
    """ ROS Parameters
    """
    tf_prefix = rospy.get_param('tf_prefix', '')  # The prefix to be added to the links when calling ros tf.
    if tf_prefix != '':
        tf_prefix = "/"+tf_prefix

    vel_pub = rospy.Publisher('key_vel', Twist, queue_size=10)
    key_pub = rospy.Publisher('key_dir', Twist, queue_size=10)
    voice_command_pub = rospy.Publisher('recognizer/output', String, queue_size=10)
    dir_marker_pub = rospy.Publisher("keyboard_marker", Marker, queue_size=10)
    text_marker_pub = rospy.Publisher('keyboard_text', Marker, queue_size=10)
    x = 0
    th = 0
    status = 0
    try:
        print msg
        print vels(speed, turn)
        key_vel = Twist()
        key_dir = Twist()
        while(1):
            key = getKey()
            

            if key in moveBindings.keys():
                x = moveBindings[key][0]
                th = moveBindings[key][1]
                key_vel.linear.x = x * speed; key_vel.linear.y = 0; key_vel.linear.z = 0
                key_vel.angular.x = 0; key_vel.angular.y = 0; key_vel.angular.z = th * turn
                vel_pub.publish(key_vel)
                print key_vel
                # Printing a marker
                marker = fill_command_marker(key_vel, moveBindings[key][2],  tf_prefix)
                dir_marker_pub.publish(marker)

            elif key in dirBindings.keys():
                dir_x = dirBindings[key][0]
                dir_th = dirBindings[key][1]
                key_dir.linear.x = dir_x; key_dir.linear.y = 0; key_dir.linear.z = 0
                key_dir.angular.x = 0; key_dir.angular.y = 0; key_dir.angular.z = dir_th
                key_pub.publish(key_dir)
            elif key in speedBindings.keys():
                speed = speed * speedBindings[key][0]
                turn = turn * speedBindings[key][1]

                print vels(speed, turn)
                if (status == 14):
                    print msg
                status = (status + 1) % 15
            elif key in voiceCommandBindings.keys():
                if key == 'b':
                    x = 0; th = 0
                    dir_x = 1; dir_th = 3.14
                    x = -1; th = 0
                    key_vel.linear.x = x * speed; key_vel.linear.y = 0; key_vel.linear.z = 0
                    key_vel.angular.x = 0; key_vel.angular.y = 0; key_vel.angular.z = th * turn
                    vel_pub.publish(key_vel)
                    print key_vel
                    key_dir.linear.x = dir_x; key_dir.linear.y = 0; key_dir.linear.z = 0
                    key_dir.angular.x = 0; key_dir.angular.y = 0; key_dir.angular.z = dir_th
                    key_pub.publish(key_dir)
                voice_command_pub.publish(voiceCommandBindings[key])
                marker = fill_text_marker(voiceCommandBindings[key], tf_prefix)
                text_marker_pub.publish(marker)
            else:
                x = 0; th = 0
                dir_x = -1; dir_th = 0 #dir_x=-1 means this is not a valid key to give directions
                if (key == '\x03'):
                    break



    except:
        rospy.loginfo("keyboard.py: exception")

    finally:
        key_vel = Twist()
        key_dir = Twist()
        key_vel.linear.x = 0; key_vel.linear.y = 0; key_vel.linear.z = 0
        key_vel.angular.x = 0; key_vel.angular.y = 0; key_vel.angular.z = 0
        vel_pub.publish(key_vel)
        print key_vel

        key_dir.linear.x = 0; key_dir.linear.y = 0; key_dir.linear.z = 0
        key_dir.angular.x = 0; key_dir.angular.y = 0; key_dir.angular.z = 0
        key_pub.publish(key_dir)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
