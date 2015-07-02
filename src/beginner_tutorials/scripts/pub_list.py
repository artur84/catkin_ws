#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String #Change this line 
from geometry_msgs.msg import Twist
from turtlesim.msg import Color

def callback(my_color):
    print my_color

def talker_listener():
    #Define ROS publisers and listeners 
    pub = rospy.Publisher('turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber("turtle1/color_sensor", Color, callback)

    
    my_vel = Twist()
    my_vel.linear.x = 10
    my_vel.angular.z = 2
    
    rospy.init_node('talker_listener', anonymous=True)
    rate = rospy.Rate(0.1) # 10hz
    while not rospy.is_shutdown():
        hello_str = "hello Im a publisher and listener"
        rospy.loginfo(hello_str)
        pub.publish(my_vel)
        rate.sleep()

if __name__ == '__main__':
    try:
        talker_listener()
    except rospy.ROSInterruptException:
        pass
