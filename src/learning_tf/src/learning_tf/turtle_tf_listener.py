#!/usr/bin/env python  
import roslib
roslib.load_manifest('learning_tf')
import rospy
import math
import tf
import geometry_msgs.msg
import turtlesim.srv

if __name__ == '__main__':
    rospy.init_node('tf_turtle')
    
    #Crea la estructura para escuchar transformaciones
    listener = tf.TransformListener()
    
    # Crea una tortuga nueva en el simulador (turtle2)
    # Recordar cuando usamos rosservice call spawn 2 2 0.2 ""
    rospy.wait_for_service('spawn')
    spawner = rospy.ServiceProxy('spawn', turtlesim.srv.Spawn)
    spawner(4, 2, 0, 'turtle2')
    
    # Crea un publicador de velocidad para la tortuga 2 (Mover turtle2)
    turtle_vel = rospy.Publisher('turtle2/cmd_vel', geometry_msgs.msg.Twist,queue_size=1)
    
    # Fija la frecuencia a la cual va a publicar turtle_vel
    rate = rospy.Rate(10.0)
    while not rospy.is_shutdown():
        try:
            #Obtiene la traslacion y rotacion de /turtle2 con respecto a /turtle1
            (trans,rot) = listener.lookupTransform('/turtle2', '/carrot1', rospy.Time(0))
            #rospy.Time(0) ultima transformacion enviada.
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        #Controla la velocidad de turtle2 proporcionalmente a la distancia lineal y angular
        angular = 0.0040 * math.atan2(trans[1], trans[0])
        linear = 0.005 * math.sqrt(trans[0] ** 2 + trans[1] ** 2)
        cmd = geometry_msgs.msg.Twist()
        cmd.linear.x = linear
        cmd.angular.z = angular
        turtle_vel.publish(cmd)

        rate.sleep()
