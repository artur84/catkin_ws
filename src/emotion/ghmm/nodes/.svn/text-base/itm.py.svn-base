#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os

BASE_DIR = os.path.abspath( os.path.join( os.path.dirname( __file__ ), ".." ) )
path     = os.path.abspath( os.path.join( BASE_DIR, "python" ) )
if os.path.exists( path ):
  sys.path.append( path )

from eitm import EITM
from graph import Graph
from kernels import mahalanobis

import roslib 
roslib.load_manifest( "ghmm" )
roslib.load_manifest( "trajectory_simulator" )
roslib.load_manifest( "visualization_msgs" )
import rospy

import numpy

from trajectory_simulator.msg import TrajectoryObservation
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

#-------------------------------------------------------------------------------

class Handler( object ):
  def __init__( 
    self, topic, frame_id, distance, insertion_distance, 
    epsilon, observation_interval, max_distance, max_height
  ):
    self.g = Graph()
    self.eitm = EITM( self.g, distance, insertion_distance, epsilon ) 
    self.publisher = rospy.Publisher( topic, MarkerArray )
    self.frame_id = frame_id
    self.observation_interval = observation_interval
    self.max_distance = max_distance
    self.max_height = max_height
    self.sequences = {}

  def __call__( self, o ):
    if o.type == TrajectoryObservation.FIRST:
      self.sequences[o.object_id] = []
    if o.object_id in self.sequences:
      s = self.sequences[o.object_id]
      s.append( o )
      if o.type == TrajectoryObservation.LAST:
        x1 = s[-1].pose.x
        y1 = s[-1].pose.y
        last_time = None
        for d in s:
          time = d.header.stamp.secs + d.header.stamp.nsecs / 1E9
          if last_time == None or time - last_time >= self.observation_interval: 
            self.eitm( [d.pose.x, d.pose.y, x1, y1] )
            last_time = time

        markers = MarkerArray()

        for n in self.g.nodes():
          markers.markers.append( self.new_node( n ) )
          lines   = self.new_lines( n )
          markers.markers.append( lines )
          for n1 in self.g.children( n ):
            self.new_edge( lines, n, n1 )
        self.publisher.publish( markers )
        del self.sequences[o.object_id]

  def new_node( self, node ):
    m = Marker()
    m.header.frame_id = self.frame_id
    m.header.stamp = rospy.get_rostime()
    m.ns = "ghmm"
    m.id = node.index
    m.type = Marker.SPHERE
    m.action = Marker.ADD
    m.pose.position.x = node.data[0]
    m.pose.position.y = node.data[1]
    m.pose.position.z = self.height( node )
    m.pose.orientation.x = 0
    m.pose.orientation.y = 0
    m.pose.orientation.z = 0
    m.pose.orientation.w = 1
    m.scale.x = 0.3
    m.scale.y = 0.3
    m.scale.z = 0.3
    m.color.a = 1.0
    m.color.r = 0.5
    m.color.g = abs( node.data[2] ) / self.max_distance
    m.color.b = abs( node.data[3] ) / self.max_distance
    return m

  def new_lines( self, node ):
    x = node.data[2]
    y = node.data[3]
    m = Marker()
    m.header.frame_id = self.frame_id
    m.header.stamp = rospy.get_rostime()
    m.ns = "ghmm"
    m.id = 30000 + node.index
    m.type = Marker.LINE_LIST
    m.action = Marker.ADD
    m.color.a = 1.0
    m.color.r = 0.5
    m.color.g = abs( x ) / self.max_distance
    m.color.b = abs( y ) / self.max_distance
    m.scale.x = 0.1
    return m

  def new_edge( self, lines, n1, n2 ):
    p1 = Point()
    p1.x = n1.data[0]
    p1.y = n1.data[1]
    p1.z = self.height( n1 )
    
    p2 = Point()
    p2.x = n2.data[0]
    p2.y = n2.data[1]
    p2.z = self.height( n2 )

    lines.points.append( p1 )
    lines.points.append( p2 )

  def height( self, node ):
    return self.max_height / self.max_distance * abs( node.data[2] * node.data[3] ) ** 0.5


#-------------------------------------------------------------------------------

if __name__ == "__main__":
  try:
    sigma = [
      [1., 0., 0., 0.],
      [0., 1., 0., 0.],
      [0., 0., 9., 0.],
      [0., 0., 0., 9.],
    ]
    handler = Handler( "itm", "/world", mahalanobis( sigma ), 1, 0.01, 0.3, 14., 3 )
    rospy.init_node( "itm" )
    rospy.Subscriber( "dynamic_objects", TrajectoryObservation , handler )
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
