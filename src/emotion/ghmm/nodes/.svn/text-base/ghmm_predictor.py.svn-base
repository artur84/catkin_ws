#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os

BASE_DIR = os.path.abspath( os.path.join( os.path.dirname( __file__ ), ".." ) )
path     = os.path.abspath( os.path.join( BASE_DIR, "python" ) )
if os.path.exists( path ):
  sys.path.append( path )

from ghmm import GHMM

import roslib 
roslib.load_manifest( "ghmm" )
roslib.load_manifest( "trajectory_simulator" )
roslib.load_manifest( "nav_msgs" )
import rospy

import numpy

from trajectory_simulator.msg import TrajectoryObservation
from nav_msgs.msg import OccupancyGrid

#-------------------------------------------------------------------------------

class Handler( object ):
  def __init__( 
    self, topic, frame_id, sigma, insertion_distance, 
    epsilon, state_prior, transition_prior, observation_interval,
    x1, y1, x2, y2, cell_size
  ):
    self.ghmm = GHMM( 
      sigma, insertion_distance, epsilon, state_prior, transition_prior 
    )
    self.publisher = rospy.Publisher( topic, OccupancyGrid )
    self.frame_id = frame_id
    self.observation_interval = observation_interval
    self.x1 = x1
    self.y1 = y1
    self.x2 = x2
    self.y2 = y2
    self.cell_size = cell_size
    self.sequences = {}
    self.tracks = {}
    self.times = {}

  def __call__( self, o ):
    time = o.header.stamp.secs + o.header.stamp.nsecs / 1E9
    if o.type == TrajectoryObservation.FIRST:
      print "new"
      self.sequences[o.object_id] = []
      self.tracks[o.object_id] = self.ghmm.reset()
    if o.object_id in self.sequences:
      if (    not o.object_id in self.times
           or time - self.times[o.object_id] >= self.observation_interval
           or o.type == TrajectoryObservation.LAST
      ) :
        self.times[o.object_id] = time
        track = self.tracks[o.object_id]
        s = self.sequences[o.object_id]
        s.append( o )
        if o.type == TrajectoryObservation.LAST:
          print "last"
          x1 = s[-1].pose.x
          y1 = s[-1].pose.y
          last_time = None
          sequence = []
          for d in s:
            sequence.append( [d.pose.x, d.pose.y, x1, y1] )
            last_time = time
          del self.times[o.object_id]
          del self.sequences[o.object_id]
          del self.tracks[o.object_id]
          self.ghmm.learn( sequence )
          print "learned"
        else:
          self.ghmm.update( track, [o.pose.x, o.pose.y] )

          grid = OccupancyGrid()
          grid.header.stamp = rospy.get_rostime()
          grid.header.frame_id = self.frame_id
          grid.info.map_load_time = rospy.get_rostime()
          grid.info.resolution = self.cell_size
          grid.info.width  = int( ( self.x2 - self.x1 ) / self.cell_size )
          grid.info.height = int( ( self.y2 - self.y1 ) / self.cell_size )
          grid.info.origin.position.x = self.x1
          grid.info.origin.position.y = self.y1
          grid.info.origin.position.z = 0
          grid.info.origin.orientation.x = 0
          grid.info.origin.orientation.y = 0
          grid.info.origin.orientation.z = 0
          grid.info.origin.orientation.w = 1
          grid.data = [0] * ( grid.info.width * grid.info.height )
          
          proba = [0.0] * len( grid.data )

          x = self.x1 + self.cell_size / 2.0
          total = 0
          for i in xrange( grid.info.width ):
            y = self.y1 + self.cell_size / 2.0
            for j in xrange( grid.info.height ):
              p = 0
              for n in track:
                #tmp = n.belief * self.ghmm.p_observation( [x, y], n )
                tmp = n.prior * self.ghmm.p_observation( [x, y], n )
                p  += tmp
              proba[ j * grid.info.width + i] = p
              total += p
              y += self.cell_size
            x += self.cell_size

          if total != 0:
            for i in xrange( grid.info.width ):
              for j in xrange( grid.info.height ):
                index = j * grid.info.width + i
                grid.data[index] = int( 100.0 * proba[index] / total )
            print "publish"
            self.publisher.publish( grid )
          else:
            print "total = 0"

#-------------------------------------------------------------------------------

if __name__ == "__main__":
  try:
    sigma = [
      [1., 0., 0., 0.],
      [0., 1., 0., 0.],
      [0., 0., 9., 0.],
      [0., 0., 0., 9.],
    ]
    handler = Handler( "ghmm", "/world", sigma, 1, 0.01, 0.01, 0.01, 0.3, -3, -3, 13, 13, 2 )
    rospy.init_node( "ghmm" )
    rospy.Subscriber( "dynamic_objects", TrajectoryObservation , handler )
    rospy.spin()
  except rospy.ROSInterruptException:
    pass
