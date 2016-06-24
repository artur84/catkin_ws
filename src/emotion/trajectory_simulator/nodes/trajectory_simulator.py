#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os

BASE_DIR = os.path.abspath( os.path.join( os.path.dirname( __file__ ), ".." ) )
path     = os.path.abspath( os.path.join( BASE_DIR, "python" ) )
if os.path.exists( path ):
  sys.path.append( path )

from trajectory_generator import TrajectoryGenerator

import roslib; roslib.load_manifest( "trajectory_simulator" )
import rospy

from trajectory_simulator.msg import TrajectoryObservation
from geometry_msgs.msg import Pose

import tf

import math
import random


import yaml
import glob
import os

#-------------------------------------------------------------------------------

class Trajectory( object ):
  object_id = 0
  def __init__( self, generators, publisher, br ):
    self.generators = generators
    self.publisher = publisher
    self.br = br

    self.traj = []
    self.object_id
    self.first = True
    self.seq = 0
    self.x0 = 0
    self.y0 = 0

  def __call__( self ):
    if len( self.traj ) == 0:
      #if random.random() > 0.995:
      if random.random() > 0.5:
        generator = generators[random.randint( 0, len( generators ) - 1 )]
        self.traj.extend( generator.next_trajectory() )
        self.first = True
        self.object_id = Trajectory.object_id
        Trajectory.object_id += 1
    if len( self.traj ) != 0:
      x, y = self.traj.pop( 0 )[:2]
      observation = TrajectoryObservation()
      observation.header.seq = self.seq
      observation.header.stamp = rospy.get_rostime()
      observation.header.frame_id = "1"
      observation.object_id = self.object_id
      if self.first:
        observation.type = TrajectoryObservation.FIRST
      elif len( self.traj ) == 0:
        observation.type = TrajectoryObservation.LAST
      observation.pose.x = x
      observation.pose.y = y
      observation.pose.theta = math.atan2( y - self.y0, x - self.x0 )
      self.publisher.publish( observation )

      self.br.sendTransform( 
        ( observation.pose.x, observation.pose.y, 0 ), 
        tf.transformations.quaternion_from_euler( 0, 0, observation.pose.theta ),
        observation.header.stamp,
        "dynamic_object_pose_%i" % self.object_id,
        "/map"
      )
      self.x0 = x
      self.y0 = y
      self.seq += 1
      self.first = False

class Handler( object ):
  def __init__( self, generators, max_objects ):
    publisher = rospy.Publisher( "dynamic_objects", TrajectoryObservation, queue_size=10) 
    br = tf.TransformBroadcaster()
    self.steps = [
      Trajectory( generators, publisher, br ) 
      for i in xrange( max_objects )
    ]

  def __call__( self ):
    for step in self.steps:
      step()


def read_generators( base_velocity, sampling_frequency ):
  files  = glob.glob( os.path.join( BASE_DIR, "data/*.sim" ) )
  generators = []

  for name in files:
    f = open( name )
    stream = yaml.load_all( f.read() )
    stream = [s for s in stream][1]
    model = {}

    for n in stream:
      model[n[":id"]] = n
    generators.append( TrajectoryGenerator( model, base_velocity, sampling_frequency ) )
    f.close()
  return generators

#-------------------------------------------------------------------------------

if __name__ == "__main__":
  try:
    base_velocity = 10
    sampling_frequency = 10
    generators = read_generators( base_velocity, sampling_frequency ) 
    rospy.init_node( "trajectory_simulator" )
    handler = Handler( generators, max_objects = 1 )
    while not rospy.is_shutdown():
      handler()
      rospy.sleep( 1. / sampling_frequency )
  except rospy.ROSInterruptException:
    pass
