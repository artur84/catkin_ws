import random
import numpy as np
import scipy.interpolate

class TrajectoryGenerator( object ):
  def __init__( self, nodes, base_velocity, sampling_frequency ):
    self.__nodes = nodes
    self.__sampling_frequency = sampling_frequency
    self.__max_speed = base_velocity * 1000 / ( 3600. * sampling_frequency )
    self.__start = [item for item in nodes.values() if item[":start"]]
    self.__end = [item for item in nodes.values() if item[":end"]]
    self.__s_min = 0.5

  def next_trajectory( self ):
    n1 = self.__start[random.randint( 0, len( self.__start ) - 1 )]
    n2 = self.__end[random.randint( 0, len( self.__end ) - 1 )]
    path = self.find_path( n1, n2 )
    while path == None or n1 == n2 or len( path ) < 4:
      n1 = self.__start[random.randint( 0, len( self.__start ) - 1 )]
      n2 = self.__end[random.randint( 0, len( self.__end ) - 1 )]
      path = self.find_path( n1, n2 )
    return self.path_to_trajectory( path )

  def find_path( self, n1, n2 ):
    if n1 == n2:
      return [n1]

    if n1[":end"] or len( n1[":neighbors"] ) == 0:
      return None

    for id in n1[":neighbors"]:
      result = self.find_path( self.__nodes[id], n2 )
      if result:
        result.insert( 0, n1 )
        return result
    return None

  def path_to_trajectory( self, path ):
    x0 = np.array( [
      n[":x"] + np.random.normal( 0, n[":position_variance_factor"] )
      for n in path
    ] ) 
    y0 = np.array( [
      n[":y"] + np.random.normal( 0, n[":position_variance_factor"] )
      for n in path
    ] ) 
    speed0 = np.array( [n[":speed"] * self.__max_speed / 100 for n in path] )

    s0 = [0]
    for i in xrange( 1, len( x0 ) ):
      x1 = path[i - 1][":x"]
      y1 = path[i - 1][":y"]
      x2 = path[i][":x"]
      y2 = path[i][":y"]
      s0.append( ( ( x2 - x1 ) ** 2 + ( y2 - y1 ) ** 2 ) ** 0.5 + s0[i - 1] )

    s0 = np.array( s0 )
    s_x = scipy.interpolate.InterpolatedUnivariateSpline( s0, x0 )
    s_y = scipy.interpolate.InterpolatedUnivariateSpline( s0, y0 )
    s_speed = scipy.interpolate.InterpolatedUnivariateSpline( s0, speed0 )

    s = 0
    x = s_x( s )
    y = s_y( s )
    speed = s_speed( s )
    traj = [[x, y, speed, s_x.derivatives( s )[0], s_y.derivatives( s )[0]]]

    while abs( s - s0[-1] ) > self.__s_min:
      s += speed
      x = s_x( s )
      y = s_y( s )
      speed = s_speed( s )
      traj.append( [x, y, speed, s_x.derivatives( s )[0], s_y.derivatives( s )[0]] )
    return traj

if __name__ == "__main__":
  import yaml

  stream = yaml.load_all( open( "data/model.sim" ).read() )
  stream = [s for s in stream]

  model = {}

  for n in stream[1]:
    model[n[":id"]] = n

  generator = TrajectoryGenerator( model, 30, 10 )
