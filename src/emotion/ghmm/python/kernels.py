import numpy

def euclidean( v1, v2 ):
  dif = 0
  for i in xrange( len( v1 ) ):
    dif += ( v1[i] - v2[i] ) ** 2
  return dif ** 0.5

class mahalanobis( object ):
  def __init__( self, sigma ):
    self.sigma = numpy.matrix( sigma )
    self.inv_sigmas = {}
    
  def __call__( self, v1, v2 ):
    if not len( v1 ) in self.inv_sigmas:
      self.inv_sigmas[len( v1 )] = self.sigma[:len( v1 ), :len( v1 )].getI()
    inv = self.inv_sigmas[len( v1 )]
    v2 = v2[:len( v1 )]
    dif = numpy.matrix( v1 - v2 )
    return ( dif * inv * dif.getT() )[0, 0] ** 0.5
