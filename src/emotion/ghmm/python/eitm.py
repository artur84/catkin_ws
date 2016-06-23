import numpy

class Node( object ): pass

magic_number = 2

class EITM( object ):
  def __init__( self, graph, distance, insertion_distance, epsilon ):
    self.__graph = graph
    self.__distance = distance
    self.__insertion_distance = insertion_distance
    self.__epsilon = epsilon 
    self.__last_new = None
    self.__node_index = 0

  def insert_node( self, o ):
    n = Node()
    n.data = o
    n.index = self.__node_index
    self.__node_index += 1
    self.__graph.add_node( n )
    return n

  def __call__( self, o ):
    o = numpy.array( o )
    best = None
    second = None
    best_distance = -1.
    second_distance = -1.

    for n in self.__graph.nodes():
      distance = self.__distance( o, n.data )
      if  distance < best_distance or best_distance == -1:
        second = best
        best = n
        second_distance = best_distance
        best_distance = distance
      elif distance < second_distance or second_distance == -1:
        second = n
        second_distance = distance

    if best == None:
      self.insert_node( o )
      return
    
    if second == None:
      second = best;
      best = self.insert_node( o )

    if self.__distance( best.data, second.data ) < magic_number * self.__insertion_distance:
      self.__graph.add_edge( best, second )
      self.__graph.add_edge( second, best )

    best.data += self.__epsilon * ( o - best.data )

    children = self.__graph.children( best ).copy()
    for n in children:
      center = ( best.data + n.data ) * 0.5
      if (    self.__distance( center, second.data ) 
            < self.__distance( center, n.data ) 
          and
            self.__distance( best.data, n.data ) > 1
       ):
        self.__graph.remove_edge( best, n )
        self.__graph.remove_edge( n, best )

        if len( self.__graph.children( n ) ) == 0:
          self.__graph.remove_node( n )

    center = ( best.data + second.data ) * 0.5

    if (
         (  self.__distance( center, second.data ) < self.__distance( center, o )
          or
            self.__distance( second.data, o ) > self.__insertion_distance 
         )
        and
          self.__distance( best.data, o ) > self.__insertion_distance 
    ):
      r = self.insert_node( o )
      if self.__distance( best.data, r.data ) < self.__insertion_distance:
        self.__graph.add_edge( best, r )
        self.__graph.add_edge( r, best )

      if ( 
              self.__last_new != None 
          and r != self.__last_new 
          and self.__distance( r.data, self.__last_new.data ) < magic_number * self.__insertion_distance
      ):
        self.__graph.add_edge( r, self.__last_new )
        self.__graph.add_edge( self.__last_new, r )
      self.__last_new = r

      if self.__distance( best.data, second.data ) < 0.25 * self.__insertion_distance:
        self.__graph.remove_node( second )


if __name__ == "__main__":
  from graph import Graph

  def euclidean( v1, v2 ):
    dif = 0
    for i in xrange( len( v1 ) ):
      dif += ( v1[i] - v2[i] ) ** 2
    return dif ** 0.5

  g = Graph()
  itm = ITM( g, 2, euclidean, 0.5, 0.001 )
  itm( [1.1, 0.1] )
  itm( [2.1, 0.1] )
  itm( [3.1, 0.1] )
  itm( [4.1, 0.1] )
  print g.nodes()
