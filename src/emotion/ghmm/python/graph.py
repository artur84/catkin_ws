from collections import defaultdict

class InvalidGraphOperation: pass

class Graph( object ):
  def __init__( self ):
    self.__nodes = set()
    self.__children = defaultdict( set )
    self.__parents = defaultdict( set )

  def children( self, node ):
    return self.__children[node]

  def parents( self, node ):
    return self.__parents[node]

  def nodes( self ):
    return self.__nodes

  def has_edge( self, node1, node2 ):
    return node1 in self.__children and node2 in self.__children[node1]

  def has_node( self, node ):
    return node in self.__nodes

  def add_node( self, node ):
    if not node in self.__nodes:
      self.__nodes.add( node )
      self.__children[node] = set()
      self.__parents[node] = set()

  def add_edge( self, node1, node2 ):
    if not node1 in self.__nodes or not node2 in self.__nodes:
      raise InvalidGraphOperation()
    self.__children[node1].add( node2 )
    self.__parents[node2].add( node1 )

  def remove_node( self, node ):
    if node in self.__nodes:
      children = self.__children[node].copy()
      for e in children:
        self.remove_edge( node, e )
      del self.__children[node]

      parents = self.__parents[node].copy()
      for e in parents:
        self.remove_edge( e, node )
      del self.__parents[node]

  def remove_edge( self, node1, node2 ):
    self.__children[node1].remove( node2 )
    self.__parents[node2].remove( node1 )

  def print_structures( self ):
    print self.__nodes
    print self.__children
    print self.__parents
    

if __name__ == "__main__":
  g = Graph()

  g.add_node( 0 )
  g.add_node( 1 )

  g.add_edge( 0, 0 )
  g.add_edge( 0, 1 )
  g.add_edge( 1, 1 )
  g.add_edge( 1, 0 )

  g.print_structures()
