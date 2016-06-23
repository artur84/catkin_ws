import sys
from collections import defaultdict
from graph import Graph
from kernels import mahalanobis
from eitm import EITM, Node
import math

class GHMM( object ):
  def __init__( 
      self, sigma, insertion_distance, epsilon,
      state_prior, transition_prior 
    ):

    self.__n = len( sigma )
    self.__sigma = sigma
    self.__insertion_distance = insertion_distance
    self.__epsilon = epsilon
    self.__state_prior = state_prior
    self.__transition_prior = transition_prior
    self.__graph = Graph()
    self.__distance = mahalanobis( sigma )
    self.__k = 0
    self.__itm = EITM( 
      self.__graph, 
      self.__distance, 
      self.__insertion_distance, 
      self.__epsilon 
    ) 

  def learn( self, trajectory ):
    self.__k += 1
    for o in trajectory:
      self.__itm( o )
    sum_prior = 0
    for n in self.__graph.nodes():
      try:
        old_transitions = n.transitions
        n.transitions = {}
        if not n in n.transitions.keys():
          n.transitions[n] = self.__transition_prior
        z = n.transitions[n]
        for c in self.__graph.children( n ):
          try:
            n.transitions[c] = old_transitions[c]
          except KeyError:
            n.transitions[c] = self.__transition_prior
          z += n.transitions[c]
        for c, value in n.transitions.items():
          n.transitions[c] = value / z
      except AttributeError:
        n.prior = self.__state_prior
        n.transitions = {}
        children = self.__graph.children( n )
        count = len( children )
        for c in children:
          n.transitions[c] = 1. / count

      sum_prior += n.prior

    for n in self.__graph.nodes():
      n.prior /= sum_prior

    for n in self.__graph.nodes():
      if n.prior == 0:
        print "Before forward"

    self.compute_forward( trajectory )
    for n in self.__graph.nodes():
      if n.prior == 0:
        print "Before backwards"
    self.compute_backwards( trajectory )
    for n in self.__graph.nodes():
      if n.prior == 0:
        print "Before update"
    self.update_parameters( trajectory )
    for n in self.__graph.nodes():
      if n.prior == 0:
        print "After update"

  def compute_forward( self, trajectory ):
    nodes = self.__graph.nodes()
    total = 0
    alpha = [{}] * len( trajectory )
    for n in nodes:
      tmp = n.prior * self.p_observation( trajectory[0], n )
      if tmp == 0:
        tmp = 1E-100
      alpha[0][n] = tmp
      total += tmp
    factors = [0] * len( trajectory )
    factors[0] = total

    for n in nodes:
      alpha[0][n] /= total

    t = 1
    for o in trajectory[1:]:
      total = 0
      alpha[t] = {}
      for n2 in nodes:
        alpha[t][n2] = 0
        for n1 in self.__graph.parents( n2 ):
          tmp = alpha[t - 1][n1] * n1.transitions[n2] * self.p_observation( o, n2 ) 
          alpha[t][n2] += tmp
          total += tmp

      factors[t] = total
      for n in nodes:
        alpha[t][n] /= total
      t += 1

    self.__alpha = alpha
    self.__factors = factors

  def compute_backwards( self, trajectory ):
    t = len( trajectory )
    beta = [{}] * ( t + 1 )
    nodes = self.__graph.nodes()
    for n in nodes:
      beta[t][n] = 1

    t -= 1

    for o in reversed( trajectory ):
      for n1 in nodes:
        beta[t][n1] = 0
        for n2, transition in n1.transitions.items():
          beta[t][n1] += (
              transition
            * self.p_observation( o, n2 )
            * beta[t + 1][n2]
            / self.__factors[t]
          )
      t -= 1
    self.__beta = beta

  def update_parameters( self, trajectory ):
    nodes   = self.__graph.nodes()
    alpha   = self.__alpha
    beta    = self.__beta
    factors = self.__factors

    total_prior = 0

    for n in nodes:
      tmp = alpha[0][n] * beta[0][n]
      if tmp == 0:
        tmp = 1E-100
      total_prior += tmp

    for n1 in nodes:
      tmp = alpha[0][n1] * beta[0][n1]
      if tmp == 0:
        tmp = 1E-100
      n1.new_prior = tmp / total_prior
      if n1.new_prior == 0:
        n1.new_prior = 1E-100
      n1.new_transitions = {}

      for n2, transition in n1.transitions.items():
        tmp = 0
        t = 1
        for o in trajectory[1:]:
          tmp += alpha[t - 1][n1] * n1.transitions[n2] * self.p_observation( o, n2 )
          if tmp == 0:
            print "-" * 80
            print t, alpha[t-1][n1], n1.transitions[n2], self.p_observation( o, n2 )
            tmp = 1E-100
          t += 1
        n1.new_transitions[n2] = tmp

    for n1 in nodes:
      tmp = 0
      for n2, transition in n1.new_transitions.items():
        tmp += transition
      for n2, transition in n1.new_transitions.items():
        n1.new_transitions[n2] = transition / tmp
      n1.transitions = n1.new_transitions
      n1.prior = ( n1.prior * ( self.__k - 1 ) + n1.new_prior ) / self.__k

  def update( self, nodes, o ):
    total = 0

    for n in nodes:
      n.new_belief = 0

    for n1 in nodes:
      for n2, transition in n1.transitions.items():
        tmp = n2.belief * transition * self.p_observation( o, n2 )
        total += tmp
        n2.new_belief += tmp

    for n in nodes:
      n.belief = n.new_belief / total
      if n.belief == 0:
        n.belief = 1E-100

  def predict( self, nodes, horizon ):
    for n in nodes:
      n.predictions = [n.belief] + [0] * horizon 

    for h in xrange( 1, horizon ):
      total = 0

      for n1 in nodes:
        for n2, transition in n1.transitions.items():
          tmp += n1.predictions[h - 1] * transition
          total += tmp

      for n in nodes:
        n.predictions[h] /= total

  def reset( self ):
    nodes = self.__graph.nodes()
    result = {}
    for n in nodes:
      new_node = Node()
      new_node.prior = n.prior
      new_node.belief = n.prior
      new_node.data = n.data
      result[n] = new_node
    for old_n, new_n in result.items():
      new_n.transitions = {}
      for n, value in old_n.transitions.items():
        new_n.transitions[result[n]] = value
    return result.values()

  def states( self ):
    return self.__graph.nodes()

  def p_observation( self, o, n ):
    d = self.__distance( o, n.data )
    return math.exp( -0.5 * d ** 2 )

if __name__ == "__main__":
  sigma = [
    [1., 0.],
    [0., 1.]
  ]
  g = GHMM( sigma, 0.5, 0.1, 0.001, 0.001 )
