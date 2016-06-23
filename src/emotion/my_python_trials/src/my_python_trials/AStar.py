
import numpy as np

class Path:
    def __init__(self,nodes, totalCost):
        self.nodes = nodes;
        self.totalCost = totalCost;

    def getNodes(self):
        return self.nodes

    def getTotalMoveCost(self):
        return self.totalCost

class Node:
    def __init__(self,location,g,lid,parent=None):
        self.location = location # where is this node located
        self.g = g # Total move cost from start to this node
        self.parent = parent # parent node
        self.f = 0 # calculated score for this node
        self.lid = lid # location id - unique for each location in the map

    def __eq__(self, n):
        if n.lid == self.lid:
            return 1
        else:
            return 0

class AStar:
    """ This class performs the A* path planning. It keeps all the lists 
    """

    def __init__(self,maphandler):
        """
            @param maphandler: Type SQ_MapHandler, the map where we are going to perform the planning 
        """

        self.map_handler = maphandler
        self.open_list = []    #Open nodes: The set of tentative nodes to be evaluated, initially containing the "start" node.
        self.open_list_lid = []     #Open list: The same as open_list but it uses lid "location id" index to speed up searching
        self.closed_list = []     #Closed list: The set of nodes already evaluated

    def _getBestOpenNode(self):
        bestNode = None
        for n in self.open_list:
            if not bestNode:
                bestNode = n
            else:
                if n.f<=bestNode.f:
                    bestNode = n
        return bestNode

    def _tracePath(self,n):
        """ Traces the path from the starting point to the given Node "n"
            @param n: Final node
        """
        nodes = [];
        totalCost = n.g;
        p = n.parent;
        nodes.insert(0,n);

        while 1:
            if p.parent is None:
                break

            nodes.insert(0,p)
            p=p.parent

        return Path(nodes,totalCost)

    def _handleNode(self,node,end):
        print "in here"
        i = self.open_list_lid.index(node.lid)
        print "i"
        """Adds current node to the closed list"""
        self.open_list.pop(i)#first erase it from the open list
        self.open_list_lid.pop(i)#first erase it from the open list
        self.closed_list.append(node.lid)#add it to the closed list
        """Gets adjacent nodes"""
        adjacent_nodes_list = self.map_handler.getAdjacentNodes(node,self.end_node)
        """ Check if the node has reached the destination,
        is already in the closed list, or is a better element than those in the open list. """
        for n in adjacent_nodes_list:
            if n.location == self.end_node:
                # reached the destination
                return n
            elif n.lid in self.closed_list:
                # already in close, skip this
                continue
            elif n.lid in self.open_list_lid:
                # already in open, check if better score
                i = self.open_list_lid.index(n.lid)
                on = self.open_list[i];
                if n.g<on.g:
                    self.open_list.pop(i);
                    self.open_list_lid.pop(i);
                    self.open_list.append(n);
                    self.open_list_lid.append(n.lid);
            else:
                # new node, append to open list
                self.open_list.append(n);
                self.open_list_lid.append(n.lid);
        return None

    def start(self,fromlocation, tolocation):
        self.end_node = tolocation #The final point
        """ Add the starting point to the open list """
        self.start_node = self.map_handler.getNode(fromlocation)#The first node
        self.current_node = self.start_node
        self.open_list.append(self.start_node) #
        self.open_list_lid.append(self.start_node.lid)
        """init next node to be evaluated, in the beginning it is the starting point """
        self.nextNode = self.start_node

    def stepPath(self):
        """ Executes one cicle of the A* algorithm
            @return: adjacent_locations
            @return: next_location:
            @return: path_locations, if the algo is finished
        """
        """ Get the adjacent nodes for the current node (to be printed)"""
        self.current_node = self.nextNode
        adjacent_nodes = self.map_handler.getAdjacentNodes(self.current_node,self.end_node)
        """" Get location of the adjacent nodes in the list """
        adjacent_locations=[]
        for n in adjacent_nodes:
            adjacent_locations.append(n.location)
        """ Check if the search was already finished """
        if self.nextNode is not None:
            finish = self._handleNode(self.nextNode,self.end_node)
            print finish

            if finish:
                #Trace the path from the starting point to the end
                #returns the path
                path_locations = []
                path = self._tracePath(finish)
                for n in path.nodes:
                    path_locations.append(n.location)
                print "sending path"
                return adjacent_locations, self.current_node.location, path_locations

            self.nextNode=self._getBestOpenNode()
            next_location = self.nextNode.location
            return adjacent_locations, next_location, None
        else:
            return adjacent_locations, self.current_node.location, None

    def findPath(self,fromlocation, tolocation):
        self.end_node = tolocation #The final point

        """ Add the starting point to the open list """
        self.start_node = self.map_handler.getNode(fromlocation)
        self.open_list.append(self.start_node)
        """ Add the node to the o list """
        self.open_list_lid.append(self.start_node.lid)
        nextNode = self.start_node

        while nextNode is not None:
            #Searching the path
            finish = self._handleNode(nextNode,self.end_node)
            if finish:
                #Trace the path from the starting point to the end
                #returns the path
                return self._tracePath(finish)
            nextNode=self._getBestOpenNode()

        return None

class SQ_Location:
    """A simple Square Map Location implementation"""
    def __init__(self,x,y):
        self.x = x
        self.y = y

    def __eq__(self, l):
        """MUST BE IMPLEMENTED"""
        if l.x == self.x and l.y == self.y:
            return 1
        else:
            return 0

class SQ_MapHandler:
    """A simple Square Map implementation (This is the real structure to make calculations,
    not the one used to display the results). """

    def __init__(self,mapdata,width,height):
        self.m = mapdata #Type [], it contains the values for all the cells in the map (-1 == obstacle, other == free)
        self.w = width
        self.h = height

    def getNode(self, location):
        """Checks if "location" exists in the map, gets its value, and local id "lid"
           @param location: @type SQ_Location(x,y),
           @return: Node(location,value,lid, parent=None)
        """
        x = location.x
        y = location.y
        if x<0 or x>=self.w or y<0 or y>=self.h:
            return None
        d = self.m[(y*self.w)+x]  #self.m = mapdata
        if d == -1:#If it is an obstacle
            return None

        return Node(location,d,((y*self.w)+x));

    def getAdjacentNodes(self, curnode, dest):
        """Computes the adjacent nodes to "curnode"
            This gives not only the position of the adjacent nodes but also the
            score value f for each one.
            @param dest: The destination node, it is necessary to compute the distance value "f" for each adjacent node 
            @return: adj_nodes; a list [] containing the adjacent nodes.
        """
        adj_nodes = []

        cl = curnode.location
        dl = dest

        # Gets four neighbors
        #     [  ]
        # [  ][cl][  ]
        #     [  ]
        n = self._handleNode(cl.x,cl.y+1,curnode,dl.x,dl.y, False)
        if n: adj_nodes.append(n)
        n = self._handleNode(cl.x,cl.y-1,curnode,dl.x,dl.y, False)
        if n: adj_nodes.append(n)
        n = self._handleNode(cl.x+1,cl.y,curnode,dl.x,dl.y, False)
        if n: adj_nodes.append(n)
        n = self._handleNode(cl.x-1,cl.y,curnode,dl.x,dl.y, False)
        if n: adj_nodes.append(n)
        # The other four neighbors
        # [  ]    [  ]
        #     [cl]
        # [  ]    [  ]
        n = self._handleNode(cl.x+1,cl.y-1,curnode,dl.x,dl.y, True)
        if n: adj_nodes.append(n)
        n = self._handleNode(cl.x+1,cl.y+1,curnode,dl.x,dl.y, True)
        if n: adj_nodes.append(n)
        n = self._handleNode(cl.x-1,cl.y-1,curnode,dl.x,dl.y, True)
        if n: adj_nodes.append(n)
        n = self._handleNode(cl.x-1,cl.y+1,curnode,dl.x,dl.y, True)
        if n: adj_nodes.append(n)

        return adj_nodes

    def _handleNode(self,x,y,fromnode,destx,desty,is_corner=False):
        """ Creates a node in the selected x,y position of the map. 
            It will compute the node costfunctions f, h, and g. 
            @return: n; the node if it exists. otherwise it returns None.
        """
        n = self.getNode(SQ_Location(x,y))
        if n is not None:
            #Computes the manhatan distance from current point to destination
            dx = np.max([x,destx]) - np.min([x,destx]) #Distance in x, from current point to destination
            dy = max(y,desty) - min(y,desty) #Distance in y, from current point to destination
            h = np.sqrt(np.power(float(x)-float(destx),2)+np.power(float(y)-float(desty),2)) #heuristic euclidean distance
            #print "h=", h
            #h = dx+dy #heuristic = Manhatan distance
            if is_corner:
                n.g = fromnode.g+1.0
            else:
                n.g = fromnode.g+1
            n.f = n.g + h #Total cost of this node
            n.parent=fromnode
            return n

        return None
