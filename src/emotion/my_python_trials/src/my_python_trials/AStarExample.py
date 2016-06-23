#/usr/bin/env python

import pygame
from pygame.locals import *
from copy import deepcopy
from time import time

import AStar
from copy import deepcopy

class AStarExample:

    #           -11 white free cells    2 grey cells           3 path      4  open list    5 Start    6 end     7 Next       8 current
    colors = [(0,0,0),(255,255,255),(255-10,255-10,255-10),(180,180,180),(255,255-60,255),(0,255,0),(255,0,0),(255,255,0),(100,100,255)]
    color_dict ={"white":1, "gray":2, "path":3, "adjacent":4, "start":5, "end":6, "next":7, "current":8 }
    pathlines = []

    def initMap(self,w,h):
        self.mapdata = []
        self.mapw = w
        self.maph = h
        self.startpoint = [1,h/2]
        self.endpoint = [w-2,h/2]
        self.current_location = AStar.SQ_Location(self.startpoint[0],self.startpoint[1]) #Initializes the current location as the starting point
        self.next_location = deepcopy(self.current_location)
        size = w*h;
        for i in range(size):
            #Alternates color, to easily visualize the grid
            if i%2==0:
                self.mapdata.append(1)
            else:
                self.mapdata.append(2)


        self.mapdata[(self.startpoint[1]*w)+self.startpoint[0]] = 5
        self.mapdata[(self.endpoint[1]*w)+self.endpoint[0]] = 6
        """Create a copy of the initial configuration of the map """
        self.initial_mapdata = deepcopy(self.mapdata)
        self.maprect = Rect(0,0,w*32,h*32) #each small cell is 32 pix size.
        """init astar planner """
        """first the map"""
        self.astar = AStar.AStar(AStar.SQ_MapHandler(self.mapdata,self.mapw,self.maph))
        """init the start and end positions"""
        start = AStar.SQ_Location(self.startpoint[0],self.startpoint[1])
        end = AStar.SQ_Location(self.endpoint[0],self.endpoint[1]) #SQ means "square" map location
        self.astar.start(start, end)

    def drawPoints(self,locations,color=6):
        for point in locations:
            #Only draw in "white" cells (remember that values =1 is white and =2 for gray cells)
            if color == self.color_dict["adjacent"]:
                if self.mapdata[(point.y*self.mapw)+point.x] == self.color_dict["white"] \
                or self.mapdata[(point.y*self.mapw)+point.x] == self.color_dict["gray"]: ##white free cells, gray free cells,adjacent cells
                    self.mapdata[(point.y*self.mapw)+point.x] = color
            if color == self.color_dict["current"]:
                if  self.mapdata[(point.y*self.mapw)+point.x] != self.color_dict["start"]:
                    self.mapdata[(point.y*self.mapw)+point.x] = color
            if color == self.color_dict["next"]:
                if  self.mapdata[(point.y*self.mapw)+point.x] != self.color_dict["end"]:
                    self.mapdata[(point.y*self.mapw)+point.x] = color
            if color == self.color_dict["path"]:
                if  self.mapdata[(point.y*self.mapw)+point.x] != self.color_dict["end"]\
                and self.mapdata[(point.y*self.mapw)+point.x] != self.color_dict["start"]:
                    self.mapdata[(point.y*self.mapw)+point.x] = color

        self.drawMap()

    def drawMap(self):
        x = 0
        y = 0
        rect = [0,0,32,32]
        for p in self.mapdata:
            if p == -1:
                p = 0
            rect[0] = x*32 #Each map cell is 32 pix size
            rect[1] = y*32
            self.screen.fill(self.colors[p],rect)
            x+=1
            if x>=self.mapw:
                x=0
                y+=1

    def updateMap(self,x,y,v):
        mi = (y*self.mapw)+x
        if v == 5: # startpoint
            if self.mapdata[mi] != 5 and self.mapdata[mi] != 6:
                self.mapdata[(self.startpoint[1]*self.mapw)+self.startpoint[0]] = 1
                self.screen.fill(self.colors[1],(self.startpoint[0]*32,self.startpoint[1]*32,32,32))
                self.startpoint = [x,y]
                self.mapdata[mi] = 5
                self.screen.fill(self.colors[5],(x*32,y*32,32,32))
        elif v == 6: # endpoint
            if self.mapdata[mi] != 5 and self.mapdata[mi] != 6:
                self.mapdata[(self.endpoint[1]*self.mapw)+self.endpoint[0]] = 1
                self.screen.fill(self.colors[1],(self.endpoint[0]*32,self.endpoint[1]*32,32,32))
                self.endpoint = [x,y]
                self.mapdata[mi] = 6
                self.screen.fill(self.colors[6],(x*32,y*32,32,32))
        else:
            if self.mapdata[mi] != 5 and self.mapdata[mi] != 6:
                if v == 0:
                    self.mapdata[mi] = -1
                else:
                    self.mapdata[mi] = v

                self.screen.fill(self.colors[v],(x*32,y*32,32,32))

    def drawMenu(self):

        text = ["Blocking (-1)", "Walkable (1)", "Walkable (2)", "Walkable (3)", "Walkable (4)", "Start point", "End point", "Find path", "Step"]

        fnt = pygame.font.Font(pygame.font.get_default_font(),11)

        self.menurect = Rect(550,5,85,32*len(self.colors))

        rect = Rect(550,5,85,32)

        i = 0
        for c in self.colors:
            self.screen.fill(c,rect)
            ts=fnt.render(text[i], 1, (200,200,100))
            trect = ts.get_rect()
            trect.center = rect.center
            self.screen.blit(ts,trect.topleft)
            rect.y+=32;
            i+=1
    def stepPath(self):
        """ Avance one step of the algorithm
        """
        self.current_location = self.next_location
        self.adjacent_locations, self.next_location, self.path = self.astar.stepPath()



    def findPath(self):


        start = AStar.SQ_Location(self.startpoint[0],self.startpoint[1])
        end = AStar.SQ_Location(self.endpoint[0],self.endpoint[1]) #SQ means "square" map location

        s = time()
        p = self.astar.findPath(start,end)
        e = time()

        if not p:
            print "No path found!"
        else:
            print "Found path in %d moves and %f seconds." % (len(p.nodes),(e-s))
            self.pathlines = []
            self.pathlines.append((start.x*32,start.y*32+32/2-1))
            for n in p.nodes:
                self.pathlines.append((n.location.x*32,n.location.y*32+32/2-1))
            self.pathlines.append((end.x*32+32*2,end.y*32+32/2-1))

    def mainLoop(self):

        pygame.init()

        self.screen = pygame.display.set_mode((640, 480),HWSURFACE)
        pygame.display.set_caption('AStarExample')


        self.screen.fill((150,150,150))

        self.initMap(7,7)
        self.drawMap()
        self.editmode = 0

        self.drawMenu()

        while 1:
            for event in pygame.event.get():
                if event.type == QUIT:
                    return
                elif event.type == KEYDOWN:
                    if event.key == K_ESCAPE:
                        return

                elif event.type == MOUSEBUTTONDOWN:
                    if len(self.pathlines):
                        self.pathlines=[]
                        self.drawMap()
                    mx = event.pos[0]
                    my = event.pos[1]
                    if self.maprect.collidepoint(mx,my):
                        self.updateMap(mx/32,my/32,self.editmode)
                    elif self.menurect.collidepoint(mx,my):
                        my-=self.menurect.y
                        em = my/32
                        if em == 7: #trace
                            self.findPath()
                            if len(self.pathlines):
                                pygame.draw.lines(self.screen, (200,100,200,200), 0, self.pathlines, 32)
                        elif em == 8: #Step
                            self.stepPath()

                            self.drawPoints([self.next_location], self.color_dict["next"])
                            self.drawPoints(self.adjacent_locations, self.color_dict["adjacent"])
                            self.drawPoints([self.current_location],self.color_dict["current"])
                            print self.path
                            if self.path is not None:
                                self.drawPoints(self.path, self.color_dict["path"])


                        else:
                            self.editmode = em

                elif event.type == MOUSEMOTION and event.buttons[0]:
                    mx = event.pos[0]
                    my = event.pos[1]
                    if self.maprect.collidepoint(mx,my):
                        if len(self.pathlines):
                            self.pathlines=[]
                            self.drawMap()
                        self.updateMap(mx/32,my/32,self.editmode)

            pygame.display.flip()

def main():
    g = AStarExample()
    g.mainLoop()


#this calls the 'main' function when this script is executed
if __name__ == '__main__': main()

