# MIT License
#
# Copyright (c) [2020] [mouad boumediene]
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

import cv2 as cv
import numpy as np
import random
import math
class RRTMap:
    def __init__(self,start,goal,MapDimensions):
        self.start=start
        self.goal=goal
        self.MapDimension=MapDimensions
        self.Maph,self.Mapw=self.MapDimension[0],self.MapDimension[1]
        self.MapImg=np.ones([self.Maph,self.Mapw,3],np.uint8)*255 # white background
        self.MapWindowName="RRT path planning"
        self.nodeRad=0
        self.nodeThickness=-1
        self.edgeThickness=1
        #Colors
        self.Black = (20, 20, 20)
        self.Blue = (255, 0, 0)
        self.Green = (0, 255, 0)
        self.Red = (0, 0, 255)
        self.white = (255, 255, 255)

        self.obstacles=[]



    def drawMap(self,obstacles):
        self.drawNode(None,nodeType='G')
        self.drawNode(None,nodeType='S')
        self.drawObs(obstacles)
        cv.imshow(self.MapWindowName,self.MapImg)
        cv.waitKey(1)

    def refreshMap(self):
        cv.imshow(self.MapWindowName, self.MapImg)
        cv.waitKey(1)

    def drawNode(self,coords,nodeType):
        if nodeType=='G': # draw goal node
            cv.circle(self.MapImg,
                      (self.goal[0],self.goal[1]),
                      self.nodeRad,
                      self.Blue,
                      10)
        if nodeType=='S': # draw start node
            cv.circle(self.MapImg,
                      (self.start[0], self.start[1]),
                      self.nodeRad,
                      self.Green,
                      10)
        if nodeType=='N': # draw normal node
            cv.circle(self.MapImg,
                      (coords[0], coords[1]),
                      self.nodeRad,
                      self.Blue,
                      self.nodeThickness)
        if nodeType=='P': # draw path node
            cv.circle(self.MapImg,
                      (coords[0], coords[1]),
                      self.nodeRad,
                      self.Red,
                      4)

    # draw the edge between two nodes
    def drawEdge(self,node1,node2):
        cv.line(self.MapImg,
                (node1[0],node1[1]),
                (node2[0],node2[1]),
                self.Blue,
                self.edgeThickness)

    # draw the pah given the path nodes coords
    def drawPath(self,path):
        for node in path:
            self.drawNode(coords=node,nodeType='P')
            cv.imshow(self.MapWindowName,self.MapImg)
            cv.waitKey(1)

    # draw obstacles given the obstacle list as a param
    def drawObs(self,obstacles):
        obstaclesList=obstacles.copy()
        while(len(obstaclesList)>0):
            upper=obstaclesList.pop(0) # upper corner of the  obstacle
            lower=obstaclesList.pop(0) # lower corner of the obstacle
            cv.rectangle(self.MapImg,
                         (upper[0],upper[1]),
                         (lower[0],lower[1]),
                          self.Black,
                         -1)


class RRTGraph:
    def __init__(self,nstart,ngoal,mapdimensions):
        (x,y)=nstart
        self.start=nstart
        self.ngoal=ngoal
        self.goalFlag=False
        self.x=[]
        self.y=[]
        self.parent=[]
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0) # the parent of node 0 is the node 0 itself
        self.obstacles=[]
        self.obsDim=30 # the dimensions of the square obstacle
        self.obsNum=100 # the number of obstacles
        self.maph,self.mapw=mapdimensions
        self.goalstate = None # the goalstate is  the node from the tree that is the closest to the goal
        self.path = []

    # get cost from start to node
    def cost(self, n):
        ninit = 0
        n = n
        parent = self.parent[n]
        c = 0
        while n is not ninit:
            c = c + self.metric(n, parent)
            n = parent
            if n is not ninit:
                parent = self.parent[n]
        return c

    def makeRandomRect(self):
        # centers of random rects
        centerx=int(random.uniform(self.obsDim/2,self.mapw-self.obsDim/2))
        centery=int(random.uniform(self.obsDim/2,self.maph-self.obsDim/2))
        # upper and lower conrners
        uppercornerx=(centerx-int(self.obsDim/2))
        uppercornery=(centery-int(self.obsDim/2))
        return [uppercornerx,uppercornery]

    def makeobs(self):
        obs=[]
        for i in range(0,self.obsNum-1):
            upper=self.makeRandomRect()
            obs.append(upper)
            obs.append([upper[0]+self.obsDim,upper[1]+self.obsDim])
        self.obstacles=obs.copy()
        return obs

# add node
    def add_node(self,n,x,y):
        self.x.insert(n,x)
        self.y.insert(n,y)

# remove node
    def remove_node(self,n):
        self.x.pop(n)
        self.y.pop(n)

# add edge
    def add_edge(self,child,parent):
        self.parent.insert(parent,child)

#remove node
    def remove_edge(self,n):
        self.parent.pop(n)

# total number of nodes
    def number_of_nodes(self):
        return len(self.x)

# get the distance between two nodes
    def metric(self,n1,n2):
        (x1,y1)=(self.x[n1],self.y[n1])
        (x2,y2)=(self.x[n2],self.y[n2])
        x1=float(x1)
        x2=float(x2)
        y1=float(y1)
        y2=float(y2)
        px=(x1-x2)**(2)
        py=(y1-y2)**(2)
        metric=(px+py)**(0.5)
        return metric

# nearest node
    def nearest(self,n):
        dmin=self.metric(0,n)
        nnear=0
        for i in range(0,n):
            if self.metric(i,n)<dmin:
                dmin=self.metric(i,n)
                nnear=i
        return nnear

    #sampling a new node
    def sample_envir(self):
        x=int(random.uniform(0,self.mapw))
        y=int(random.uniform(0,self.maph))
        return x,y

# is the new sample located in free space
    def isFree(self):
        n=self.number_of_nodes()-1# get the last node ( new sample)
        (x,y)=(self.x[n],self.y[n])
        obs=self.obstacles.copy()
        while len(obs)>0:
            upper=obs.pop(0)
            lower=obs.pop(0)
            if upper[0] < x < lower[0] and upper[1] < y < lower[1]:
                self.remove_node(n)
                return False
        return True

# is this edge cross an obstacle
    def crossObstacle(self,x1,x2,y1,y2):
        obs=self.obstacles.copy()
        while(len(obs)>0):
            upper=obs.pop(0)
            lower=obs.pop(0)
            for i in range(0,101):
                u=i/100
                x=x1*u + x2*(1-u)
                y=y1*u + y2*(1-u)
                if upper[0] < x < lower[0] and upper[1] < y < lower[1]:
                    return True
        return False

# connect two nodes
    def connect(self,n1,n2):
        (x1, y1)=(self.x[n1],self.y[n1])
        (x2,y2)=(self.x[n2],self.y[n2])
        n=self.number_of_nodes()-1
        if self.crossObstacle(x1,x2,y1,y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1,n2)
            return True

# use the random sample to create a node and add it to the tree
    def step(self,nnear,nrand,dmax=8):
        d=self.metric(nnear,nrand)
        if d>dmax:
            u=dmax/d
            (xnear,ynear)=(self.x[nnear],self.y[nnear])
            (xrand,yrand)=(self.x[nrand],self.y[nrand])
            (px,py)=(xrand-xnear,yrand-ynear)
            theta=math.atan2(py,px)
            (x,y)=(int(xnear+ dmax * math.cos(theta) ),
                   int(ynear + dmax * math.sin(theta)))
            self.remove_node(nrand)
            self.add_node(nrand,x,y)
            if abs(x-self.ngoal[0])<20 and abs(y-self.ngoal[1])<10:
                self.goalstate = nrand
                self.goalFlag=True



    def path_to_goal(self,ngoal):
        if self.goalFlag:
            self.path=[]
            self.path.append(self.goalstate)
            newpos=self.parent[self.goalstate]
            while ( newpos !=0):
                self.path.append(newpos)
                newpos=self.parent[newpos]
            self.path.append(0)
        return self.goalFlag

    def getPathCoords(self):
        pathCoords=[]
        for node in self.path:
            x,y=(self.x[node],self.y[node])
            pathCoords.append((x,y))
        return pathCoords

    def bias(self,ngoal):
        n=self.number_of_nodes()
        self.add_node(n,ngoal[0],ngoal[1])
        nnear=self.nearest(n)
        self.step(nnear,n)
        self.connect(nnear,n)
        return self.x,self.y,self.parent

    def expand(self):
        n=self.number_of_nodes()
        x,y=self.sample_envir()
        self.add_node(n,x,y)
        if self.isFree():
            xnearest=self.nearest(n)
            self.step(xnearest,n)
            x1=self.x[n]
            y1=self.y[n]
            self.connect(xnearest,n)
        return self.x,self.y,self.parent
