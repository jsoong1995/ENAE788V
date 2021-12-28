#!/usr/bin/env python
# coding: utf-8

# In[1]:


# Importing necessary classes
import csv
from heapq import heappush,heappop,heapify
import math as m
import matplotlib
import matplotlib.pyplot as plt
get_ipython().run_line_magic('matplotlib', 'notebook')

# The Priority Queue code comes from a built-in heap queue structure in Python. All I did
# was just import it. 


# In[2]:


class Node():
    # NOTE: self.status and self.priority was not used    
    def __init__(self,iden,x,y):
        self.iden = int(iden)
        self.x = float(x)
        self.y = float(y)
        self.edgeCollection = []
        self.neighborID = []
        self.neighbor = []
        self.status = 0    # Status: 0 for unvisited, 1 for open list, 2 for closed list
        self.parent = None
        self.g = 0    # Node's cost
        self.priority = 0    # empty structure

        
    # "neigh" function constructs the list of nearest neighbors for each node
    def neigh(self):
        for item in edgeList:
            if self.iden == item.start:
                self.edgeCollection.append(item)
                self.neighborID.append(item.end)
        for item in nodeList:
            for idenNo in self.neighborID:
                if item.iden == idenNo:
                    self.neighbor.append(item)

                    
    # "heuristic" calculates the heuristic for each node
    def heuristic(self,goalX,goalY):
        self.h = ((self.x - goalX)**2 + (self.y - goalY)**2)**(0.5)


# In[3]:


class Edge():
    def __init__(self,startID,endID,cost):
        self.cost = float(cost)
        for item in nodeList:
            if item.iden == startID:
                self.start = item.iden
                self.xStart = item.x
                self.yStart = item.y
            if item.iden == endID:
                self.end = item.iden
                self.xEnd = item.x
                self.yEnd = item.y
                
        


# In[8]:


class AstarAlg():
    def __init__(self,start,goal,nodes,edges):
        self.unvisited = nodes
        self.pQueue = []
        self.closed = []
        self.startPoint = start
        self.goalPoint = goal
        self.failure = 'Path could not be found'
        self.turnOffTraceback = 0
    
    
    # "costFinder" finds the cost between two nodes
    def costFinder(self,node1,node2):
        id1 = node1.iden
        id2 = node2.iden
        thisEdgeCost = 0
        for item in edgeList:
            if item.start == id1 and item.end == id2:
                thisEdgeCost = item.cost
        return thisEdgeCost
                
        
    # "solver" performs the A* algorithm.
    def solver(self):
        for item in self.unvisited:
            if item.iden == self.startPoint:
                item.status = 1
                self.pQueue.append((item.g,item))
                self.unvisited.pop(self.unvisited.index(item))        
        breakVar = 0       
        while len(self.pQueue) != 0:
            v = heappop(self.pQueue)[1]
            self.closed.append(v)
            plt.plot(v.x,v.y,'og')
            if v.parent != None:
                plt.plot([v.x,v.parent.x],[v.y,v.parent.y],'g')
            while len(v.neighbor) != 0:
                u = v.neighbor.pop()
                travelCost = self.costFinder(v,u)
                if u in self.unvisited or u.g > v.g + travelCost:
                    if u in self.unvisited:
                        self.unvisited.pop(self.unvisited.index(u)) 
                    u.parent = v
                    u.g = v.g + travelCost
                    heappush(self.pQueue,(u.g + u.h,u))
                if v.iden == self.goalPoint:
                    breakVar = 1
                    break
            if breakVar == 1:
                break
        if breakVar == 0:
            print(self.failure)
            self.turnOffTraceback = 1
            
    
    
    # "traceback" recreates the path from goal to start by matching a node's parent
    # to one in the closed list and writes the ID, x, and y to a text file.
    # "traceback" also writes out a file denoting failure to find path if such a 
    #scenario occurs.
    def traceback(self,start,goal):
        idPath = []
        xPath = []
        yPath = []        
        if self.turnOffTraceback == 1:
            with open('paths_3.txt','w') as pathNode:
                pathData = csv.writer(pathNode,delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                pathData.writerow('Path could not be found')
        else:    
            for item in self.closed:
                if item.iden == goal:
                    idPath.append(item.iden)
                    xPath.append(item.x)
                    yPath.append(item.y)
                    self.closed.pop(self.closed.index(item))
                    currentParentID = item.parent.iden        
            breakVar2 = 0
            while len(self.closed) != 0:
                for item in self.closed:
                    if item.iden == currentParentID:
                        idPath.append(item.iden)
                        xPath.append(item.x)
                        yPath.append(item.y)
                        self.closed.pop(self.closed.index(item))
                        currentParentID = item.parent.iden
                    if currentParentID == start:
                        for item in self.closed:
                            if item.iden == currentParentID:
                                idPath.append(item.iden)
                                xPath.append(item.x)
                                yPath.append(item.y)
                                self.closed.pop(self.closed.index(item))
                                breakVar2 = 1
                                break
                    if breakVar2 == 1:
                        break
                if breakVar2 == 1:
                    break
            plt.plot(xPath,yPath,'r')
            print(idPath)
            with open('paths_3.txt','w') as pathNode:
                pathData = csv.writer(pathNode,delimiter=',', quotechar='"', quoting=csv.QUOTE_MINIMAL)
                for i in range(0,len(idPath)):
                    pathData.writerow([idPath[i],xPath[i],yPath[i]])
                
                    


# In[5]:


# "visualization" graphed the raw edges and nodes.
def visualization():
    for item in nodeList:
        plt.plot(item.x,item.y,'or')
    
    for item in edgeList:
        plt.plot([item.xStart,item.xEnd],[item.yStart,item.yEnd],'y')
    
    plt.title('Graph of Question 3')
    plt.show()
    
    for item in nodeList:
        if item.iden == startID:
            plt.plot(item.x,item.y,'*b',markersize = 10)
        if item.iden == goalID:
            plt.plot(item.x,item.y,'Db',markersize = 7.5)
    


# In[6]:


nodeList = []
edgeList =[]
startID = 30
goalID = 95


# Two .csv readers to read in node and edge data
with open('nodes_3.txt','r') as nodes:
    nodeData = csv.reader(nodes,delimiter=',')
    nodeCount = int(next(nodeData)[0])
    for item in nodeData:
        nodeList.append(Node(item[0],item[1],item[2]))
        
with open('edges_with_costs_3.txt','r') as edges:
    edgeData = csv.reader(edges,delimiter=',')
    edgeCount = int(next(edgeData)[0])
    for item in edgeData:
        edgeList.append(Edge(int(item[0]),int(item[1]),item[2]))
    

visualization()


# Construction of nearest neighbors
for item in nodeList:
    item.neigh()
    if item.iden == goalID:
        xHeur = item.x
        yHeur = item.y

        
# Calculation of heuristics for each node.
for item in nodeList:
    item.heuristic(xHeur,yHeur)

Alg = AstarAlg(startID,goalID,nodeList,edgeList)

Alg.solver()

Alg.traceback(startID,goalID)



# In[ ]:




