#!/usr/bin/env python
# coding: utf-8

# In[1]:


# Imported libraries
import math as m
import numpy as np
import random as ran
import matplotlib
import matplotlib.pyplot as plt
import csv
get_ipython().run_line_magic('matplotlib', 'notebook')


# In[2]:


# Obstacle data structure
class obstacle():
    def __init__(self,obst):
        self.obst = obst
        self.xObst = []
        self.yObst = []
        self.radObst = []
        for obs in self.obst:
            self.xObst.append(obs[0])
            self.yObst.append(obs[1])
            self.radObst.append(obs[2])

    def showObst(self):
        for obs in self.obst:
            i = 0
            xObs = []
            yObs = []
            for i in range(0,359):
                tht = i*2*m.pi/360
                xObs.append(obs[2]*m.cos(tht) + obs[0])
                yObs.append(obs[2]*m.sin(tht) + obs[1])
                i +=1
            plt.plot(xObs,yObs,'b')



# In[3]:


# Pictorial visualization of c-space

def vis():
    # Setting plot axes to be that of total configuration space
    plt.xlim([B.xmin, B.xmax])
    plt.ylim([B.ymin, B.ymax])

    # Goal region: xOffset and yOffset are the coordinates for goal's center.
    xGoal = []
    yGoal = []
    xOffset = G.xGoal
    yOffset = G.yGoal
    r = G.rGoal
    i = int(0)
    for i in range(0,359):
        tht = i*2*m.pi/360
        xGoal.append(r*m.cos(tht) + xOffset)
        yGoal.append(r*m.sin(tht) + yOffset)
        i += 1
    plt.plot(xGoal,yGoal,'r')

    # Plotting the obstacles
    O.showObst()

    T.showStart()

    plt.title('Configuration space of Question 4')
    plt.show()


# In[4]:


# Storage of c-space boundary information for RRT collision checking.
class boundary():
    def __init__(self,xmin,xmax,ymin,ymax):
        self.xmin = xmin
        self.xmax = xmax
        self.ymin = ymin
        self.ymax = ymax


# In[5]:


class Goal():
    def __init__(self,goalRegion):
        self.xGoal = goalRegion[0]
        self.yGoal = goalRegion[1]
        self.rGoal = goalRegion[2]


# In[6]:


class RRT():
    def __init__(self,start,eps):
        self.xStart = start[0]
        self.yStart = start[1]
        self.eps = eps
        self.V = [] # Vertices
        self.V.append((self.xStart,self.yStart))
        self.E = [] # Edges
        self.VGoal = []
        self.EGoal = []

    def showStart(self):
        plt.plot(self.xStart,self.yStart,'*')

    # This generates a random point
    def randPoint(self):
        xRandom = ran.uniform(B.xmin,B.xmax)
        yRandom = ran.uniform(B.ymin,B.ymax)
        qRandom = (xRandom,yRandom)
        return qRandom

    def nearPoint(self,qRand,qTree):
        distRandNode = []     # List storing all distances between random pt and node collection
        xRand = qRand[0]
        yRand = qRand[1]
        for node in qTree:    # iterates through each node to calculate the distance from random pt
            xList = node[0]
            yList = node[1]
            distRandNode.append(m.sqrt((xRand - xList)**2 + (yRand - yList)**2))
        # Grab index of shortest distance in distance list
        iNear = distRandNode.index(min(distRandNode))
        return qTree[iNear]   # spits out closest node to random point

    def march(self,qRand,qNear,eps):
        # Goal of fxn is to create new pt qStep in direction of qRand given step size epsilon
        xRand,yRand = qRand[0],qRand[1]
        xNear,yNear = qNear[0],qNear[1]
        dist = m.sqrt((xRand - xNear)**2 + (yRand - yNear)**2)
        if dist < eps:
            eps = dist
        slope = (yRand - yNear)/(xRand - xNear)
        if xRand >= xNear:
            xStep = xNear + eps/m.sqrt(slope**2 + 1)
            yStep = slope*(xStep - xNear) + yNear
        if xRand < xNear:
            xStep = xNear - eps/m.sqrt(slope**2 + 1)
            yStep = slope*(xStep - xNear) + yNear
        return (xStep,yStep)

    def collCheck(self,qStep):
        # Goal is to see if our step node collides with any obstacles
        # First, find the nearest obstacle and check distance
        # For check, 0 = safe. 1 = unsafe.
        check = 0
        distObstStep = []
        xStep = qStep[0]
        yStep = qStep[1]
        for obstacle in O.obst:
            xObst = obstacle[0]
            yObst = obstacle[1]
            rObst = obstacle[2]
            distObst = m.sqrt((xStep - xObst)**2 + (yStep - yObst)**2)
            # I add + 1 to the radius to provide some buffer region
            if distObst < rObst + 1:
                check =1
                break
        return check

    def goalCheck(self,w):
        # Compares the distance between the goal's center and new node
        # if that distance is <= goal's radius, we've achieved the goal
        # checkGoal = 0 if not in goal, 1 if in goal
        checkGoal = 0
        xStep = w[0]
        yStep = w[1]
        dNodeToGoal = m.sqrt((xStep - G.xGoal)**2 + (yStep - G.yGoal)**2)
        if dNodeToGoal <= G.rGoal:
            checkGoal = 1
        return checkGoal

    # This is the main RRT algorithm (what Dr. Otte's pseudocode directs to)
    def buildTree(self):
        for i in range(0,5000):
            u = self.randPoint()
            v = self.nearPoint(u,self.V)
            w = self.march(u,v,self.eps)
            z = self.collCheck(w)
            # safety check for obstacle collision goes here
            if self.collCheck(w) == 0:
                self.V.append((w))
                self.E.append((v[0],v[1],w[0],w[1]))
                plt.plot([v[0],w[0]],[v[1],w[1]],'k')
                plt.plot(w[0],w[1],'or')
            if self.goalCheck(w) == 1:
                self.VGoal.append((w))
                self.EGoal.append((v[0],v[1],w[0],w[1]))
                break

    def traceback(self):
        for i in range(0,5000):
            edgeCurrent = self.EGoal[i]
            v = (edgeCurrent[0],edgeCurrent[1])
            if v == start:
                break
            for edge in self.E:
                if (edge[2],edge[3]) == v:
                    self.EGoal.append(edge)
        for edge in self.EGoal:
            plt.plot([edge[0],edge[2]],[edge[1],edge[3]],'g',linewidth=2)


# In[7]:


obstList = []
xmin = -50
xmax = 50
ymin = -50
ymax = 50
start = (39,5)
epsilon = 1
gl = (38,-8,3)
with open('obstacles.txt','r') as obst:
    obstData = csv.reader(obst,delimiter=',')
    obstCount = int(next(obstData)[0])
    for row in obstData:
        obstList.append((float(row[0]),float(row[1]),float(row[2])))

T = RRT(start,epsilon)
B = boundary(xmin,xmax,ymin,ymax)
O = obstacle(obstList)
G = Goal(gl)
T.buildTree()
T.traceback()
vis()
