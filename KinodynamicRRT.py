# IMPORT RELEVANT CLASSES
import csv
import math as m
import matplotlib
import matplotlib.pyplot as plt
import random as ran
import numpy as np


# Obstacle class --------------------------------------------------------------
class obstacle():
    def __init__(self,xpos,ypos,radius):
        self.x = xpos
        self.y = ypos
        self.r = radius
        
    def collisionCheck(self,node,body):
        bodyX = []
        bodyY = []
        inBody = False
        for part in body:
            bodyX.append(node.x + part[0])
            bodyY.append(node.y + part[1])
        
        for i in range(0,len(bodyX) - 1):
            dist = ((bodyX[i] - self.x)**2 + (bodyY[i] - self.y)**2)**0.5
            if dist <= self.r:
                inBody = True
                break
        
        return inBody
    
    def collisionCheck2(self,x,y):
        vehRadius = 1
        inBody = False
        dist = ((x - self.x)**2 + (y - self.y)**2)**0.5
        if dist <= self.r + vehRadius:
            inBody = True
        return inBody

# Node class ------------------------------------------------------------------
'''
NOTE: This class will be modified such that the aircraft geometry is
built into every node.
'''
class node():
    def __init__(self,iden,xpos,ypos,angle = 0,vel = 0,omg = 0,accel = 0,alph = 0,tim = 0):
        self.id = iden
        self.x = xpos
        self.y = ypos
        self.tht = angle
        self.v = vel
        self.omg = omg
        self.a = accel
        self.alph = alph
        self.tim = tim
        self.neighborID = []
        self.neighbor = []
        self.parent = None
            
# Edge class ------------------------------------------------------------------
class edge():
    def __init__(self,startNode,endNode):
        self.start = startNode
        self.end = endNode
        self.xStart = startNode.x
        self.yStart = startNode.y
        self.xEnd = endNode.x
        self.yEnd = endNode.y
        # List of points (x,y,theta) that make up the trajectory between start
        # and end nodes
        self.trajX = []
        self.trajY = []
        self.trajTht = []

# Tree class ------------------------------------------------------------------
class tree():
    def __init__(self):
        self.nodeCount = 0
        self.edgeCount = 0
        self.nodeList = []
        self.edgeList = []
        
    def addNode(self,node):
        self.nodeList.append(node)
        self.nodeCount += 1
        
    def addEdge(self,edge):
        self.edgeList.append(edge)
        self.edgeCount += 1

# RRT Algorithm ---------------------------------------------------------------
class RRT():
    def __init__(self,start,goal,epsilon,xmin,xmax,ymin,ymax,obstacles,body):
        self.finalNode = None
        # Create tree
        Tree = tree()
        # Create a start node
        startNode = node(1,start[0],start[1],start[2])
        # Add start node as tree
        Tree.addNode(startNode)
        # Creates the ID for the next node
        nextid = 2
        # Boolean variable for determining if in goal or not
        inGoal = False
        # Main loop 
        while inGoal == False:
            # Initially assume node is safe until proven otherwise
            safe = True
            # Generates a random node with random x, y, and theta
            u = nodeRand(xmin,xmax,ymin,ymax,goal)
            # Locates the closest node
            v = closest(u,Tree)
            # stepping an epsilon the closest node to create a pointing node
            extender = march(u,v,epsilon,nextid)
            # finding the best trajectory that is closest to step.
            bestEdge = pathHunt(v,extender,epsilon,nextid,obstacles,body)
            if bestEdge == None:
                continue
            w = bestEdge.end
            plt.plot(bestEdge.trajX,bestEdge.trajY,'b')
            # vwTraj = trajectory(v,w)
            '''
            for obst in obstacles:
                if obst.collisionCheck(w,body) == True:
                    safe = False
            '''
            passVar = False
            for leaf in Tree.nodeList:
                if w.x == leaf.x and w.y == leaf.y:
                    passVar = True
                    break
            if passVar == True:
                continue
            if safe == True:
                print(f'Node {w.id} with coordinates {w.x}, {w.y} will be appended.')
                w.parent = v
                Tree.addNode(w)
                # NOTE: replace edge(v,w) with some other edge function
                Tree.addEdge(bestEdge)
                nextid +=1
            if goalCheck(w,body,goal) == True:
                print('The algorithm has found the goal!')
                self.finalNode = w
                inGoal = True
            
            
            if nextid == 7500:
                break # Remove when you're done designing the trajectory function
            
        drawTree(Tree)
        #drawPath(self.finalNode,startNode,Tree)
        if inGoal == True:

            writePath(self.finalNode,startNode,Tree)

# writeTree function ----------------------------------------------------------
'''
- "writeTree" writes the compoenents of each node to a .csv file.
'''
def writePath(nodeFinal,startNode,tree):
    pathX = []
    pathY = []
    pathTime = []
    pathTheta = []
    pathV = []
    pathOmega = []
    pathA = []
    pathAlpha = []
    currentParentID = nodeFinal.parent.id
    breaker = True
    while breaker == True:
        for leaf in tree.nodeList:
            if leaf.id == currentParentID:
                pathX.append(leaf.x)
                pathY.append(leaf.y)
                pathTime.append(leaf.tim)
                pathTheta.append(leaf.tht)
                pathV.append(leaf.v)
                pathOmega.append(leaf.omg)
                pathA.append(leaf.a)
                pathAlpha.append(leaf.alph)
                currentParentID = leaf.parent.id
            if currentParentID == startNode.id:
                for leaf in tree.nodeList:
                    if leaf.id == currentParentID:
                        pathX.append(leaf.x)
                        pathY.append(leaf.y)
                        pathTime.append(leaf.tim)
                        pathTheta.append(leaf.tht)
                        pathV.append(leaf.v)
                        pathOmega.append(leaf.omg)
                        pathA.append(leaf.a)
                        pathAlpha.append(leaf.alph)
                        breaker = False
    with open('paths_Q1.txt','w') as pathNode:
        pathData = csv.writer(pathNode,delimiter=',',quotechar='"',quoting=csv.QUOTE_MINIMAL)
        for i in range(0,len(pathX)):
            pathData.writerow([pathTime[i],pathX[i],pathY[i],pathTheta[i],pathV[i],pathOmega[i],pathA[i],pathAlpha[i]])
    

# pathHunt function -----------------------------------------------------------
'''
- "pathHunt" takes in a start node, end node, epsilon, next idm and obstacles
- A bunch of feasible paths are created and iterated over the entirety of the
range of permissible acceleration values.
- The path will be generated using a "trajectory" function.
- The path information containing the shortest distance between its last point
and end node.
- Should return an edge data structure
- Print statements should be commented out as they unnecessarily clog the 
console
'''

def pathHunt(startNode,endNode,epsilon,nextid,obstacles,body):
    # Define the acceleration lists
    minNodeDist = 10*10*10
    bestEdge = None
    a = np.linspace(-2,2,9)    
    alpha = np.linspace(-m.pi/2,m.pi/2,9)
    
    # Iterate through each acceleration to generate a trajectory
    for i in range(0,len(a)):
        for j in range(0,len(alpha)):
            #print(f'Current translational acceleration: {a[i]}')
            #print(f'Current angular acceleration: {alpha[j]}')
            edgePossible = trajectory(startNode,endNode,epsilon,a[i],alpha[j],obstacles,body)
            
            if edgePossible != None:
                # print(f'possible edge angular accelerations: {edgePossible.end.alph}')
                dist = ((edgePossible.xEnd - endNode.x)**2 + (edgePossible.yEnd - endNode.y)**2)**0.5
                #print(f'Min distance: {dist}')
                if dist < minNodeDist:
                    minNodeDist = dist
                    bestEdge = edgePossible
    
    #plt.plot(bestEdge.trajX,bestEdge.trajY,'b')
    return bestEdge
            

# Trajectory function ---------------------------------------------------------
'''
- "trajectory" takes in a start, end, epsilon, and both accelerations
- The velocities, theta, and positions (x,y) are started as arrays
- Euler time step is used to generate the curves at a given time step dt
- Each point is run through an obstacle checking function

'''

def trajectory(startNode,endNode,epsilon,accel,angAccel,obstacles,body):
    # Create a working edge that will be filled out later
    # Define time step
    dt = 0.1  #second
    a = accel
    alpha = angAccel
    velocity = [startNode.v]
    omega = [startNode.omg]
    theta = [startNode.tht]
    x = [startNode.x]
    y = [startNode.y]
    # Break variable when best trajectory is reached
    dontPlot = False
    distance = 0
    t = startNode.tim
    for i in range(1,10000):
        velocity.append(velocity[i-1] + a*dt)
        omega.append(omega[i-1] + alpha*dt)
        theta.append((theta[i-1] + omega[i-1]*dt) % (2*m.pi))
        x.append(x[i-1] + velocity[i-1]*dt*m.cos(theta[i-1]))
        y.append(y[i-1] + velocity[i-1]*dt*m.sin(theta[i-1]))
        distance += ((x[i] - x[i-1])**2 + (y[i] - y[i-1])**2)**0.5
        #distance = ((x[i] - startNode.x)**2 + (y[i] - startNode.y)**2)**0.5
        t += dt
        if abs(omega[i]) > m.pi/2:
            # print('Angular velocity limit exceeded.')
            dontPlot = True
            break
        if abs(velocity[i]) > 5:
            # print('Translational velocity limit exceeded.')
            dontPlot = True
            break
        inObst = False
        for obst in obstacles:
            if obst.collisionCheck2(x[i],y[i]) == True:
                inObst = True
                break
        if inObst == True:
            # print('The path hit an obstacle')
            dontPlot = True
            break
        if distance > epsilon:
            #print('Epsilon limit reached')
            #print(f'Distance from start to end of Euler pathing: {distance}')
            #print(f'Time taken to navigate to path end: {t}')
            break

    if dontPlot == False:
        newEnd = node(endNode.id,x[-1],y[-1],theta[-1],velocity[-1],omega[-1],a,alpha,t)
        edgePath = edge(startNode,newEnd)
        edgePath.trajX = x
        edgePath.trajY = y
        # plt.plot(edgePath.trajX,edgePath.trajY,'r')
        return edgePath       

# Tree-drawing function -------------------------------------------------------        
def drawTree(tree):
    for edge in tree.edgeList:
        # plt.plot([edge.xStart,edge.xEnd],[edge.yStart,edge.yEnd],'k')
        plt.plot(edge.trajX,edge.trajY,'b')
        # plt.plot(edge.xEnd,edge.yEnd,'or')

# Drawing RRT-found path ------------------------------------------------------        
def drawPath(nodeFinal,startNode,tree):
    pathX = []
    pathY = []
    pathX.append(nodeFinal.x)
    pathY.append(nodeFinal.y)
    currentParentID = nodeFinal.parent.id
    breaker = True
    while breaker == True:
        for edge in tree.edgeList:
            if node.id == currentParentID:
                pathX.append(node.x)
                pathY.append(node.y)
                currentParentID = node.parent.id
            if currentParentID == startNode.id:
                for node in tree.nodeList:
                    if node.id == currentParentID:
                        pathX.append(node.x)
                        pathY.append(node.y)
                        breaker = False    
    plt.plot(pathX,pathY,'r')

# Goal-checking function ------------------------------------------------------
def goalCheck(node,body,goal):
    # Constructing body.    
    bodyX = []
    bodyY = []
    inGoal = False
    for part in body:
        bodyX.append(node.x + part[0])
        bodyY.append(node.y + part[1])
    # Checking to see if body part exists in goal
    for i in range(0,len(bodyX) - 1):
        dist = ((bodyX[i] - goal[0])**2 + (bodyY[i] - goal[1])**2)**0.5
        if dist < goal[2]:
            inGoal = True
            break
    
    return inGoal

# Node-stepping function ------------------------------------------------------
def march(randNode,closeNode,epsilon,nextID):
    # Calculate distance from random node to closest node
    dist = ((randNode.x - closeNode.x)**2 + (randNode.y - closeNode.y)**2)**0.5
    if dist < epsilon:
        dist = epsilon
    # Slope of line between random node and closest node
    m = (randNode.y - closeNode.y)/(randNode.x - closeNode.x)
    # Following "if" statements denote the direction of the tree expansion
    if randNode.x >= closeNode.x:
        xStep = closeNode.x + epsilon/(m**2 + 1)**0.5
        yStep = m*(xStep - closeNode.x) + closeNode.y
    if randNode.x < closeNode.x:
        xStep = closeNode.x - epsilon/(m**2 + 1)**0.5
        yStep = m*(xStep - closeNode.x) + closeNode.y
    return(node(nextID,xStep,yStep,randNode.tht))
    
# Closest node function -------------------------------------------------------
def closest(randNode,tree):
    minDist = 10*10*10
    xRand = randNode.x
    yRand = randNode.y
    for leaf in tree.nodeList:
        x = leaf.x
        y = leaf.y
        # Check distance between Random node and every tree node
        dist = ((x - xRand)**2 + (y - yRand)**2)**0.5
        if dist < minDist:
            minDist = dist
            minID = leaf.id - 1
    return tree.nodeList[minID]

# Random node function --------------------------------------------------------
def nodeRand(xmin,xmax,ymin,ymax,goal):
    
    '''
    xRandom = ran.uniform(xmin,xmax)
    yRandom = ran.uniform(ymin,ymax)
    '''
    coin = ran.randint(0,1)
    if coin == 0:
        xRandom = ran.uniform(xmin,xmax)
        yRandom = ran.uniform(ymin,ymax)
    if coin == 1:
        xRandom = goal[0]
        yRandom = goal[1]
        
    return node(0,xRandom,yRandom)         
            
        
# Visualization function ------------------------------------------------------
def visualization(obstacles,xmin,xmax,ymin,ymax,start,goal):

    # Drawing the obstacles into the configuration space.
    plt.xlim([xmin,xmax])
    plt.ylim([ymin,ymax])
    
    for item in obstacles:
        i = 0
        xObs = []
        yObs = []
        # Populating plottable lists with the outer edges of obstacles
        for i in range(0,359):
            tht = i*2*m.pi/360
            xObs.append(item.r*m.cos(tht) + item.x)
            yObs.append(item.r*m.sin(tht) + item.y)
            i += 1
        plt.plot(xObs,yObs,'k')
      
    # Drawing the goal region into the configuration space.
    i = 0
    xGoal = []
    yGoal = []
    for i in range(0,359):
        tht = i*2*m.pi/360
        xGoal.append(goal[2]*m.cos(tht) + goal[0])
        yGoal.append(goal[2]*m.sin(tht) + goal[1])
    plt.plot(xGoal,yGoal,'g')
    
    # Drawing the start point into the configuration space.
    plt.plot(start[0],start[1],'*r')
    
    
    plt.title('Graph of Question 1')
    plt.show()    
    print('Drawing configuration space components completed.')

# Main Loop -------------------------------------------------------------------
def main():
    # Empty array to store obstacles
    obstList = []
    # Empty array to store body parts
    bodyList = []
    # Coordinates for screen dimensions
    xmin = -50
    xmax = 50
    ymin = -50
    ymax = 50
    # Start location (x,y)
    start = (-30,-35,-m.pi/4)
    # Step size
    epsilon = 2
    # Goal region (x,y,radius)
    gl = (-35,30,5)
    '''
    Extracting obstacle data from obstacles.txt, populating them into obstacle
    class, and storing them into array.
    '''
    with open('obstacles.txt','r') as obst:
        obstData = csv.reader(obst,delimiter=',')
        obstCount = int(next(obstData)[0])
        for row in obstData:
            obstList.append(obstacle(float(row[0]),float(row[1]),float(row[2])))
    print('Obstacle data extracted')
    # Extract robot body data
    with open('H3_robot.txt','r') as bodypart:
        bodyData = csv.reader(bodypart,delimiter=',')
        for row in bodyData:
            bodyList.append((float(row[0]),float(row[1])))
    print('Body part data extracted')
        

        
    alg = RRT(start,gl,epsilon,xmin,xmax,ymin,ymax,obstList,bodyList)
    visualization(obstList,xmin,xmax,ymin,ymax,start,gl)


# ----------------------------------------------------------------------------

if __name__ == '__main__':
    main()

