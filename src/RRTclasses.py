import math
import random
import numpy
import pygame


#init: takes start and goal co-ordinates, dimension of maps, dimension of obstacle, number of obstacles

class RRTmap:
    def __init__(self, start, goal, MapDimensions, ObsDim, ObsNum, extracted_dim):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.MapHeight, self.MapWidth = self.MapDimensions
        self.extracted_dim = extracted_dim

        #windowName
        self.MapWindowName = 'RRT Path Planning'
        pygame.display.set_caption(self.MapWindowName) #display window name
        self.Map = pygame.display.set_mode((self.MapWidth,self.MapHeight)) #display window size
        self.Map.fill((0, 0, 0)) #color to fill map
        self.nodeRadius = 2   # radius
        self.nodeThickness = 0  # thickness
        self.edgeThickness = 1  # edge thickness

        self.obstacles = []   # list to store obstacles
        self.ObsDim = ObsDim    # dimension of obstacle
        self.ObsNum = ObsNum    # number of Obstacles

        # colors
        self.grey = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.White = (255, 255, 255)

    def drawMap(self, obstacles): # function to draw start and end point
        pygame.draw.circle(self.Map, self.Green, self.start, self.nodeRadius + 5, 0)
        pygame.draw.circle(self.Map, self.Green, self.goal, self.nodeRadius + 20, 1)
        self.drawObs(obstacles)

    def drawPath(self, path):   # function to draw path
        for node in path:
            pygame.draw.circle(self.Map, self.Red, node, self.nodeRadius+3, 0)

    def drawObs(self,obstacles):    # function to draw Obstacles
        obstacle_list = obstacles.copy()
        while len(obstacle_list) > 0:
            obstacle = obstacle_list.pop(0)
            pygame.draw.rect(self.Map, self.grey, obstacle)




class RRTGraph:
    def __init__(self, start, goal, MapDimensions, ObsDim, ObsNum, extracted_dim):
        (x, y) = start #start dimension
        self.start = start
        self.goal = goal
        self.extracted_dim = extracted_dim
        self.reachedgoal = False
        self.MapHeight, self.MapWidth = MapDimensions
        self.x = []     # x co-ordinate of nodes
        self.y = []     # y co-ordinate of nodes
        self.parent = []    # parent of nodes
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        # the obstacles
        self.obstacles = []
        self.ObsDim = ObsDim
        self.ObsNum = ObsNum

        #path
        self.goalstate = None
        self.path = []

    def makeRandomRect(self):    #random co-ordinates for rectangle, will change later to take input from openCV
        uppercornerx = int(random.uniform(0, self.MapWidth - self.ObsDim))
        uppercornery = int(random.uniform(0, self.MapHeight - self.ObsDim))
        return (uppercornerx, uppercornery)

    def makeextractedobs(self):
        obs = []
        for i in range(0, self.ObsNum):
            rectangle = None
            startgoalcol = True   # flag to indicate if start and goal positions are inside a newly created obstacle
            while startgoalcol:
                upper = (self.extracted_dim[0], self.extracted_dim[1])  #random upper corner
                rectangle = pygame.Rect(upper, (self.extracted_dim[2], self.extracted_dim[3]))  # turns upper corner into full rectangle
                if rectangle.collidepoint(self.start) or rectangle.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
        #while loop is set to true, as soon as an obstacle which does not collide with start/end point is created, the flag is set to false and loop is terminated
            obs.append(rectangle)
        self.obstacles = obs.copy()
        return obs     #returns list of obstacle


    def makeObs(self):
        obs = []

        for i in range(0, self.ObsNum):
            rectangle = None
            startgoalcol = True   # flag to indicate if start and goal positions are inside a newly created obstacle
            while startgoalcol:
                upper = self.makeRandomRect()  #random upper corner
                rectangle = pygame.Rect(upper, (self.ObsDim, self.ObsDim))  # turns upper corner into full rectangle
                if rectangle.collidepoint(self.start) or rectangle.collidepoint(self.goal):
                    startgoalcol = True
                else:
                    startgoalcol = False
        #while loop is set to true, as soon as an obstacle which does not collide with start/end point is created, the flag is set to false and loop is terminated
            obs.append(rectangle)
        self.obstacles = obs.copy()
        return obs     #returns list of obstacle

    def add_node(self, n, x, y):  # method is used to add nodes
        self.x.insert(n,x)
        self.y.append(y)

    def remove_node(self, n):     # popes x and y co-ordinate from tree
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self, parent, child):  # inserts parent of child in parent list
        self.parent.insert(child, parent)  # child is index, parent is element

    def remove_edge(self, n):   # pops edge using index of child
        self.parent.pop(n)

    def number_of_nodes(self):  #lates node added to tree or know exact number of nodes
        return len(self.x)

    def distance(self, n1, n2):     # distance between two nodes by indexes
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        px = (float(x1) - float(x2))**2
        py = (float(y1) - float(y2))**2
        return (px+py)**(0.5)       # euclidean distance

    def random_sample(self):    # generates random sample between 0 and map height and map width
        x = int(random.uniform(0, self.MapWidth))
        y = int(random.uniform(0, self.MapHeight))
        return x, y





    def isFree(self):       # test to see if random sample is in free space or not
        n = self.number_of_nodes() - 1   # newly added node is at last place
        (x, y) = (self.x[n], self.y[n])
        obs = self.obstacles.copy()    # copy obstacle in temporary list
        while len(obs) > 0:
            rectangle = obs.pop(0)
            if rectangle.collidepoint(x,y):
                self.remove_node(n)
                return False
        return True

    def crossObstacle(self, x1, x2, y1, y2):  # check if connection between two nodes collide with any obstacle # how do you test a line for collision, you can make interpolations between nodes and check if any interpolation collides with any obstacle
        obs = self.obstacles.copy()
        while(len(obs) > 0):
            rectangle = obs.pop(0)
            for i in range(0, 101):
                u = i/100
                x = x1*u + x2*(1-u)
                y = y1*u + y2*(1-u)
                if rectangle.collidepoint(x, y):
                    return True
        return False

    def connect(self, n1, n2):
        (x1, y1) = (self.x[n1], self.y[n1])
        (x2, y2) = (self.x[n2], self.y[n2])
        if self.crossObstacle(x1,x2,y1,y2):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1,n2)
            return True

    def nearest(self, n):
        mindistance = self.distance(0, n)
        nnear = 0    #id of closest node so far
        for i in range(0, n):
            if self.distance(i, n) < mindistance:
                mindistance = self.distance(i,n)
                nnear = i
        return nnear

    def step(self, nnear, nrand, distancemax = 35):   #will create a new node between new node and nearest node with step of 35
        d = self.distance(nnear, nrand)
        if d > distancemax:
            u = distancemax/d
            (xnear, ynear) = (self.x[nnear], self.y[nnear])
            (xrand, yrand) = (self.x[nrand], self.y[nrand])
            (px, py) =  (xrand-xnear, yrand-ynear)
            theta = math.atan2(py,px)       #angle to calculate new distance by using angle for step size on a line
            (x,y) = (int(xnear + distancemax * math.cos(theta)), (int(ynear + distancemax * math.sin(theta))))
            self.remove_node(nrand)
            if abs(x-self.goal[0])<35 and abs(y-self.goal[1])<35:
                self.add_node(nrand, self.goal[0], self.goal[1])
                self.goalstate = nrand
                self.reachedgoal = True
            else:
                self.add_node(nrand, x, y)

    # higher percentage of iteration should be expansion and smaller should be biasing
    def bias(self, n_goal):  # as random numbers are not truly random, we need to add a degree of bias to the samples
        n = self.number_of_nodes()
        self.add_node(n, n_goal[0], n_goal[1])
        nnear = self.nearest(n)
        self.step(nnear,n)
        self.connect(nnear,n)
        return self.x, self.y, self.parent

    def expand(self):
        n = self.number_of_nodes()
        x,y =self.random_sample()
        self.add_node(n,x,y)
        if self.isFree():
            xnearest = self.nearest(n)
            self.step(xnearest,n)
            self.connect(xnearest,n)
        return self.x, self.y, self.parent


    def path_to_goal(self):
        if self.reachedgoal:
            self.path = []
            self.path.append(self.goalstate)
            newpos = self.parent[self.goalstate]
            while(newpos != 0):
                self.path.append(newpos)
                newpos = self.parent[newpos]
            self.path.append(0)
        return self.reachedgoal

    def getPathCords(self):  #extract path nodes whenever we need them
        path_cords = []
        for node in self.path:
            x, y = (self.x[node], self.y[node])
            path_cords.append((x, y))
        return path_cords



    def cost(self):
        pass