#!/usr/bin/env python
import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid
import cv2
from datetime import datetime
from math import sqrt
from heapq import heappush, heappop
import random

class PlanningNode :

    def __init__(self, node_name, start=None, end=None):

        self.nname = node_name

        rospy.init_node(self.nname)

        # Connectivity of the map. Can either be :
        # 4-connected : north, south, east, west
        # 8-connected : north, south, east, west, north-east, north-west, south-east, south-west
        self.connectivity = rospy.get_param('~connectivity', 8) 

        if (self.connectivity != 4 and self.connectivity != 8):
            raise ValueError("Connectivity must be equal to 4 or 8")
        
        rospy.Subscriber('/map', OccupancyGrid, self.callback)

        # Instanciates variables for later use
        self.mapInfo = None
        self.map = None
        self.start = start
        self.end = end

        # Set some constants for better code readability
        self.FREE_SPACE = 0
        self.UNKNOWN_SPACE = -1
        self.OCCUPIED_SPACE = 100
        self.PATH_SPACE = 50
        self.START_SPACE = 25
        self.END_SPACE = 75

        rospy.spin()


    def callback(self, msg):
        """
        Main method of the Node
        Receives a map then traces the path between
        """
        if (msg.data == self.map):
            print(rospy.get_caller_id() + "> Map already received")
            return
        # Get map and map info like width and height
        self.mapInfo = msg.info
        self.map = list(msg.data)

        # Randomly generates start and end point if not specified
        if (self.start is None):
                self.start, self.end = self.computePathExtremeties()

        # Tries to compute a path between start and end
        # If not possible randomly generates another pair
        isPathPossible = self.computePath()
        if (not(isPathPossible)):
            print(rospy.get_caller_id() + "> No path possible for ")
            print("start x : " + str(self.start[0]) + " y : " + str(self.start[1]) )
            print("end x : " + str(self.end[0]) + " y : " + str(self.end[1]) )

            self.start, self.end = self.computePathExtremeties()
            isPathPossible = self.computePath()

        # Save the map as an image file
        self.saveMap()

        # Exit the node
        rospy.signal_shutdown('Exiting node ' + rospy.get_name())

    def convertToRealWorldCoordinates(self, i):
        """
        Converts Map coordinates to (x,y) coordinates
        """
        return [i % self.mapInfo.width, i // self.mapInfo.width]

    def computePathExtremeties(self):
        """
        Assign 2 random points to start and end 
        """
        freeSpaces = []
        for i in range(len(self.map)):
            if (self.map[i] == self.FREE_SPACE):
                freeSpaces.append(i)
        
        i,j = random.sample(freeSpaces, 2)

        return self.convertToRealWorldCoordinates(i), self.convertToRealWorldCoordinates(j)

    def drawPath(self, path):
        """
        Traces back through path directory 
        and changes the values on the map list
        according to their types :
        START_SPACE, END_SPACE and PATH_SPACE 
        """

        # Changes end space
        currentSpace = self.end
        x, y = currentSpace 
        spaceCoor = self.getMapCoordinates(x, y)
        self.map[spaceCoor] = self.END_SPACE
        
        # Changes path spaces
        currentSpace = path[tuple(self.end)]
        while currentSpace != tuple(self.start):            
            x, y = currentSpace 
            spaceCoor = self.getMapCoordinates(x, y)
            self.map[spaceCoor] = self.PATH_SPACE
            
            currentSpace = path[currentSpace]

        # Changes start space
        x, y = currentSpace 
        spaceCoor = self.getMapCoordinates(x, y)
        self.map[spaceCoor] = self.START_SPACE

    def computePath(self):
        """
        Computes path between start and end point by
        conducting an A* search     
        @return boolean
        if true : path is possible and was drawn
        if false : no path is possible   
        """
        
        # All points to traverse. Init with start of path
        pointsToTraverse = []
        heappush(pointsToTraverse, (0, tuple(self.start)))

        # Dictionnary of all points traversed following this pattern :
        # key : Space ->  value : formerSpace in path
        path = {}

        # Scores for A* algo. f = g + h scores
        # g : cost of the path from start to space
        # h : heuristic which estimates the cost from space to end space
        # h is computed by self.heuristic method so no need to store it in a dict
        f = {tuple(self.start) : self.heuristic(self.start)}
        g = {tuple(self.start): 0}
        

        while pointsToTraverse != []:

            currentSpace = heappop(pointsToTraverse)[1]

            if currentSpace == tuple(self.end):
                self.drawPath(path)
                return True
                
            x, y = currentSpace

            for neighbourSpace in self.getNeighbours(x, y):

                # Try to access neighbourSpace from currentSpace
                gScoreCandidate = g[tuple(currentSpace)] + self.cost(currentSpace, neighbourSpace)
                
                # If currentSpace is better than formerly recorded space
                # we update the g score
                if neighbourSpace not in g or gScoreCandidate < g[tuple(neighbourSpace)]:
                    
                    path[tuple(neighbourSpace)] = tuple(currentSpace)
                    g[tuple(neighbourSpace)] = gScoreCandidate
                    f[tuple(neighbourSpace)] = gScoreCandidate + self.heuristic(neighbourSpace)

                    heappush(pointsToTraverse, (f[neighbourSpace], neighbourSpace))
        
        # No more points to go through and end not found so no path is possible
        return False 

    def cost(self, point1, point2):
        """
        Determines the cost between 2 neighbouring points
        """
        return np.linalg.norm(np.array(point2) - np.array(point1))
    
    def heuristic(self, space):
        """
        Determines the distance between space with (x, y) coordinate 
        and end point
        """
        x, y = space
        dx = abs(self.end[0] - x)
        dy = abs(self.end[1] - y)

        # Return Manhattan distance
        if (self.connectivity == 4):
            return dx + dy 
        
        # Return Octile distance
        return max(dx, dy) + (sqrt(2) - 1) * min(dx, dy)

    def getNeighbours(self, x, y):
        """
        Gets the neighbours of space in (x,y) coordinates according 
        to selected connectivity
        """
        neighbors = []
        directions = [[0,1], [1,0], [0,-1], [-1,0]]  
        
        # Add diagonal directions for 8-connected grid
        if self.connectivity == 8:
            directions += [[1,1], [-1,-1], [-1,1], [1,-1]]
            
        # Add all valid neighbours to list
        for dx, dy in directions:
            if self.isValidSpace(x+dx, y+dy):
                neighbors.append((x+dx, y+dy))
        
        return neighbors

    def isValidSpace(self, x, y):
        """
        Check if a space is within grid boundaries and is a free space
        """
        if (x >= self.mapInfo.width or x < 0):
            return False
        if (y >= self.mapInfo.height or y < 0):
            return False
        if (self.getSpace(x, y) != self.FREE_SPACE):
            return False
        return True

    def saveMap(self):
        """
        Saves the map with A* path
        Image is saved in terminal where node is run
        """
        image = np.zeros((self.mapInfo.height, self.mapInfo.width, 3), dtype=np.uint8)
        mapArray = np.array(self.map).reshape((self.mapInfo.height, self.mapInfo.width))

        # Convert image data to image
        image[mapArray == self.OCCUPIED_SPACE] = [0,0,0]
        image[mapArray == self.UNKNOWN_SPACE] = [128,128,128]
        image[mapArray == self.FREE_SPACE] = [255,255,255]
        
        image[mapArray == self.START_SPACE] = [0,255,0]
        image[mapArray == self.END_SPACE] = [255,0,0]  
        image[mapArray == self.PATH_SPACE] = [128,0,128]  

        # Adapt image to normal coordinates
        image = cv2.flip(image, 0)

        # Current time used for filename
        currentTime = rospy.Time.now().to_sec()
        formatted_time = datetime.fromtimestamp(currentTime).strftime("%Y-%m-%d_%H-%M-%S")

        # Save image
        cv2.imwrite("Map - " + str(formatted_time) + ".png", image)
        print(rospy.get_caller_id() + " > Image saved as 'Map - "+ str(formatted_time) + ".png' !")     


    def getSpace(self, x, y):
        """
        Returns the space of the space at real world
        coordinates (x,y)
        """
        return self.map[self.getMapCoordinates(x, y)]
    
    def getMapCoordinates(self, x, y):
        """
        Converts real world coordinates to map coordinates
        """
        return x + (y * self.mapInfo.width)


if __name__ == '__main__':
    process = PlanningNode('planning_node')
