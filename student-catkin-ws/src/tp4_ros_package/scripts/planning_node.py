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

    def __init__(self, node_name):

        self.nname = node_name

        rospy.init_node(self.nname)

        # Connectivity of the map. Can either be :
        # 4-connected : north, south, east, west
        # 8-connected : north, south, east, west, north-east, north-west, south-east, south-west
        self.connectivity = rospy.get_param('~connectivity', 4) 

        if (self.connectivity != 4 and self.connectivity != 8):
            raise ValueError("Connectivity must be equal to 4 or 8")
        
        rospy.Subscriber('/map', OccupancyGrid, self.callback)

        # Instanciates variables for later use
        self.mapInfo = None
        self.map = None
        self.start = None
        self.end = None

        # Set some constants for better code readability
        self.FREE_SPACE = 0
        self.UNKNOWN_SPACE = -1
        self.OCCUPIED_SPACE = 100
        self.PATH_SPACE = 50
        self.START_SPACE = 25
        self.END_SPACE = 75

        rospy.spin()

        print('Exiting node ' + rospy.get_name()) # This will only be executed if 'rospy.spin()' finishes, after having pressed Ctrl+c


    def callback(self, msg):
        """
        Main method of the 
        """
        if (msg.data == self.map):
            print(rospy.get_caller_id() + "> Map already received")
            return
        
        self.mapInfo = msg.info
        self.map = msg.data

        self.start, self.end = self.computePathExtremeties()

        isPathPossible = self.computePath()


        if (not(isPathPossible)):
            print(rospy.get_caller_id() + "> No path possible for ")
            print("start x : " + str(self.start[0]) + " y : " + str(self.start[1]) )
            print("end x : " + str(self.end[0]) + " y : " + str(self.end[1]) )
            return 

        self.saveMap()

    def convertToRealWorldCoordinates(self, i):
        return [i % self.mapInfo.width, i // self.mapInfo.width]

    def computePathExtremeties(self):
        freeSpaces = []
        for i in range(len(self.map)):
            if (self.map[i] == self.FREE_SPACE):
                freeSpaces.append(i)
        
        i,j = random.sample(freeSpaces, 2)

        return self.convertToRealWorldCoordinates(i), self.convertToRealWorldCoordinates(j)

    def drawPath(self, path):
        currentSpace = self.end
        x, y = currentSpace 
        spaceCoor = self.getMapCoordinates(x, y)
        self.map[spaceCoor] = self.END_SPACE

        currentSpace = path[self.end]

        while currentSpace != self.start:            
            x, y = currentSpace 
            spaceCoor = self.getMapCoordinates(x, y)
            self.map[spaceCoor] = self.PATH_SPACE
            
            currentSpace = path[currentSpace]

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
        heappush(pointsToTraverse, (0, self.start))

        # Dictionnary of all points traversed following this pattern :
        # key : Space ->  value : formerSpace in path
        path = {}

        # Scores for A* algo. f = g + h scores
        # g : cost of the path from start to space
        # h : heuristic which estimates the cost from space to end space
        # h is computed by self.heuristic method so no need to store it in a dict
        f = {self.start : self.heuristic(self.start)}
        g = {self.start: 0}
        

        while pointsToTraverse != []:

            currentSpace = heappop(pointsToTraverse)[1]
            
            if currentSpace == self.end:
                self.drawPath(path)
                return True
                
            x, y = currentSpace

            for neighbourSpace in self.getNeighbours(x, y):

                # Try to access neighbourSpace from currentSpace
                gScoreCandidate = g[currentSpace] + self.cost(currentSpace, neighbourSpace)
                
                # If currentSpace is better than formerly recorded space
                # we update the g score
                if neighbourSpace not in g or gScoreCandidate < g[neighbourSpace]:
                    
                    path[neighbourSpace] = currentSpace
                    g[neighbourSpace] = gScoreCandidate
                    f[neighbourSpace] = gScoreCandidate + self.heuristic(neighbourSpace)

                    heappush(pointsToTraverse, (f[neighbourSpace], neighbourSpace))
        
        # No more points to go through and end not found so no path is possible
        return False 

    def cost(self, point1, point2):
        """
        Determines the cost between 2 neighbouring points
        """
        return np.linalg.norm(np.array(point2) - np.array(point1))
    
    def heuristic(self, x, y):
        """
        Determines the distance between space with (x, y) coordinate 
        and end point
        """
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
                neighbors.append([x+dx, y+dy])
        
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
        """
        image = np.zeros((self.mapInfo.height, self.mapInfo.width, 3), dtype=np.uint8)
        
        # Convert image data to image
        image[self.map == self.OCCUPIED_SPACE] = [0,0,0]
        image[self.map == self.UNKNOWN_SPACE] = [128,128,128]
        image[self.map == self.FREE_SPACE] = [255,255,255]
        
        image[self.map == self.START_SPACE] = [0,255,0]
        image[self.map == self.END_SPACE] = [255,0,0]  
        image[self.map == self.PATH_SPACE] = [128,0,128]  

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
