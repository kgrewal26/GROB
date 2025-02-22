import numpy as np
from math import sqrt
import matplotlib.pyplot as plt

from scipy.spatial import KDTree
from scipy.interpolate import interp1d

class Vertex():
    def __init__(self, x, y, parent = None, cost = float('inf'), children = []):
        self.x = x
        self.y = y
        self.parent = parent
        self.cost = cost
        self.children = children
    
    def calcCost(self, other):
        return sqrt((self.x - other.x)**2 + (self.y - other.y)**2)
    
    def addChild(self, childToAdd):
        self.children.append(childToAdd)

    def connectToParent(self, parent):
        self.parent = parent
        parent.addChild(self)

        # Total cost is the cost from parent to this position, plus cost of parent
        extraCost = self.calcCost(parent)
        self.cost = parent.cost + extraCost

    def printVertex(self):
        print("Vertex: (" + str(self.x) + "," + str(self.y) + "), with cost: " + str(self.cost))

class RRT_Path():
    def __init__(self, startX, startY, endX, endY, lengthX, lengthY, resolution, obstacle_list):
        self.start = [startX, startY]
        self.end = [endX, endY]
        self.lengthX = lengthX
        self.lengthY = lengthY

        # Create the start vertex - dont add the end vertex to the tree yet
        # Exit condition will be when the new vertex, is close enough to the end vertex
        self.startVertex = Vertex(startX, startY, cost = 0)
        self.endVertex = Vertex(endX, endY)

        # Adds to this through random sampling later
        self.tree = [self.startVertex]

        # The step size between samples
        self.deltaQ = 3

        self.obstacle_list = obstacle_list

        # Convert to a point format so it can be stored as a KD tree 
        obstacles = np.column_stack((obstacle_list[0], obstacle_list[1]))
        
        # Store as a KD tree so it can easily query for nearest obstacle
        self.obstacle_KD_tree = KDTree(obstacles)

        self.clearance_radius = 3 # TODO: feed this in as cartesian units and convert to indices

    def isValidVertex(self, new_x, new_y):
        # Check for obstacle clearance
        distanceToObstacle, obstacleIndex = self.obstacle_KD_tree.query(np.array([new_x, new_y]).T)

        # Add any other requirements here

        return (distanceToObstacle > self.clearance_radius)

    # Rewires any existing tree vertex's to go through this new node if its cheaper
    # def rewireTree(self, newVertex):
    #     for vertex in self.tree:
    #         newCost = newVertex.cost + newVertex.calcCost(vertex)
    #         if (newCost < vertex.cost):

    # This will only check the last element in the tree. The idea is you call this after each vertex is added to the tree.
    def canReachEnd(self):
        lastVertex = self.tree[-1]

        distanceToEnd = lastVertex.calcCost(self.endVertex)

        # We've reached the end if we can make a connection to the goal vertex
        withinRange = distanceToEnd <= self.deltaQ

        # Ensure that there is no collision
        noCollision = self.isValidVertex(self.endVertex.x, self.endVertex.y)

        # We've reached the end - add it to the tree
        if (withinRange and noCollision):
            self.endVertex.connectToParent(lastVertex)
            self.tree.append(self.endVertex)
            
            return True

        return False

    def addNewVertex(self):

        rng = np.random.default_rng()

        # Generate a random sample - q_rand
        # TODO: should this generate integers?
        x = rng.integers(low=0, high=self.lengthX, size=1)
        y = rng.integers(low=0, high=self.lengthY, size=1)
        randVertex = Vertex(x, y)

        # Find the closest existing vertex to this randomly generated one
        closestVertex = min(self.tree, key=lambda vertex: randVertex.calcCost(vertex))

        # Generate the new sample to be along the path from closest->random, but only deltaQ along the way.
        # Rather than finding the angle with atan (has ambiguity), scale the differentials between the two points.
        dx = x - closestVertex.x
        dy = y - closestVertex.y

        # Scale down to a distance of deltaQ away from the closest point. Only do this if we're further than deltaQ.
        # Connect them directly otherwise
        dist = closestVertex.calcCost(randVertex)
        if (dist > self.deltaQ):
            dx = dx / dist * self.deltaQ
            dy = dy / dist * self.deltaQ

        new_x = closestVertex.x + dx
        new_y = closestVertex.y + dy

        # Add the vertex to the tree if it is a valid connection (obstacle-free)
        if (self.isValidVertex(new_x, new_y)):
            newVertex = Vertex(new_x, new_y)
            newVertex.connectToParent(closestVertex)
            self.tree.append(newVertex)

    def createPath(self):
        x = []
        y = []

        iterations = 0
        while (not self.canReachEnd() and (iterations < 1500)):
            self.addNewVertex()
            x.append(self.tree[-1].x)
            y.append(self.tree[-1].y)
            iterations += 1
        
        # Reverse through the tree to get the path
        vertex = self.endVertex
        path = []
        x_path = []
        y_path = []
        while (vertex is not None):
            path.append(vertex)
            x_path.append(vertex.x)
            y_path.append(vertex.y)

            vertex = vertex.parent

        path = path[::-1] # Reverse path
        
        plt.scatter(x, y, color="red")
        plt.scatter(self.obstacle_list[0], self.obstacle_list[1], color="blue")
        plt.scatter(x_path, y_path, color="green")
        plt.show()

        return path


        


