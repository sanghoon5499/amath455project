import random
import math
import pygame

class RRTMap:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        self.start = start
        self.goal = goal
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions
        
        # window settings
        self.MapWindowName = 'RRT Path Planning'
        pygame.display.set_caption(self.MapWindowName)
        self.map = pygame.display.set_mode((self.Mapw, self.Maph))
        self.map.fill((255, 255, 255))  # Fill the map with white color
        self.nodeRad = 2
        self.nodeThickness = 0
        self.edgeThickness = 1
        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum
        # colors
        self.grey = (70, 70, 70)
        self.Blue = (0, 0, 255)
        self.Green = (0, 255, 0)
        self.Red = (255, 0, 0)
        self.White = (255, 255, 255)

    def drawMap(self, obstacles):
        # Draw the start and goal nodes
        pygame.draw.circle(self.map, self.Green, self.start, self.nodeRad + 5, 0)
        pygame.draw.circle(self.map, self.Red, self.goal, self.nodeRad + 20, 1)

        # Draw the obstacles
        self.drawObs(obstacles)

    def drawObs(self, obstacles):
        # Draw each obstacle as a rectangle
        obstaclesList = obstacles.copy()
        while len(obstaclesList) > 0:
            obstacle = obstaclesList.pop(0)
            pygame.draw.rect(self.map, self.grey, obstacle)

    def drawPath(self, path):
        # Draw the path as a series of red circles
        for node in path:
            pygame.draw.circle(self.map, self.Red, node, self.nodeRad + 3, 0)



class RRTGraph:
    def __init__(self, start, goal, MapDimensions, obsdim, obsnum):
        (x, y) = start
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.MapDimensions = MapDimensions
        self.Maph, self.Mapw = self.MapDimensions
        self.x = []
        self.y = []
        self.parent = []

        # Initialize the tree with the start node
        self.x.append(x)
        self.y.append(y)
        self.parent.append(0)

        # Initialize obstacles
        self.obstacles = []
        self.obsdim = obsdim
        self.obsNumber = obsnum

        # Path-related attributes
        self.goalstate = None
        self.path = []

    def makeRandomRect(self):
        # Generate random coordinates for the upper corner of a rectangle
        upperconrnerx = int(random.uniform(0, self.Mapw - self.obsdim))
        upperconrnery = int(random.uniform(0, self.Maph - self.obsdim))
        return (upperconrnerx, upperconrnery)

    def makeobs(self):
        # Create obstacles and ensure they do not overlap
        obs = []
        max_iter = 1000
        for i in range(0, self.obsNumber):
            rectang = None
            for j in range(max_iter):
                upper = self.makeRandomRect()
                rectang = pygame.Rect(upper, (self.obsdim, self.obsdim))

                # Check collision with existing obstacles only (not start/goal)
                if any(rectang.colliderect(existing_obstacle) for existing_obstacle in obs):
                    continue  # Skip to next iteration if collides with existing obstacles

                obs.append(rectang)  # Add valid obstacle to the list
                break
            if j == max_iter - 1:
                print("Unable to place all obstacles")
                break

        self.obstacles = obs.copy()
        return obs
    

