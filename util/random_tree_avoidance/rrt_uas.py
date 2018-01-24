import math
import sys
import random
import time
from math import *
from matplotlib import pyplot as plt
import numpy as np
import matplotlib


class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent


class RRT:
    def __init__(self, startCoord, endCoord, obstacleList):
        self.DELTA = 10
        self.NUMNODES = 5000

        startCoord = startCoord.split(',')
        self.START_XCOORD = float(startCoord[0])
        self.START_YCOORD = float(startCoord[1])

        endCoord = endCoord.split(',')
        self.END_XCOORD = float(endCoord[0])
        self.END_YCOORD = float(endCoord[1])

        self.COUNT = 0

        self.OBS_XCOORD = []
        self.OBS_YCOORD = []
        self.OBS_RADIUS = []
        for temp in range(0, len(obstacleList)):
            obs = obstacleList[temp]
            obs = obs.split(',')
            self.OBS_XCOORD.append(float(obs[0]))
            self.OBS_YCOORD.append(float(obs[1]))
            self.OBS_RADIUS.append(float(obs[2]))

        # NUM_SEGMENTS would have to be a larger constant when used on the drone
        self.NUM_SEGMENTS = 100
        self.ROUTE = []

    # function that will return route
    def returnRoute(self):
        return self.ROUTE

    def outputRoute(self):
        print("-------------------")
        print("Path has been found")
        print("-------------------")

        x = list()
        y = list()
        obstacles = list()

        for rt in self.ROUTE:
            x.append(rt[0])
            y.append(rt[1])
            print(rt)

        for i in range(0, len(self.OBS_RADIUS)):
            obstacle = plt.Circle( \
                    (self.OBS_XCOORD[i], self.OBS_YCOORD[i]), \
                    self.OBS_RADIUS[i], \
                    color='r')
            obstacles.append(obstacle)

        fig, ax = plt.subplots()

        ax.scatter(x, y)
        for obstacle in obstacles:
            ax.add_artist(obstacle)

        plt.show()

    def getDistance(self, p1, p2):
        return float(
            sqrt(((p2[0] - p1[0]) * (p2[0] - p1[0])) + (
                (p2[1] - p1[1]) * (p2[1] - p1[1]))))

    def getSegX(self, x1, x2, numPoints, segment):
        partDist = ((x2 - x1) / float((numPoints + 1)))
        segX = x1 + (partDist * segment)
        return segX

    def getSegY(self, y1, y2, numPoints, segment):
        partDist = ((y2 - y1) / float((numPoints + 1)))
        segY = y1 + (partDist * segment)
        return segY

    def getTheta(self, p1, p2):
        return float(atan2(p2[1] - p1[1], p2[0] - p1[0]))

    def stepFromTo(self, p1, p2):
        if self.getDistance(p1, p2) < self.DELTA:
            return p2
        else:
            theta = self.getTheta(p1, p2)
            return (p1[0] + (self.DELTA * float(cos(theta)))), (
                p1[1] + (self.DELTA * float(sin(theta))))

    def collides(self, p):
        for obsID in range(0, len(self.OBS_XCOORD)):
            obsCenter = []
            obsCenter = self.OBS_XCOORD[obsID], self.OBS_YCOORD[obsID]

            if self.getDistance(p, obsCenter) <= (self.OBS_RADIUS[obsID] + 10):
                return True
        return False

    def getRandomPt(self, p2, pTemp, dist):
        while True:
            p1 = []
            if self.START_XCOORD >= self.END_XCOORD and self.START_YCOORD >= self.END_YCOORD:
                p1 = random.uniform(p2[0] - dist * 9 / 7,
                                    p2[0] + dist * 2 / 7), random.uniform(
                                        p2[1] - dist * 9 / 7,
                                        p2[1] + dist * 2 / 7)
            elif self.START_XCOORD < self.END_XCOORD and self.START_YCOORD < self.END_YCOORD:
                p1 = random.uniform(p2[0] - dist * 2 / 7,
                                    p2[0] + dist * 9 / 7), random.uniform(
                                        p2[1] - dist * 2 / 7,
                                        p2[1] + dist * 9 / 7)
            elif self.START_XCOORD >= self.END_XCOORD and self.START_YCOORD < self.END_YCOORD:
                p1 = random.uniform(p2[0] - dist * 9 / 7,
                                    p2[0] + dist * 2 / 7), random.uniform(
                                        p2[1] - dist * 2 / 7,
                                        p2[1] + dist * 9 / 7)
            else:
                p1 = random.uniform(p2[0] - dist * 2 / 7,
                                    p2[0] + dist * 9 / 7), random.uniform(
                                        p2[1] - dist * 9 / 7,
                                        p2[1] + dist * 2 / 7)

            collision = not self.collides(p1)
            if collision:
                if self.getDistance(
                        p1, p2) > (2 / 7) * dist and self.getDistance(
                            p1, p2) < (9 / 7) * dist and self.getDistance(
                                p1, pTemp) < dist:
                    return p1

    def reset(self):
        self.COUNT = 0

    def findPath(self):
        currentState = 'goStraight'
        nodes = []
        self.reset()
        startPtCtrVal = -1

        while True:
            if currentState == 'goStraight':
                for seg in range(startPtCtrVal + 1, self.NUM_SEGMENTS):
                    tempX = self.getSegX(self.START_XCOORD, self.END_XCOORD,
                                         self.NUM_SEGMENTS - 1, seg)
                    tempY = self.getSegY(self.START_YCOORD, self.END_YCOORD,
                                         self.NUM_SEGMENTS - 1, seg)
                    nextTempX = self.getSegX(self.START_XCOORD,
                                             self.END_XCOORD,
                                             self.NUM_SEGMENTS - 1, seg + 1)
                    nextTempY = self.getSegY(self.START_YCOORD,
                                             self.END_YCOORD,
                                             self.NUM_SEGMENTS - 1, seg + 1)
                    nextTempPt = nextTempX, nextTempY

                    if nextTempPt == (self.END_XCOORD, self.END_YCOORD):
                        currentState = 'goalFound'

                    if self.collides(nextTempPt) == False:
                        self.ROUTE.append(nextTempPt)
                    else:

                        counter = 0
                        temp2Pt = []
                        for test in range(0, self.NUM_SEGMENTS):
                            temp2X = self.getSegX(
                                self.START_XCOORD, self.END_XCOORD,
                                self.NUM_SEGMENTS - 1, seg + 1 + test)
                            temp2Y = self.getSegY(
                                self.START_YCOORD, self.END_YCOORD,
                                self.NUM_SEGMENTS - 1, seg + 1 + test)
                            temp2Pt = temp2X, temp2Y
                            if self.collides(temp2Pt) == True:
                                counter = counter + 1
                            else:
                                break

                        startPtCtrVal = seg + counter

                        nodes = []
                        self.reset()
                        tempStart = tempX, tempY
                        tempEnd = temp2Pt
                        initialPoint = Node(tempStart, None)
                        nodes.append(initialPoint)
                        currentState = 'buildTree'
                        break

            elif currentState == 'buildTree':
                self.COUNT = self.COUNT + 1
                if self.COUNT < self.NUMNODES:
                    dist = self.getDistance(tempStart, tempEnd)
                    foundNext = False
                    while foundNext == False:
                        theta = self.getTheta(tempStart, tempEnd)
                        rand = self.getRandomPt(tempStart, tempEnd, dist)
                        parentNode = nodes[0]
                        for p in nodes:
                            if self.getDistance(p.point,
                                                rand) <= self.getDistance(
                                                    parentNode.point, rand):
                                if self.getDistance(p.point, rand) > 10:
                                    newPoint = self.stepFromTo(p.point, rand)
                                    if self.collides(newPoint) == False:
                                        parentNode = p
                                        foundNext = True

                    newNodePoint = self.stepFromTo(parentNode.point, rand)
                    newNode = Node(newNodePoint, parentNode)
                    nodes.append(newNode)

                    if self.getDistance(newNodePoint, tempEnd) < 10:
                        currNode = newNode
                        rrtRoute = []
                        while currNode.parent != None:
                            rrtRoute.append(currNode.point)
                            currNode = currNode.parent
                        rrtRoute.reverse()
                        rrtRoute.append(tempEnd)

                        for z in range(0, len(rrtRoute)):
                            self.ROUTE.append(rrtRoute[z])

                        currentState = 'goStraight'
                else:
                    return

            else:
                break


def main():
    startCoord = raw_input("Enter Start Coordinates in x,y Format:   ")
    endCoord = raw_input("Enter End Coordinates in x,y Format:   ")
    numObs = int(raw_input("Enter Number of Obstacles Needed:   "))

    obs = []
    for x in range(0, numObs):
        obsCoord = raw_input("Enter Obstacle #" + str(x + 1) +
                             " Coordinates and Radius in x,y,r Format:   ")
        obs.append(obsCoord)

    rrt = RRT(startCoord, endCoord, obs)
    rrt.findPath()
    # for competition, drone would have to call rrt.returnRoute() instead of rrt.outputRoute()
    rrt.outputRoute()


if __name__ == '__main__':
    main()
