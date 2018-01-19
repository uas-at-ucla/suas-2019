import math, sys, pygame, random, time
from math import *
from pygame import *

class Node(object):
    def __init__(self, point, parent):
        super(Node, self).__init__()
        self.point = point
        self.parent = parent

XDIM = 500
YDIM = 500
windowSize = [XDIM, YDIM]
delta = 10.0
GAME_LEVEL = 1
GOAL_RADIUS = 10
MIN_DISTANCE_TO_ADD = 1.0
NUMNODES = 5000
pygame.init()
fpsClock = pygame.time.Clock()
screen = pygame.display.set_mode(windowSize)
white = 255,255,255
black = 0,0,0
red = 255,0,0
green = 0,255,0
blue = 0,0,255
orange = 255,165,0

count = 0
circleObs = []

def roundup(x):
    import math
    return int(math.ceil(x / 50.0)) * 50

def dist(p1,p2):    
    return sqrt((p1[0]-p2[0])*(p1[0]-p2[0])+(p1[1]-p2[1])*(p1[1]-p2[1]))

def getMidPointsX(p1, p2, numPoints, section):
    partDistX = (p2 - p1) / (numPoints + 1)
    ptX = p1 + (partDistX * section)
    return ptX

def getMidPointsY(p1, p2, numPoints, section):
    partDistY = (p2 - p1) / (numPoints + 1)
    ptY = p1 + (partDistY * section)
    return ptY

def getAngleBetween(p1, p2):
    return atan2(p2[1] - p1[1], p2[0] - p1[0])

def point_circle_collision(p1, p2, radius):
    distance = dist(p1,p2)
    if (distance <= radius):
        return True
    return False

def step_from_to(p1,p2):
    if dist(p1,p2) < delta:
        return p2
    else:
        theta = atan2(p2[1]-p1[1],p2[0]-p1[0])
        return p1[0] + delta*cos(theta), p1[1] + delta*sin(theta)

def collides(p):
    global circleObs
    for circ in circleObs:
        if dist(p, circ.center) <= 30:
            return True
    return False

# Important to note that for pygame, (X,Y) = (0,0) is top left
# and (X,Y) = (720,500) is bottom right 
def get_random_clear(q, z, theta):
    global startX
    global startY
    global endX
    global endY
    while True:
        p = []
        if startX >= endX and startY >= endY:
            p = random.uniform(q[0] - 90, q[0] + 20), random.uniform(q[1] - 90, q[1] + 20)
        elif startX < endX and startY < endY:
            p = random.uniform(q[0] - 20, q[0] + 90), random.uniform(q[1] - 20, q[1] + 90)
        elif startX >= endX and startY < endY:
            p = random.uniform(q[0] - 90, q[0] + 20), random.uniform(q[1] - 20, q[1] + 90)
        else:
            p = random.uniform(q[0] - 20, q[0] + 90), random.uniform(q[1] - 90, q[1] + 20)
        noCollision = collides(p)
        if noCollision == False:
            if dist(p, q) > 20 and dist(p, q) < 90 and dist(p, z) < 75:
                return p

def init_obstacles():  
    global circleObs
    circleObs = []
    obs1 = pygame.Rect(100 - 25, 100 - 25, 25 * 2, 25 * 2)
    obs2 = pygame.Rect(100 - 25, 400- 25, 25 * 2, 25 * 2)
    obs3 = pygame.Rect(400 - 25, 400 - 25, 25 * 2, 25 * 2)
    obs4 = pygame.Rect(400 - 25, 100 - 25, 25 * 2, 25 * 2)
    obs5 = pygame.Rect(200 - 25, 300 - 25, 25 * 2, 25 * 2)
    obs6 = pygame.Rect(200 - 25, 200 - 25, 25 * 2, 25 * 2)
    obs7 = pygame.Rect(300 - 25, 200 - 25, 25 * 2, 25 * 2)
    obs8 = pygame.Rect(300 - 25, 300 - 25, 25 * 2, 25 * 2)

    circleObs.append(obs1)
    circleObs.append(obs2)
    circleObs.append(obs3)
    circleObs.append(obs4)
    circleObs.append(obs5)
    circleObs.append(obs6)
    circleObs.append(obs7)
    circleObs.append(obs8)
    
    for circ in circleObs:
        pygame.draw.circle(screen, black, circ.center, 25)


def reset():
    global count
    screen.fill(white)
    init_obstacles()
    count = 0

def main():
    global count
    global circleObs
    global startX
    global startY
    global endX
    global endY

    global tempStart
    global tempEnd

    global startPtCtrVal
    
    initPoseSet = False
    initialPoint = Node(None, None)
    goalPoseSet = False
    goalPoint = Node(None, None)
    currentState = 'init'

    nodes = []
    reset()
    startPtCtrVal = -1

    while True:
        if currentState == 'init':
            pygame.display.set_caption('Select Starting Point and then Goal Point')
            fpsClock.tick(10)
        elif currentState == 'goalFound':
            pygame.display.set_caption('Goal Reached')
            currentState == 'optimize'
        elif currentState == 'optimize':
            time.sleep(20)
        elif currentState == 'buildPath':
            for ptCtr in range(startPtCtrVal + 1, 50):
                tempX = getMidPointsX(startX, endX, 49, ptCtr)
                tempY = getMidPointsY(startY, endY, 49, ptCtr)
                nextTempX = getMidPointsX(startX, endX, 49, ptCtr + 1)
                nextTempY = getMidPointsY(startY, endY, 49, ptCtr + 1)        
                tempPt = nextTempX, nextTempY

                if tempPt == (endX,endY):
                    currentState = 'goalFound'
                
                if collides(tempPt) == False:
                    pygame.draw.line(screen,blue,(tempX, tempY), (nextTempX, nextTempY))
                else:
                    
                    myCounta = 0
                    tempyPt = []
                    for test in range(0,50):
                        tempyX = getMidPointsX(startX, endX, 49, ptCtr + 1 + test)
                        tempyY = getMidPointsY(startY, endY, 49, ptCtr + 1 + test)
                        tempyPt = tempyX,tempyY
                        if collides(tempyPt) == True:
                            myCounta = myCounta + 1
                        else:
                            break
                        
                    startPtCtrVal = ptCtr + myCounta 
                    
                    nodes = []
                    count = 0
                    tempStart = tempX, tempY
                    tempEnd = tempyPt
                    initialPoint = Node(tempStart, None)
                    nodes.append(initialPoint)
                    currentState = 'rrt'
                    break
            
        elif currentState == 'rrt':
            count = count+1
            if count < NUMNODES:
                foundNext = False
                while foundNext == False:
                    theta = getAngleBetween(tempStart, tempEnd)
                    rand = get_random_clear(tempStart, tempEnd, theta)
                    parentNode = nodes[0]
                    for p in nodes:
                        if dist(p.point,rand) <= dist(parentNode.point,rand):
                            if dist(p.point, rand) > 20:
                                newPoint = step_from_to(p.point,rand)
                                if collides(newPoint) == False:
                                    parentNode = p
                                    foundNext = True

                newnode = step_from_to(parentNode.point,rand)
                daNewNode = Node(newnode, parentNode)
                nodes.append(daNewNode)
                pygame.draw.line(screen,orange,parentNode.point,newnode)

                
                if dist(newnode, tempEnd) < 10:
                    currNode = daNewNode
                    while currNode.parent != None:
                        pygame.draw.line(screen,blue,currNode.point,currNode.parent.point)
                        currNode = currNode.parent
                        pygame.draw.line(screen,blue,newnode,tempEnd)
                    currentState = 'buildPath'
            else:
                return
            
        for e in pygame.event.get():
            if e.type == QUIT or (e.type == KEYUP and e.key == K_ESCAPE):
                pygame.quit()
                break
            if e.type == MOUSEBUTTONDOWN:
                if currentState == 'init':
                    if initPoseSet == False:
                        nodes = []
                        a,b = pygame.mouse.get_pos()
                        startX = roundup(a)
                        startY = roundup(b)
                        pt = startX,startY
                        initialPoint = Node(pt, None)
                        nodes.append(initialPoint)
                        initPoseSet = True
                        pygame.draw.circle(screen, red, initialPoint.point, GOAL_RADIUS)
                    elif goalPoseSet == False:
                        a,b = pygame.mouse.get_pos()
                        endX = roundup(a)
                        endY = roundup(b)
                        pt = endX,endY
                        goalPoint = Node(pt,None)
                        goalPoseSet = True
                        pygame.draw.circle(screen, green, goalPoint.point, GOAL_RADIUS)
                        currentState = 'buildPath'
                else:
                    currentState = 'init'
                    initPoseSet = False
                    goalPoseSet = False
                    reset()
        pygame.display.update()
        fpsClock.tick(100000)

if __name__ == '__main__':
    main()
