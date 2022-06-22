#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from time import sleep
import sys
from math import sqrt, cos, sin, radians, atan2, floor, degrees

class Node:
    
    def __init__(self, start, env, goal, stepSize, parent=None, action=None):
        self.env = env
        self.parent = parent
        self.goal = goal
        self.action = action

        if parent is not None:
            step = sqrt((env[0] - parent.env[0]) ** 2 + (env[1] - parent.env[1]) ** 2)
            self.g = parent.g + step
        else:
            self.g = 0
        
        self.fScore = self.g + sqrt((env[0] - goal[0]) ** 2 + (env[1] - goal[1]) ** 2) + (
                (env[2] - floor(atan2((goal[1] - start[1]), (goal[0] - start[0])))) / 30)  # * (step / 5)

    def path(self):
        node, p = self, []
        while node:
            p.append(node)
            node = node.parent
        return reversed(p)

    def actions(self):
        if self.action is None:
            return self.env.getActionSet()
        else:
            return self.env.getActionSet(self.action)


class Environment:
    
    def __init__(self, currentPosition, clearance):
        self.currentPosition = currentPosition
        self.clearance = clearance

    
    def insideCircles(self,position):
        if (position[0] - 51) ** 2 + (position[1] - 51) ** 2 <= (10 + self.clearance) ** 2:
            return True
        elif (position[0] - 31) ** 2 + (position[1] - 21) ** 2 <= (10 + self.clearance) ** 2:
            return True     
        elif (position[0] - 71) ** 2 + (position[1] - 21) ** 2 <= (10 + self.clearance) ** 2:
            return True
        elif (position[0] - 71) ** 2 + (position[1] - 81) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

    def insideSquares(self,position):
        if position[0] > 23.5 - self.clearance and position[0] < 38.5 + self.clearance and position[1] < 88.5 + self.clearance and position[1] > 73.5 - self.clearance:
            return True
        elif position[0] > 3.5 - self.clearance and position[0] < 18.5 + self.clearance and position[1] < 58.5 + self.clearance and position[1] > 43.5 - self.clearance:
            return True
        elif position[0] > 83.5 - self.clearance and position[0] < 98.5 + self.clearance and position[1] < 58.5 + self.clearance and position[1] > 43.5 - self.clearance:
            return True
        else:
            return False

    def outsideMap(self, position):
        if position[0] > 1.0 + self.clearance and position[0] < 101.0 - self.clearance and position[1] < 101.0 - self.clearance and position[1] > 1.0 + self.clearance:
            return False
        else:
            return True

    def checkObstacleSpace(self, position):
        flag = True
        if self.insideCircles(position):
            flag = False
        if self.insideSquares(position):
            flag = False
        if self.outsideMap(position):
            flag = False
        return flag

    def getActionSet(self, start, node, stepSize):
        actions = []
        actionStr = ['1','2','3','4','5','6','7','8']
        for i in actionStr:
            temp = self.performAction(start, i, stepSize, node)
            if temp is not None:
                actions.append(temp)

        return actions
    
    
    def performAction(self, start, val, stepSize, node):
        temp = None
        actionStr = ['1','2','3','4','5','6','7','8']
        steps = [[0, stepSize[1]],[stepSize[0], stepSize[1]],[stepSize[1], stepSize[1]],[stepSize[1], stepSize[0]],[stepSize[1], 0],[0, stepSize[0]],[stepSize[0], stepSize[0]],[stepSize[0], 0]]
        
        for i in range(len(actionStr)):
            if val == actionStr[i]:
                notInsideObs = True
                x = self.currentPosition[0]
                y = self.currentPosition[1]
                theta = self.currentPosition[2]
                for c in range(100):
                    step1 = steps[i][0]
                    step2 = steps[i][1]
                    x, y, theta = self.plot_curve(x, y, theta, step1,step2)
                    if not self.checkObstacleSpace([x, y]):
                        notInsideObs = False
                if notInsideObs:
                    temp = Node(start, [x, y, theta], node.goal, stepSize, node, actionStr[i])
                    return temp
    
    def checkAngle(self, theta):
        if theta >= 360:
            theta -= 360
        if theta < 0:
            theta = 360 + theta
        return theta
    
    def plot_curve(self, x, y, theta, rpm1, rpm2):
        r = 0.033
        l = 0.160
        rpm1 /= 10
        rpm2 /= 10
        x += r / 2 * (rpm1 + rpm2) * cos(radians(theta))
        y += r / 2 * (rpm1 + rpm2) * sin(radians(theta))
        theta += r / l * (rpm1 - rpm2)
        theta = self.checkAngle(theta)
        return x, y, theta
        
class Planner:
    def __init__(self, start, goal, clearance, stepSize):
        self.start = start
        self.goal = goal
        self.clearance = clearance
        self.stepSize = stepSize

    def __str__(self):
        return str(self.start) + str(self.goal) + str(self.clearance) + str(self.stepSize)

    def astar(self):
        visited = []
        curr = Node(self.start, self.start, self.goal, self.stepSize)
        queue = [curr]
        nodeDict = {tuple(curr.env)}
        visited.append(curr)
        
        while sqrt((curr.env[0] - self.goal[0]) ** 2 + (curr.env[1] - self.goal[1]) ** 2) > 1.5:
            if len(queue) > 0:
                curr = queue.pop()
                environment = Environment(curr.env, self.clearance)
                for action in environment.getActionSet(self.start, curr, self.stepSize):
                    if tuple((int(action.env[0]), int(action.env[1]), action.env[2])) not in nodeDict:
                        queue.append(action)
                        visited.append(action)
                        nodeDict.add(tuple((int(action.env[0]), int(action.env[1]), action.env[2])))
                queue.sort(key=lambda x: x.fScore, reverse=True)

            else:
                return -1, visited
        
        x = curr.path()
        path = []
        for node in x:
            path.append(node)
        return path, visited

def main():
    sleep(5)
    rospy.init_node('talker')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(1) # 1hz
    rpm = [int(sys.argv[7]), int(sys.argv[8])]
    planner = Planner([int((float(sys.argv[1])+5)*10), int((float(sys.argv[2])+5)*10), int(degrees(float(sys.argv[3])))],[int((float(sys.argv[4])+5)*10), int((float(sys.argv[5])+5)*10)], int(sys.argv[6]), rpm)
    path,visited = planner.astar()
    action_set = [[0, rpm[1]],[rpm[0], rpm[1]],[rpm[1], rpm[1]],[rpm[1], rpm[0]],[rpm[1], 0],[0, rpm[0]],[rpm[0], rpm[0]],[rpm[0], 0]]
  
    for i in range(1, len(path)):
        velocity = Twist()
        r = 0.0033
        l = 0.9

        action = path[i].action
        rpm1, rpm2 = action_set[int(action)-1]

        velocity.linear.x = r/2*(rpm1+rpm2)
        velocity.angular.z = r/l*(rpm1-rpm2)
        print(i,rpm1,rpm2)
        pub.publish(velocity)
        sleep(10)
    velocity = Twist()
    velocity.linear.x = 0
    velocity.angular.z = 0
    pub.publish(velocity)


if __name__ == '__main__':
    main()
