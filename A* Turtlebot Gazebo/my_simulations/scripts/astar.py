
from math import sqrt, cos, sin, radians, atan2, floor, degrees



class Node:
    # Initialize
    def __init__(self, start, env, goal, stepSize, parent=None, action=None):
        self.env = åenv
        self.parent = parent
        self.goal = goal
        self.action = action

        if parent is not None:
            step = sqrt((env[0] - parent.env[0]) ** 2 + (env[1] - parent.env[1]) ** 2)
            self.g = parent.g + step
        else:
            self.g = 0
        # Heuristic function
        self.weight = self.g + sqrt((env[0] - goal[0]) ** 2 + (env[1] - goal[1]) ** 2) + (
                (env[2] - floor(atan2((goal[1] - start[1]), (goal[0] - start[0])))) / 30)  # * (step / 5)

    # Solve for path from goal to start node
    def path(self):
        node, p = self, []
        while node:
            p.append(node)
            node = node.parent
        return reversed(p)

    # Get possible actions
    def actions(self):
        if self.action is None:
            return self.env.possibleMoves()
        else:
            return self.env.possibleMoves(self.action)


class Environment:
    # Initialize
    def __init__(self, currentPosition, clearance):
        self.currentPosition = currentPosition
        self.clearance = clearance

    def insideCircle1(self, position):
        if (position[0] - 51) ** 2 + (position[1] - 51) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

        # Check if node is in bottom left cirlce

    def insideCircle2(self, position):
        if (position[0] - 31) ** 2 + (position[1] - 21) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

        # Check if node is in bottom right cirlce

    def insideCircle3(self, position):
        if (position[0] - 71) ** 2 + (position[1] - 21) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

        # Check if node is in top right cirlce

    def insideCircle4(self, position):
        if (position[0] - 71) ** 2 + (position[1] - 81) ** 2 <= (10 + self.clearance) ** 2:
            return True
        else:
            return False

        # Check if node is in top left square using half planes

    def insideSquare1(self, position):
        if position[0] > 23.5 - self.clearance and position[0] < 38.5 + self.clearance and position[
            1] < 88.5 + self.clearance and position[1] > 73.5 - self.clearance:
            return True
        else:
            return False

        # Check if node is in middle left square using half planes

    def insideSquare2(self, position):
        if position[0] > 3.5 - self.clearance and position[0] < 18.5 + self.clearance and position[
            1] < 58.5 + self.clearance and position[1] > 43.5 - self.clearance:
            return True
        else:
            return False

        # Check if node is in middle right square using half planes

    def insideSquare3(self, position):
        if position[0] > 83.5 - self.clearance and position[0] < 98.5 + self.clearance and position[
            1] < 58.5 + self.clearance and position[1] > 43.5 - self.clearance:
            return True
        else:
            return False

        # Check if node is outside of map using half planes

    def outsideMap(self, position):
        if position[0] > 1.0 + self.clearance and position[0] < 101.0 - self.clearance and position[
            1] < 101.0 - self.clearance and position[1] > 1.0 + self.clearance:
            return False
        else:
            return True

    # Check if position is inside map or inside an object
    def possiblePostion(self, position):
        possiblity = True
        if self.insideCircle1(position):
            possiblity = False
        if self.insideCircle2(position):
            possiblity = False
        if self.insideCircle3(position):
            possiblity = False
        if self.insideCircle4(position):
            possiblity = False
        if self.insideSquare1(position):
            possiblity = False
        if self.insideSquare2(position):
            possiblity = False
        if self.insideSquare3(position):
            possiblity = False
        if self.outsideMap(position):
            possiblity = False
        return possiblity

        # Check if each action is possible

    def possibleMoves(self, start, node, stepSize):
        actions = []
        temp = self.move(start, '1', stepSize, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '2', stepSize, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '3', stepSize, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '4', stepSize, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '5', stepSize, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '6', stepSize, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '7', stepSize, node)
        if temp is not None:
            actions.append(temp)
        temp = self.move(start, '8', stepSize, node)
        if temp is not None:
            actions.append(temp)
        return actions
    # Move robot position according to action
    def move(self, start, val, stepSize, node):
        temp = None
        if val == '1':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, 0, stepSize[1])
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, stepSize, node, val)
        if val == '2':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, stepSize[0], stepSize[1])
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, stepSize, node, val)
        if val == '3':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, stepSize[1], stepSize[1])
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, stepSize, node, val)
        if val == '4':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, stepSize[1], stepSize[0])
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, stepSize, node, val)
        if val == '5':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, stepSize[1], 0)
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, stepSize, node, val)
        if val == '6':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, 0, stepSize[0])
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, stepSize, node, val)
        if val == '7':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, stepSize[0], stepSize[0])
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, stepSize, node, val)
        if val == '8':
            tempBoolean = True
            x = self.currentPosition[0]
            y = self.currentPosition[1]
            theta = self.currentPosition[2]
            for i in range(100):
                x, y, theta = self.increment(x, y, theta, stepSize[0], 0)
                if not self.possiblePostion([x, y]):
                    tempBoolean = False
            if tempBoolean:
                temp = Node(start, [x, y, theta], node.goal, stepSize, node, val)
        return temp

    def increment(self, x, y, theta, rpm1, rpm2):
        r = 0.033
        l = 0.160
        rpm1 /= 10
        rpm2 /= 10
        x += r / 2 * (rpm1 + rpm2) * cos(radians(theta))
        y += r / 2 * (rpm1 + rpm2) * sin(radians(theta))
        theta += r / l * (rpm1 - rpm2)
        theta = self.angleCheck(theta)
        return x, y, theta

    # Keep angle value from 0 to 360
    def angleCheck(self, angle):
        if angle >= 360:
            angle -= 360
        if angle < 0:
            angle = 360 + angle
        return angle
        
class PathPlanning:
    # Initiation method
    def __init__(self, start, goal, clearance, stepSize):
        self.start = start
        self.goal = goal
        self.clearance = clearance
        self.stepSize = stepSize

    # Method to solve a A* object
    def Astar(self):
        search = []
        # Set current node to start and add start node to the node list and node search dictionary
        CurrentNode = Node(self.start, self.start, self.goal, self.stepSize)
        NodeList = [CurrentNode]
        NodeDict = {tuple(CurrentNode.env)}
        search.append(CurrentNode)
        # Check if the current node is the goal node
        while sqrt((CurrentNode.env[0] - self.goal[0]) ** 2 + (CurrentNode.env[1] - self.goal[1]) ** 2) > 1.5:

            # Keep checking if there are nodes in list
            if len(NodeList) > 0:
                # Set current node to the first node in the list and then delete from list
                CurrentNode = NodeList.pop()

                Course = Environment(CurrentNode.env, self.clearance)
                # Check all of the possible actions
                for action in Course.possibleMoves(self.start, CurrentNode, self.stepSize):

                    # Search dictonary and add node to list and dictionary if it hasn't been explored yet
                    if tuple((int(action.env[0]), int(action.env[1]), action.env[2])) not in NodeDict:
                        NodeList.append(action)
                        search.append(action)
                        NodeDict.add(tuple((int(action.env[0]), int(action.env[1]), action.env[2])))
                # Sort list of nodes based on cost
                NodeList.sort(key=lambda x: x.weight, reverse=True)

            else:
                return -1, CurrentNode.path(), search
        # solve for path
        x = CurrentNode.path()
        path = []
        for node in x:
            path.append(node)
        return path, search
