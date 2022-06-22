import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.patches import Polygon
import matplotlib.patches as mpatches
import math
import sys
import timeit
import matplotlib.path as mplPath
import io
import random

fig, ax = plt.subplots()


class Env:
    def __init__(self,height,width,start,goal):
        self.obs_circle = []
        self.obs_poly = []
        self.height = height
        self.width = width
        self.start = start
        self.goal = goal
        
        self.bbox = [[0,0],[0,self.height],[self.width,self.height],[self.width,0]]

    def __str__(self):
    # return "("+str(self.i)+" "+str(self.j)+" "+str(self.theta)+")"+" "+str(self.cost)+" Index: "+str(self.index)+" P.I: "+str(self.parent_index)
        return len(self.obs)

    def addCircle(self,x,y,r):
        circle = [x,y,r]
        # ax.add_artist(plt.Circle((circle[0], circle[1]), circle[2], color = "black"))
        self.obs_circle.append(circle)

    def addPoly(self,pts):
        poly = pts
        # ax.add_artist(Polygon(poly, color = "black"))
        self.obs_poly.append(poly)

    def checkInside(self,node):
        flaglist = []

        for ob in self.obs_circle:
            flag=(node.x-ob[0])**2+(node.y-ob[1])**2-(ob[2])**2 ## clearance is included 40+5
            if flag<=0:
                flag = True
            else: 
                flag = False
            flaglist.append(flag)
        for ob in self.obs_poly:
            poly_path = mplPath.Path(np.array(ob))
            flag = poly_path.contains_point((node.x,node.y))
            flaglist.append(flag)
            
         
        return any(flaglist)



class Node:
    def __init__(self,coords,parent):
        self.x = coords[0]
        self.y = coords[1]
        self.parent = None

class Graph:
    def __init__(self,start,goal,env,dmax=1):
        self.x = []
        self.y = []
        self.parent = []
        self.x.append(start[0])
        self.y.append(start[1])
        self.parent.append(0)
        self.goalState = None
        self.path = []
        self.start = start
        self.goal = goal
        self.goalFlag = False
        self.dmax = dmax
        self.env = env

    def add_node(self,n,x,y):
        self.x.insert(n,x)
        self.y.insert(n,y)

    def remove_node(self,n):
        self.x.pop(n)
        self.y.pop(n)

    def add_edge(self,parent,child):
        self.parent.insert(child,parent)
        
    def remove_edge(self,n):
        self.parent.pop(n)

    def num_nodes(self):
        return len(self.x)

    def get_distance(self,a,b):
        return distance([self.x[a],self.y[a]],[self.x[b],self.y[b]])

    def sampleEnv(self):
        x = random.uniform(0,self.env.width)
        y = random.uniform(0,self.env.height)
        return x,y

    def insideObstacle(self):
        n = self.num_nodes()-1
        x,y = self.x[n],self.y[n]
        if self.env.checkInside(Node([x,y],None)):
            self.remove_node(n)
            return True
        return False

    def acrossObstacle(self,x1,y1,x2,y2):
        poly_path = mplPath.Path(np.array([[x1,y1],[x2,y2]]))
        flag = False
        for ob in self.env.obs_poly:
            pathtocheck = mplPath.Path(np.array(ob))
            if poly_path.intersects_path(pathtocheck):
                flag = True
                return flag

        for ob in self.env.obs_circle:
            pathtocheck = mplPath.Path.circle(center=(ob[0],ob[1]),radius = ob[2])
            if poly_path.intersects_path(pathtocheck):
                flag = True
                return flag

        return flag

    def connect(self,n1,n2):
        a,b = self.x[n1],self.y[n1]
        c,d = self.x[n2],self.y[n2]
        if self.acrossObstacle(a,b,c,d):
            self.remove_node(n2)
            return False
        else:
            self.add_edge(n1,n2)
            return True

    def nearest(self,n):
        dmin = self.get_distance(0,n)
        nnear = 0
        for i in range(0,n):
            if self.get_distance(i,n)<dmin:
                dmin = self.get_distance(i,n)
                nnear = i
        return nnear

    def step(self,nnear,nrand):
        dmax = self.dmax
        d = self.get_distance(nnear,nrand)
        if(d>dmax):
            u = dmax/d
            xnear,ynear = self.x[nnear],self.y[nnear]
            xrand,yrand = self.x[nrand],self.y[nrand]
            px,py = xrand-xnear,yrand-ynear
            theta = math.atan2(py,px)
            x = xnear+dmax*math.cos(theta)
            y = ynear+dmax*math.sin(theta)
            self.remove_node(nrand)
            if(abs(x-self.goal[0])<dmax and abs(y-self.goal[1])<dmax):
                self.add_node(nrand,self.goal[0],self.goal[1])
                self.goalState = nrand
                self.goalFlag = True
            else:
                
                self.add_node(nrand,x,y)

    def bias(self,ngoal):
        n = self.num_nodes()
        self.add_node(n,ngoal[0],ngoal[1])
        if not self.insideObstacle():
            # print(self.insideObstacle())
            nnear = self.nearest(n)
            self.step(nnear,n)
            self.connect(nnear,n)
        return self.x,self.y,self.parent

    def expand(self):
        n = self.num_nodes()
        x,y = self.sampleEnv()
        flag = False
        if self.env.checkInside(Node([x,y],None)):
                flag = True
                return None,None,None
        if not flag:
            self.add_node(n,x,y)
            if not self.insideObstacle():
                # print(self.insideObstacle())
                xnearest = self.nearest(n)
                self.step(xnearest,n)
                self.connect(xnearest,n)
            return self.x,self.y,self.parent

    def path_to_goal(self):
        if self.goalFlag:
            self.path = []
            self.path.append(self.goalState)
            temp = self.parent[self.goalState]
            while temp!=0:
                self.path.append(temp) 
                temp = self.parent[temp]
            self.path.append(0)
        return self.goalFlag

    def path_to_goal_coords(self):
        coords = []
        for node in self.path:
            x,y = self.x[node],self.y[node]
            coords.append([x,y])
        self.coords = coords
        return coords

    def drawPath(self,path,n_iterations):
        path.reverse()
        for i in range(len(path)):
            # plt.plot([path[i][0],path[i-1][0]],[path[i][1],path[i-1][1]],'b', linestyle="--")
            plt.scatter(path[i][0],path[i][1],color="red")
            plt.pause(0.001)
            n_iterations += 1
            plt.savefig('img/rrt_'+str(n_iterations)+'.png',dpi=300)
            

    # def cost(self):

    def checkNodes(self):
        for i in range(len(self.x)):
            x = self.x[i]
            y = self.y[i]
            if self.env.checkInside(Node([x,y],None)):
                print(i,x,y)

def distance(x, y):
    return np.linalg.norm(np.array(x) - np.array(y))

def drawEnv(name,env):
    ax.set(xlim=(0, env.width), ylim = (0, env.height))
    plt.grid()
    plt.xlim(0,env.width)
    plt.ylim(0,env.height)
    # ax.add_artist(Polygon(self.env.bbox, edgecolor='black',facecolor='white'))
    for pts in env.obs_poly:
        ax.add_artist(Polygon(pts, color = "black"))

    for circle in env.obs_circle:
        ax.add_artist(plt.Circle((circle[0], circle[1]), circle[2], color = "black"))
      
    plt.plot(env.start[0],env.start[1], "bs", linewidth=3)
    plt.plot(env.goal[0],env.goal[1], "gs", linewidth=3)

    plt.title(name)
    ax.set_aspect('equal')
    # plt.show()

def main():
    
    name = "RRT"
    height = 250
    width = 400
    start = [1, 1]  
    goal = [width-5, height-5]
    iterations = 500  
    n_iterations = 0
    dmax = 5
    
    hexpts = [[235,80],[200,60],[165,80],[165,120],[200,140],[235,120]]
    vpts = [[105,100],[80,180],[115,210],[36,185]]
    tri_top = [[115,210],[36,185],[80,180]]
    tri_bot = [[36,185],[80,180],[105,100]]

    env = Env(height,width,start,goal)
    env.addCircle(300,185,40)
    env.addPoly(hexpts)
    env.addPoly(vpts)
    env.addPoly(tri_top)
    env.addPoly(tri_bot)
    drawEnv(name,env)

    G = Graph(start,goal,env,dmax)

    while not G.path_to_goal():
        n = G.num_nodes()
        
        if n_iterations%50!=0:
            x,y,parent = G.expand()
            if x is not None: 
                plt.scatter(x[-1],y[-1],marker=".",c="cyan")
                plt.plot([x[-1],x[parent[-1]]],[y[-1],y[parent[-1]]],'b', linestyle="--")
        else:
            x,y,parent = G.bias(goal)
            # if x is not None: 
            plt.scatter(x[-1],y[-1],marker=".",c="cyan")
            plt.plot([x[-1],x[parent[-1]]],[y[-1],y[parent[-1]]],'b', linestyle="--")
        if x is not None: 
            plt.pause(0.001)
            plt.savefig('img/rrt_'+str(n_iterations)+'.png',dpi=300)
            n_iterations += 1

    G.drawPath(G.path_to_goal_coords(),n_iterations)

    print("Goal Found.")
    G.checkNodes()
    plt.show()
        

if __name__ == '__main__':
    main()