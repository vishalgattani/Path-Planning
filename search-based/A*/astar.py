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
import cv2

height = 250
width = 400

thresholdx = 0.5
thresholdy = 0.5
thresholdtheta = 30

fig, ax = plt.subplots()
ax.set(xlim=(0, width), ylim = (0, height))
plt.grid()
plt.xlim(0,width)
plt.ylim(0,height)
plt.title('A* Algorithm')
#plt.savefig('A* Algorithm.png', bbox_inches='tight')

def round_off_rating(number):
    """Round a number to the closest half integer.
    >>> round_off_rating(1.3)
    1.5
    >>> round_off_rating(2.6)
    2.5
    >>> round_off_rating(3.0)
    3.0
    >>> round_off_rating(4.1)
    4.0"""
    return round(number * 2) / 2

# To calculate the new intersection point of the lines after extension
def intersect_point(c1, c2):
    det = abs(c1[0] - c2[0])
    x_inter, y_inter = None, None
    if det != 0:
        x_inter = int(round(abs((c1[1] - c2[1])) / det))
        y_inter = int(round(abs(((c1[0] * c2[1]) - (c2[0] * c1[1]))) / det))
    return [x_inter, y_inter]

def line_intersection(p1,p2,p3,p4):
	[a1,b1,c1,m1,xc1,yc1] = lineFromPoints(p1,p2)
	[a2,b2,c2,m2,xc2,yc2] = lineFromPoints(p3,p4)
	determinant = a1*b2 - a2*b1;
	if (determinant == 0):
		return;
	else:
		x = (b2*c1 - b1*c2)/determinant;
		y = (a1*c2 - a2*c1)/determinant;
	return [int(x),int(y)]

def lineFromPoints(P, Q):
	a = Q[1] - P[1]
	a = np.round(a,3)
	b = P[0] - Q[0]
	b = np.round(b,3)
	# angle = math.atan(-a/b)

	c = a*(P[0]) + b*(P[1])
	c = np.round(c,3)
	if b!=0:
		m = -a/b
		yc = c/b
		xc = c/a
	else:
		m = None
		yc = None
		xc = c/a
	
	return [a,b,c,m,xc,yc]

class Node:
	def __init__(self,i,j,theta,cost,parent_index):
		self.i=np.round(i,3)
		self.j=np.round(j,3)
		self.theta=theta
		self.cost=np.round(cost,3)
		self.gscore = self.cost
		self.fscore = math.inf
		self.index = (int(round_off_rating(self.i)/thresholdx),int(round_off_rating(self.j)/thresholdy),int((self.theta%360)/thresholdtheta))
		self.parent_index = parent_index
		self.parent = None

	def __str__(self):
		# return "("+str(self.i)+" "+str(self.j)+" "+str(self.theta)+")"+" "+str(self.cost)+" Index: "+str(self.index)+" P.I: "+str(self.parent_index)
		return "("+str(self.i)+" "+str(self.j)+" "+str(self.theta)+")"+" "+str(self.cost)
	
	def calcfscore(self,goal):
		self.fscore = self.cost+c2g(self,goal)
		self.h = c2g(self,goal)

def calcNodeIndex(node):
	a = int(round_off_rating(node.i)/thresholdx)
	b = int(round_off_rating(node.j)/thresholdy)
	c = int((node.theta%360)/thresholdtheta)
	return (a,b,c)
		# return (int(node.i/thresholdx),int(node.j/thresholdy),int(int((360+node.theta)/thresholdtheta)%12))

def straight(current,goal,step,obstacles):
	move_x = 1*step
	move_y = 1*step
	x = current.i 
	y = current.j
	theta = current.theta%360
	rad = math.radians(theta)
	c = current.cost 
	c_id = current.index
	move_cost = 1*step
	# print(x,y)
	if x>=0 and y>=0 and x <= width and y <= height: 
		node = Node(current.i + move_x*math.cos(rad),current.j + move_y*math.sin(rad),theta,current.cost + move_cost, c_id)
		if(insideObs(node,obstacles)): # not inside obstacle and not obstacle
			# print("straight")
			return None
		else:
			node.index = calcNodeIndex(node)
			# node.cost = node.cost + c2g(node,goal)
			node.calcfscore(goal)
		return node
	else:
		return None

def straight_u30(current,goal,step,obstacles):
	move_x = 1*step
	move_y = 1*step
	x = current.i 
	y = current.j
	theta = current.theta%360 + 30
	rad = math.radians(theta)
	c = current.cost 
	c_id = current.index
	move_cost = 1*step
	# print(x,y)
	if x>=0 and y>=0 and x <= width and y <= height: 
		node = Node(current.i + move_x*math.cos(rad),current.j + move_y*math.sin(rad),theta,current.cost + move_cost, c_id)
		if(insideObs(node,obstacles)): # not inside obstacle and not obstacle
			# print("u30")
			return None
		else:
			node.index = calcNodeIndex(node)
			# node.cost = node.cost + c2g(node,goal)
			node.calcfscore(goal)
		return node
	else:
		return None

def straight_d30(current,goal,step,obstacles):
	move_x = 1*step
	move_y = 1*step
	x = current.i 
	y = current.j
	theta = current.theta%360 - 30
	rad = math.radians(theta)
	c = current.cost 
	c_id = current.index
	move_cost = 1*step
	# print(x,y)
	if x>=0 and y>=0 and x <= width and y <= height: 
		node = Node(current.i + move_x*math.cos(rad),current.j + move_y*math.sin(rad),theta,current.cost + move_cost, c_id)
		if(insideObs(node,obstacles)): # not inside obstacle and not obstacle
			# print("d30")
			return None
		else:
			node.index = calcNodeIndex(node)
			# node.cost = node.cost + c2g(node,goal)
			node.calcfscore(goal)
		return node
	else:
		return None

def straight_u60(current,goal,step,obstacles):
	move_x = 1*step
	move_y = 1*step
	x = current.i 
	y = current.j
	theta = current.theta%360 + 60
	rad = math.radians(theta)
	c = current.cost 
	c_id = current.index
	move_cost = 1*step
	# print(x,y)
	if x>=0 and y>=0 and x <= width and y <= height: 
		node = Node(current.i + move_x*math.cos(rad),current.j + move_y*math.sin(rad),theta,current.cost + move_cost, c_id)
		if(insideObs(node,obstacles)): # not inside obstacle and not obstacle
			# print("u60")
			return None
		else:
			node.index = calcNodeIndex(node)
			# node.cost = node.cost + c2g(node,goal)
			node.calcfscore(goal)
		return node
	else:
		return None

def straight_d60(current,goal,step,obstacles):
	move_x = 1*step
	move_y = 1*step
	x = current.i 
	y = current.j
	theta = current.theta%360 - 60
	rad = math.radians(theta)
	c = current.cost 
	c_id = current.index
	move_cost = 1*step
	# print(x,y)
	if x>=0 and y>=0 and x <= width and y <= height: 
		node = Node(current.i + move_x*math.cos(rad),current.j + move_y*math.sin(rad),theta,current.cost + move_cost, c_id)
		if(insideObs(node,obstacles)): # not inside obstacle and not obstacle
			# print("d60")
			return None
		else:
			node.index = calcNodeIndex(node)
			# node.cost = node.cost + c2g(node,goal)
			node.calcfscore(goal)
		return node
	else:
		return None

def expand(current,goal,step,obstacles):
	neighbors = []
	neighbors.append(straight(current,goal,step,obstacles))
	neighbors.append(straight_u30(current,goal,step,obstacles))
	neighbors.append(straight_d30(current,goal,step,obstacles))
	neighbors.append(straight_u60(current,goal,step,obstacles))
	neighbors.append(straight_d60(current,goal,step,obstacles))
	neighbors = [i for i in neighbors if i]

	return neighbors

def expandHex(hexpts,delta):
	increase = delta
	
	p1 = hexpts[0]
	p2 = hexpts[1]
	p3 = hexpts[2]
	p4 = hexpts[3]
	p5 = hexpts[4]
	p6 = hexpts[5]

	[a1,b1,c1,m1,xc1,yc1] = lineFromPoints(p1,p2)
	[a2,b2,c2,m2,xc2,yc2] = lineFromPoints(p2,p3)
	[a3,b3,c3,m3,xc3,yc3] = lineFromPoints(p3,p4)
	[a4,b4,c4,m4,xc4,yc4] = lineFromPoints(p4,p5)
	[a5,b5,c5,m5,xc5,yc5] = lineFromPoints(p5,p6)
	[a6,b6,c6,m6,xc6,yc6] = lineFromPoints(p6,p1)

	if increase < 1:
		return p1, p2, p3, p4, p5, p6
	else:
		yc1 = yc1 - (increase/(math.sin(1.57 - math.atan(m1))))
		# print(-yc1/m1,yc1)

		yc2 = yc2 - (increase/(math.sin(1.57 - math.atan(m2))))
		# print(-yc2/m2,yc2)

		if(yc3):
			pass
		else:
			xc3 = xc3-increase

		yc4 = yc4 + (increase/(math.sin(1.57 - math.atan(m4))))
		yc5 = yc5 +(increase/(math.sin(1.57 - math.atan(m5))))

		if(yc6):
			pass
		else:
			xc6 = xc6+increase

	a1,b1=[-yc1/m1,0],[0,yc1]
	a2,b2=[-yc2/m2,0],[0,yc2]
	a3,b3=[xc3,xc3],[xc3,xc3+1]
	a4,b4=[-yc4/m4,0],[0,yc4]
	a5,b5=[-yc5/m5,0],[0,yc5]
	a6,b6=[xc6,xc6],[xc6,xc6+1]

	p2 = line_intersection(a1,b1,a2,b2)
	p3 = line_intersection(a2,b2,a3,b3)
	p4 = line_intersection(a3,b3,a4,b4)
	p5 = line_intersection(a4,b4,a5,b5)
	p6 = line_intersection(a5,b5,a6,b6)
	p1 = line_intersection(a6,b6,a1,b1)
	return [p1, p2, p3, p4, p5, p6]

def expandQuad(vpts,delta):
	increase = delta
	# vpts = [[105,100],[80,180],[115,210],[36,185]]
	r1 = vpts[0]
	r2 = vpts[1]
	r3 = vpts[2]
	r4 = vpts[3]

	line1 = np.array(np.polyfit([r1[0], r2[0]], [r1[1], r2[1]], 1))
	line2 = np.array(np.polyfit([r2[0], r3[0]], [r2[1], r3[1]], 1))
	line3 = np.array(np.polyfit([r3[0], r4[0]], [r3[1], r4[1]], 1))
	line4 = np.array(np.polyfit([r4[0], r1[0]], [r4[1], r1[1]], 1))
	if increase < 1:
		return [r1,r2,r3,r4]
	else:
		line1[1] = line1[1] + (increase / (math.sin(1.57 - math.atan(line1[0]))))
		line2[1] = line2[1] - (increase / (math.sin(1.57 - math.atan(line2[0]))))
		line3[1] = line3[1] + (increase / (math.sin(1.57 - math.atan(line3[0]))))
		line4[1] = line4[1] - (increase / (math.sin(1.57 - math.atan(line4[0]))))
		r2 = intersect_point(line1, line2)
		r3 = intersect_point(line2, line3)
		r4 = intersect_point(line3, line4)
		r1 = intersect_point(line4, line1)
	return [r1,r2,r3,r4]


def buildObsMap_astar(start,goal,dimension=0,clearance=0):

	circle = [300,185,40]
	hexpts = [[235,80],[200,60],[165,80],[165,120],[200,140],[235,120]]
	vpts = [[105,100],[80,180],[115,210],[36,185]]
	tri_top = [[115,210],[36,185],[80,180]]
	tri_bot = [[36,185],[80,180],[105,100]]
	
	delta = dimension+clearance

	if(delta):
		newheight = height-delta
		newwidth = width-delta
		orig = [[0,0],[0,height],[width,height],[width,0]]
		bbox = [[delta,delta],[delta,newheight],[newwidth,newheight],[newwidth,delta]]
		newhexpts = expandHex(hexpts,delta)
		newvpts = expandQuad(vpts,delta)
		newcircle = [300,185,40+delta]
		
		# plot dimension + clearance obstacles
		ax.add_artist(Polygon(orig, color="red"))
		ax.add_artist(Polygon(bbox, color="white"))
		ax.add_artist(Polygon(newhexpts, color = "red"))
		ax.add_artist(Polygon(newvpts, color = "red"))
		ax.add_artist(plt.Circle((newcircle[0], newcircle[1]), newcircle[2], color = "red"))		
		
		# plot normal obstacles
		ax.add_artist(plt.Circle((circle[0], circle[1]), circle[2], color = "black"))
		ax.add_artist(Polygon(hexpts, color = "black"))
		ax.add_artist(Polygon(vpts, color = "black"))
		ax.set_aspect('equal')
		return [newcircle,newhexpts,newvpts,bbox]
	else:
		bbox = [[0,0],[0,height],[width,height],[width,0]]
		ax.add_artist(plt.Circle((circle[0], circle[1]), circle[2], color = "black"))
		ax.add_artist(Polygon(hexpts, color = "black"))
		ax.add_artist(Polygon(vpts, color = "black"))
		ax.set_aspect('equal')
		return [circle,hexpts,vpts,bbox]

	
def insideObs(node,obstacles):
	
	circleobs = obstacles[0]
	circlex = circleobs[0]
	circley = circleobs[1]
	circler = circleobs[2]
	flag1=(node.i-circlex)**2+(node.j-circley)**2-(circler)**2 ## clearance is included 40+5
	if flag1<=0:
		flag1 = True
	else: 
		flag1 = False

	hexpts = obstacles[1]
	flag2 = []
	for i in range(len(hexpts)):
		[a,b,c,m,xc,yc] = lineFromPoints(hexpts[i-1],hexpts[i])
		f = np.sign((a*node.i+b*node.j-c)*(a*0+b*0-c))
		# print(f)
		flag2.append(f)	
	# online = ~np.all(flag2)
	inside = (flag2==[1,1,-1,-1,1,1])
	flag2 = inside
	poly_path2 = mplPath.Path(np.array(hexpts))
	flag2 = poly_path2.contains_point((node.i,node.j))
	
	# lower right # left #top right # right
	vshape = obstacles[2]
	# vshape1 = [[36,185],[115,210],[80,180]]
	# vshape2 = [[36,185],[105,100],[80,180]]
	poly_path3 = mplPath.Path(np.array(vshape))
	flag3 = poly_path3.contains_point((node.i,node.j))

	bbox = obstacles[3]
	poly_path4 = mplPath.Path(np.array(bbox))
	flag4 = not(poly_path4.contains_point((node.i,node.j)))

	# if(flag1): print("c")
	# if(flag2): print("h")
	# if(flag3): print("v")
	# if(flag4): print("b")

	# False False False True
	return flag1 or flag2 or flag3 or flag4


def c2g(current,goal):
	point1 = np.array((current.i,current.j))
	point2 = np.array((goal.i,goal.j))
	return np.round(np.linalg.norm(point1 - point2),3)

def pop_node_lowest_fscore(queue):  # Priority Queue, outputs the node with least cost attached to it
    low = 0
    for i in range(len(queue)):
        if queue[i].fscore < queue[low].fscore:
            low = i
    return queue.pop(low)

def find_node(node, queue):

	x = int(round_off_rating(node.i)/thresholdx)
	y = int(round_off_rating(node.j)/thresholdy)
	z = int((node.theta%360)/thresholdtheta)
	for c in queue:
		if c.i==x and c.j==y and int((c.theta%360)/thresholdtheta)==z:
			return queue.index(c)
		else:
			return None

def planning_astar(start,goal,dimension,clearance,step):
	
	height = 250
	width = 400

	visited = np.zeros([int(width/thresholdx),int(height/thresholdy),int(360/thresholdtheta)])
	
	start_node = Node(start[0],start[1],start[2], 0.0, -1)
	goal_node = Node(goal[0],goal[1],goal[2], 0.0, -1)
	start_node.index = calcNodeIndex(start_node)
	start_node.index = calcNodeIndex(start_node)

	# print("Start Node F score: ",start_node.fscore)
	# start_node.calcfscore(goal_node)
	# # goal_node.calcfscore(goal_node)
	# print("Start Node F score: ",start_node.fscore)
	# print("Start:\t",start_node.__str__())
	# print("Goal:\t",goal_node.__str__())
	
	obstacles = buildObsMap_astar(start,goal,dimension,clearance)

	t = mpl.markers.MarkerStyle(marker=">")
	t._transform = t.get_transform().rotate_deg(start_node.theta)

	ax.add_artist(plt.Circle((goal[0], goal[1]), 1, color = "green",fill=False))
	ax.add_artist(plt.Circle((goal[0], goal[1]), step, color = "red",fill=False))
	plt.plot(start_node.i, start_node.j, marker=t)
	plt.plot(goal_node.i, goal_node.j, 'rx')
	
	if(insideObs(start_node,obstacles)):
		print("Start inside obstacle")
		plt.show()
		sys.exit()
	
	if(insideObs(goal_node,obstacles)):
		print("Goal inside obstacle")
		plt.show()
		sys.exit()

	open_set, closed_set = dict(), dict()
	open_set[calcNodeIndex(start_node)] = start_node

	queue = []
	queue.append(start_node)

	
	while queue:
		
		current = pop_node_lowest_fscore(queue)
		
		a = int(round_off_rating(current.i)/thresholdx)
		b = int(round_off_rating(current.j)/thresholdy)
		c = int((current.theta%360)/thresholdtheta)
		 
		# visited[int(current.i/thresholdx)][int(current.j/thresholdy)][int((current.theta%360)/thresholdTheta)] = 1
        # print(x,y,z,"\t",current.i,current.j,current.theta)
		visited[a][b][c] = 1

		# sys.exit()
		# print(np.count_nonzero(visited == 1))
		
		# checkgoal = (current.i-goal_node.i)**2 + (current.j-goal_node.j)**2 - (1)
		checkgoal = (current.i-goal_node.i)**2 + (current.j-goal_node.j)**2 - (step*step)
		if checkgoal<=0:
			print("DONE.")
			print("Closest to goal: ",current)
			goal_node.parent = current
			goal_node.parent_index = current.index
			goal_node.cost = current.cost+c2g(current,goal_node)
			print("Goal: \t", goal_node)
			print("Path cost: \t",goal_node.cost)
			return goal_node


		neighbors = expand(current,goal_node,step,obstacles)
		# sys.exit()

		for neighbor in neighbors:
				neighbor.parent = current
				plt.scatter(neighbor.i, neighbor.j,marker=",",c="yellow")				
				# plt.pause(0.05)
				x = int(round_off_rating(neighbor.i)/thresholdx)
				y = int(round_off_rating(neighbor.j)/thresholdy)
				z = int((neighbor.theta%360)/thresholdtheta)

				try:
					if visited[x][y][z]==0:
						neighbor.fscore = neighbor.fscore + c2g(neighbor, goal_node)
						visited[x][y][z] = 1
						queue.append(neighbor)
					else:
						n_id = find_node(neighbor, queue)
						if n_id is not None:
							temp_node = queue[n_id]
							if temp_node.cost + c2g(temp_node, goal_node) > neighbor.cost + c2g(temp, goal_node):
								temp_node.cost = neighbor.cost
								temp_node.calcfscore(goal)
								temp_node.parent = current

				except:
					continue			
	return None
	
def backtrack(node):
	coords = []
	# print(node)
	coords.append([node.i,node.j,node.theta])
	while(node.parent):
		node = node.parent
		coords.append([node.i,node.j,node.theta])
	return coords[::-1]

def plotbacktrack(path):
	t = mpl.markers.MarkerStyle(marker=">")
	for i in range(len(path)):	
		if(i==0):
			t._transform = t.get_transform().rotate_deg(path[i][2])
			plt.plot(path[i][0], path[i][1], 'g',marker=t)
		else:
			t._transform = t.get_transform().rotate_deg(path[i][2]-path[i-1][2])
			plt.plot(path[i][0], path[i][1], 'g' ,marker=t)
			# plt.plot(path[i][0], path[i][1])
			plt.plot([path[i][0],path[i-1][0]],[path[i][1],path[i-1][1]],'b', linestyle="--")
			# plt.quiver(path[i][0],path[i][1],path[i][0]-path[i-1][0],path[i][1]-path[i-1][1],color= 'green',units='xy' ,scale=1)

def main():

	dimension = int(input("Robot dimension: "))
	clearance = int(input("Obstacle clearance: "))
	startx = int(input("Start X (integer): "))
	starty = int(input("Start Y (integer): "))
	startthetha = int(input("Start ϴ (degrees): "))
	
	goalx = int(input("Goal X (integer): "))
	goaly = int(input("Goal Y (integer): "))
	step = int(input("Step Size (1-10): "))
	
	goaltheta = 0 

	start_time = timeit.default_timer()	
	goal_node = planning_astar([startx,starty,startthetha],[goalx,goaly,goaltheta],dimension,clearance,step)
	elapsed = timeit.default_timer() - start_time
	if(goal_node):
		print("Time taken: ",elapsed," seconds")
		path = backtrack(goal_node)
		plotbacktrack(path)
		# red_patch = mpatches.Patch(color='red', label='clearance area')
		# ax.legend(handles=[red_patch],loc='best', bbox_to_anchor=(0.5, 0., 0.5, 0.5))
		plt.show()
		plt.close()
	else:
		print("Goal cannot be reached!")

if __name__ == "__main__":
	main()
