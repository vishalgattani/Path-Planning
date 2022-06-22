import cv2
import numpy as np
import matplotlib.pyplot as plt
import math
import sys
import timeit
import sympy
from sympy import Point, Line, Segment

height = 250
width = 400


#opencv is BGR
black = [0,0,0]
white = [255,255,255]
blue = (255,0,0)
green = (0,255,0)
red = (0,0,255)
yellow = (0,255,255)
cyan = (255,255,0)
purple = (255,0,255)

motion = [[1, 0, 1],
                  [0, 1, 1],
                  [-1, 0, 1],
                  [0, -1, 1],
                  [-1, -1, math.sqrt(2)],
                  [-1, 1, math.sqrt(2)],
                  [1, -1, math.sqrt(2)],
                  [1, 1, math.sqrt(2)]]

def setColorPixel(img,i,j,color):
	img[j][i] = color
	return img


# To calculate the new intersection point of the lines after extension
def intersect_point(c1, c2):
    det = abs(c1[0] - c2[0])

    x_inter, y_inter = None, None
    if det != 0:
        x_inter = int(round(abs((c1[1] - c2[1])) / det))
        y_inter = int(round(abs(((c1[0] * c2[1]) - (c2[0] * c1[1]))) / det))

    return [x_inter, y_inter]

def line_intersection(p1,p2,p3,p4):
	#[a1,b1],[a2,b2]
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
	

	# try:
	# # slope = -a/b
	# 	angle = math.atan(-a/b)
	# except:
	# # slope = math.inf
	# 	angle = math.atan(-a/b)
	# 	pass 
		# angle = math.atan(math.inf)

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
	# if(b < 0):
	# 	print("line:",P,Q,a, "x  ", b, "y = ", c,xc,yc)
	# else:
	# 	print("line:",P,Q,a, "x + ", b, "y = ", c,xc,yc)
	return [a,b,c,m,xc,yc]

def expandTriTop(hexpts,delta):
	increase = delta
	#tri_top = [[115,210],[36,185],[80,180]]
	p1 = hexpts[0]
	p2 = hexpts[1]
	p3 = hexpts[2]
	[a1,b1,c1,m1,xc1,yc1] = lineFromPoints(p1,p2)
	[a2,b2,c2,m2,xc2,yc2] = lineFromPoints(p2,p3)
	[a3,b3,c3,m3,xc3,yc3] = lineFromPoints(p3,p1)
	if increase < 1:
		return p1, p2, p3
	else:
		yc1 = yc1 + (increase/(math.sin(1.57 - math.atan(m1))))
		# print(-yc1/m1,yc1)

		yc2 = yc2 - (increase/(math.sin(1.57 - math.atan(m2))))
		# print(-yc2/m2,yc2)

		yc3 = yc3 - (increase/(math.sin(1.57 - math.atan(m3))))
		# print(-yc3/m3,yc3)



	a1,b1=[-yc1/m1,0],[0,yc1]
	a2,b2=[-yc2/m2,0],[0,yc2]
	a3,b3=[-yc3/m3,0],[0,yc3]

	#lineFromPoints(a1,b1)
	p2 = line_intersection(a1,b1,a2,b2)
	p3 = line_intersection(a2,b2,a3,b3)
	p1 = line_intersection(a3,b3,a1,b1)


	return [p1, p2, p3]

def expandTriBot(hexpts,delta):
	increase = delta
	#tri_bot = [[36,185],[80,180],[105,100]]
	p1 = hexpts[0]
	p2 = hexpts[1]
	p3 = hexpts[2]
	[a1,b1,c1,m1,xc1,yc1] = lineFromPoints(p1,p2)
	[a2,b2,c2,m2,xc2,yc2] = lineFromPoints(p2,p3)
	[a3,b3,c3,m3,xc3,yc3] = lineFromPoints(p3,p1)
	if increase < 1:
		return p1, p2, p3
	else:
		yc1 = yc1 + (increase/(math.sin(1.57 - math.atan(m1))))
		# print(-yc1/m1,yc1)

		yc2 = yc2 + (increase/(math.sin(1.57 - math.atan(m2))))
		# print(-yc2/m2,yc2)

		yc3 = yc3 -(increase/(math.sin(1.57 - math.atan(m3))))
		# print(-yc3/m3,yc3)



	a1,b1=[-yc1/m1,0],[0,yc1]
	a2,b2=[-yc2/m2,0],[0,yc2]
	a3,b3=[-yc3/m3,0],[0,yc3]

	#lineFromPoints(a1,b1)
	p2 = line_intersection(a1,b1,a2,b2)
	p3 = line_intersection(a2,b2,a3,b3)
	p1 = line_intersection(a3,b3,a1,b1)


	return [p1, p2, p3]

# Output the new coordinates of the polygon obstacle
def expandHex(hexpts,delta):
	increase = delta
	# hexpts = [[235,80],[200,60],[165,80],[165,120],[200,140],[235,120]]
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




	# coff1 = np.array(np.polyfit([p1[0], p2[0]], [p1[1], p2[1]], 1))
	# coff2 = np.array(np.polyfit([p2[0], p3[0]], [p2[1], p3[1]], 1))
	# coff3 = np.array(np.polyfit([p3[0], p4[0]], [p3[1], p4[1]], 1))
	# coff4 = np.array(np.polyfit([p4[0], p5[0]], [p4[1], p5[1]], 1))
	# coff5 = np.array(np.polyfit([p5[0], p6[0]], [p5[1], p6[1]], 1))
	# coff6 = np.array(np.polyfit([p6[0], p1[0]], [p6[1], p1[1]], 1))

	#print(coff1,-c1/b1)

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
		# print(-yc4/m4,yc4)

		yc5 = yc5 +(increase/(math.sin(1.57 - math.atan(m5))))
		# print(-yc5/m5,yc5)

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

	#lineFromPoints(a1,b1)
	p2 = line_intersection(a1,b1,a2,b2)
	p3 = line_intersection(a2,b2,a3,b3)
	p4 = line_intersection(a3,b3,a4,b4)
	p5 = line_intersection(a4,b4,a5,b5)
	p6 = line_intersection(a5,b5,a6,b6)
	p1 = line_intersection(a6,b6,a1,b1)


	return [p1, p2, p3, p4, p5, p6]


def isLeft(a,b,c):
     return ((b[0] - a[0])*(c[1] - a[1]) - (b[1] - a[1])*(c[0] - a[0])) < 0
def isRight(a,b,c):
     return ((b[0] - a[0])*(c[1] - a[1]) - (b[1] - a[1])*(c[0] - a[0])) < 0




	



	
def buildObsMap(start,goal,dimension=0,clearance=0):

	height = 250
	width = 400
	img = 255 * np.ones((height, width, 3), np.uint8)
	#print(img.shape)


	shapes = []

	fill = -1
	nofill = 0


	# dimension = 3
	# clearance = 2
	delta = dimension+clearance

	hexpts = [[235,80],[200,60],[165,80],[165,120],[200,140],[235,120]]
	#print(newhexpts)
	vpts = [[105,100],[80,180],[115,210],[36,185]]
	tri_top = [[115,210],[36,185],[80,180]]
	tri_bot = [[36,185],[80,180],[105,100]]
	


	hexptsarr = np.array(hexpts,np.int32) 
	hexptsarr = hexptsarr.reshape((-1, 1, 2))
	

	vptsarr = np.array(vpts,np.int32) 
	vptsarr = vptsarr.reshape((-1, 1, 2))

	tri_toparr = np.array(tri_top,np.int32) 
	tri_toparr = tri_toparr.reshape((-1, 1, 2))
	tri_botarr = np.array(tri_bot,np.int32) 
	tri_botarr = tri_botarr.reshape((-1, 1, 2))


	if(delta>1):
		circle = [300,185,40+delta]

		boundaryDelta = [[0 + delta, 0 + delta], 
		[width - 1 - delta, 0+delta], 
		[width - 1 - delta, height - 1 - delta],
		[0+delta, height - 1 - delta]]
		boundaryDeltaarr = np.array(boundaryDelta, dtype=np.int32)

		boundary = [[0 , 0 ], 
		[width - 1 , 0], 
		[width - 1 , height - 1 ],
		[0, height - 1 ]]
		boundaryarr = np.array(boundary, dtype=np.int32)

		
		cv2.fillConvexPoly(img, boundaryarr, black)
		cv2.fillConvexPoly(img, boundaryDeltaarr, [255,255,255])
		

		newhexpts = expandHex(hexpts,delta)
		newhexptsarr = np.array(newhexpts,np.int32) 
		newhexptsarr = newhexptsarr.reshape((-1, 1, 2))

		newtritop = expandTriTop(tri_top,delta)
		newtri_toparr = np.array(newtritop,np.int32) 
		newtri_toparr = newtri_toparr.reshape((-1, 1, 2))

		newtribot = expandTriBot(tri_bot,delta)
		newtri_botarr = np.array(newtribot,np.int32) 
		newtri_botarr = newtri_botarr.reshape((-1, 1, 2))
	
		img = cv2.circle(img, (300, height-65), 40+delta, black, fill)
		img = cv2.circle(img, (300, height-65), 40, black, fill)
		#img = cv2.circle(img, (start[1],start[0]), 3, green, nofill)
		#img = cv2.circle(img, (goal[1],goal[0]), 3, red, nofill)
		#img[start[0],start[1]] = green

		img = cv2.fillPoly(img, [newhexptsarr], black)
		img = cv2.fillPoly(img, [hexptsarr], black)
		img1 = cv2.fillPoly(img, [newtri_toparr], black)
		img2 = cv2.fillPoly(img, [newtri_botarr], black)
		img = cv2.bitwise_and(img1, img2)
		img = cv2.fillPoly(img, [tri_toparr], black)
		img = cv2.fillPoly(img, [tri_botarr], black)

		shapes.append(newhexpts)
		shapes.append(tri_top)
		shapes.append(tri_bot)
		shapes.append(circle)
		shapes.append(boundaryDelta)
		shapes.append(boundary)

	else:
		circle = [300,185,40]
		boundary = [[0 , 0 ], 
		[width - 1 , 0], 
		[width - 1 , height - 1 ],
		[0, height - 1 ]]
		img = cv2.circle(img, (300, height-65), 40, black, fill)
		#img = cv2.circle(img, (start[1],start[0]), 3, green, nofill)
		#img = cv2.circle(img, (goal[1],goal[0]), 3, red, nofill)
		img[start[0],start[1]] = green
		img = cv2.fillPoly(img, [hexptsarr], black)
		img = cv2.fillPoly(img, [tri_toparr], black)
		img = cv2.fillPoly(img, [tri_botarr], black)

		shapes.append(hexpts)
		shapes.append(tri_top)
		shapes.append(tri_bot)
		shapes.append(circle)
		shapes.append(boundary)


	
	return img,shapes

class Node:
	def __init__(self,i,j,cost,parent_index):
		self.i=i
		self.j=j
		self.cost=np.round(cost,3)
		self.index = (self.j*width)+self.i
		self.parent_index = parent_index


	def __str__(self):
		return "("+str(self.i)+" "+str(self.j)+")"+" "+str(self.cost)+" "+str(self.index)+" "+str(self.parent_index)

def calcNodeIndex(node):
	return (node.j) * width + (node.i)

def up(current,img,space):
	# checkInObstacle(space,[200,100])
	move_x = 1
	move_y = 0
	x = current.i 
	y = current.j 
	c = current.cost 
	c_id = current.index
	move_cost = 1
	# print(x,y)
	try:
		check = img[x][y-1].tolist()
	except:
		check = [0,0,0]
	if x < height-1 and check!=[0,0,0]: #and not inside obstacle and not obstacle
		node = Node(current.i + move_x,current.j + move_y,current.cost + move_cost, c_id)
		node.index = calcNodeIndex(node)
		return node
	else:
	
		return None

def down(current,img,space):
	move_x = -1
	move_y = 0
	x = current.i 
	y = current.j 
	c = current.cost 
	c_id = current.index
	move_cost = 1
	# print(x,y)
	try:
		check = img[x][y+1].tolist()
	except:
		check = [0,0,0]
	if x > 0 and check!=[0,0,0]: #and not inside obstacle and not obstacle
		node = Node(current.i + move_x,current.j + move_y,current.cost + move_cost, c_id)
		node.index = calcNodeIndex(node)
		return node
	else:
	
		return None

def upleft(current,img,space):
	move_x = 1
	move_y = -1
	x = current.i 
	y = current.j 
	c = current.cost 
	c_id = current.index
	move_cost = 1
	# print(x,y)
	try:
		check = img[x-1][y-1].tolist()
	except:
		check = [0,0,0]
	if y > 0 and x<height-1 and check!=[0,0,0]: #and not inside obstacle and not obstacle
		node = Node(current.i + move_x,current.j + move_y,current.cost + move_cost, c_id)
		node.index = calcNodeIndex(node)
		return node
	else:
	
		return None

def right(current,img,space):
	move_x = 0
	move_y = 1
	x = current.i 
	y = current.j 
	c = current.cost 
	c_id = current.index
	move_cost = 1
	try:
		check = img[x+1][y].tolist()
	except:
		check = [0,0,0]
	# print(x,y)
	if x < width-1 and check!=[0,0,0]: #and not inside obstacle and not obstacle
		node = Node(current.i + move_x,current.j + move_y,current.cost + move_cost, c_id)
		node.index = calcNodeIndex(node)
		return node
	else:
		return None

def left(current,img,space):
	move_x = 0
	move_y = -1
	x = current.i 
	y = current.j 
	c = current.cost 
	c_id = current.index
	move_cost = math.sqrt(2)
	# print(x,y)
	try:
		check = img[x-1][y].tolist()
	except:
		check = [0,0,0]
	if y > 0  and check!=[0,0,0]: #and not inside obstacle and not obstacle
		node = Node(current.i + move_x,current.j + move_y,current.cost + move_cost, c_id)
		node.index = calcNodeIndex(node)
		return node
	else:
	
		return None

def upright(current,img,space):
	move_x = 1
	move_y = 1
	x = current.i 
	y = current.j 
	c = current.cost 
	c_id = current.index
	move_cost = math.sqrt(2)
	try:
		check = img[x+1][y-1].tolist()
	except:
		check = [0,0,0]
	if x < width-1 and x < height-1 and check!=[0,0,0]: #and not inside obstacle and not obstacle
		node = Node(current.i + move_x,current.j + move_y,current.cost + move_cost, c_id)
		node.index = calcNodeIndex(node)
		return node
	else:
	
		return None

def downleft(current,img,space):
	move_x = -1
	move_y = -1
	x = current.i 
	y = current.j 
	c = current.cost 
	c_id = current.index
	move_cost = math.sqrt(2)
	try:
		check = img[x-1][y+1].tolist()
	except:
		check = [0,0,0]
	if x > 0 and y > 0 and check!=[0,0,0]: #and not inside obstacle and not obstacle
		node = Node(current.i + move_x,current.j + move_y,current.cost + move_cost, c_id)
		return node
	else:
		
		return None

def downright(current,img,space):

	move_x = -1
	move_y = 1
	x = current.i 
	y = current.j 
	c = current.cost 
	c_id = current.index
	# print(x,y)
	move_cost = math.sqrt(2)
	try:
		check = img[x+1][y+1].tolist()
	except:
		check = [0,0,0]
	if x > 0 and y < width-1 and check!=[0,0,0]: #and not inside obstacle and not obstacle
		node = Node(current.i + move_x,current.j + move_y,current.cost + move_cost, c_id)
		node.index = calcNodeIndex(node)
		return node
	else:
		return None

def expand(current,img,space):
	neighbors = []
	neighbors.append(up(current,img,space))
	neighbors.append(upright(current,img,space))
	neighbors.append(right(current,img,space))
	neighbors.append(downright(current,img,space))
	neighbors.append(down(current,img,space))
	neighbors.append(downleft(current,img,space))
	neighbors.append(left(current,img,space))
	neighbors.append(upleft(current,img,space))
	neighbors = [i for i in neighbors if i]
	return neighbors


def checkStartGoal(start,goal,boundary):
	boundary_max = sorted(boundary)[-1]
	boundary_min = sorted(boundary)[-1]
	 
	start = start.reverse()
	goal = goal.reverse()
	if (start[0] > bl[0] and p[0] < tr[0] and p[1] > bl[1] and p[1] < tr[1]) :
		return True
	else :
		return False

def checkInCircle(space,point):
	circle = space[3]
	check = 0 
	# print('m',circle)
	#[300,185,40]
	f=(point[0]-circle[0])**2+(point[1]-circle[1])**2-(circle[2])**2 ## clearance is included 40+5
	if f<=0:
		print("CIR")
		return 1 ## the point is on or in the obstacle
	else: 
		return 0 ## not in the obstacle

def checkInTri(space,point):
	t1 = space[1]
	t2 = space[2]
	check = []
	for i in range(len(t1)):
		[a,b,c,m,xc,yc]=lineFromPoints(t1[i-1],t1[i])
		if a*point[0]+b*point[1]-c >= 0:
			check.append(1)
		else: 
			check.append(0)
		#print('c',check)
	if(check==[0,0,0]):
		print("TRI1")
		flag1 = 1
	else:
		flag1 = 0
	check=[]
	for i in range(len(t2)):
		[a,b,c,m,xc,yc]=lineFromPoints(t2[i-1],t2[i])
		if a*point[0]+b*point[1]-c >= 0:
			check.append(1)
		else: 
			check.append(0)
		# print('c',check)
	if(check==[1,1,1]):
		print("TRI2")
		flag2 = 1
	else:
		flag2 = 0

	if(flag1 or flag2):
		return 1
	else:
		return 0

	


def checkInHex(space,point):
	x,y=point[0],point[1]
	f1=0.577*x+y-261.677#bl
	f2=x-240#r
	f3=0.577*x-y-61.628#ul
	f4=0.577*x+y-169.252#ur
	f5=x-160#l
	f6=0.577*x-y+30.698#br
	#print(f1,f2,f3,f4,f5,f6)
	# if (f1<=0 and f2<=0 and f3<=0 and f4>=0 and f5>=0 and f6>=0):
	#     return True ## xy is in obstacle
	# else:
	#     return False ##xy is not in obstacle
	#print(f2,f6,f1,f5,f3,f4)
	check = []
	bounding = space[0]
	for i in range(len(bounding)):
		[a,b,c,m,xc,yc]=lineFromPoints(bounding[i-1],bounding[i])
		if a*point[0]+b*point[1]-c >= 0:
			check.append(1)
		else: 
			check.append(0)
	if(check==[1,1,1,1,1,1]):
		print("HEX")
		return 1
	else:
		return 0

def checkInObstacle(space,point):
	point = [point[1],point[0]]
	# print("000",point)
	check1 = checkInHex(space,point)
	check2 = checkInTri(space,point)
	check3 = checkInCircle(space,point)
	if(check1 or check2 or check3): 
		return 1
	else:
		return 0

	
def planning(start,goal,dimension,clearance,flag=0):
	start_node = Node(start[0],start[1], 0.0, -1)
	goal_node = Node(goal[0],goal[1], 0.0, -1)
	start_node.index = calcNodeIndex(start_node)

	
	print("Start:\t",start_node.__str__())
	print("Goal:\t",goal_node.__str__())
	img,space = buildObsMap(start,goal,dimension,clearance)
	# print(space)

	boundary = space[-1]
	boundary = sorted(boundary)
	# print("Bounds: ",boundary)
	if(checkInObstacle(space,start)):
		print("Start node inside one obstacle!")
		sys.exit()
	if(checkInObstacle(space,goal)):
		print("Goal node inside one obstacle!")
		sys.exit()
	


	obs = img

	
	
	if(img[start[0]][start[1]].tolist()==[0,0,0]):
		print("Start node inside obstacle!")
		print("Bounds are ",boundary[0],boundary[-1])
		sys.exit()
	
	if(img[goal[0]][goal[1]].tolist()==[0,0,0]):
		print("Goal node inside obstacle!")
		sys.exit()
	
	open_set, closed_set = dict(), dict()
	open_set[calcNodeIndex(start_node)] = start_node
	#print(open_set)
	img_arr = []
	while(len(open_set)>0):
		# print(len(open_set))
		# print("looking...")
		c_id = min(open_set, key=lambda o: open_set[o].cost)

		current = open_set[c_id]
		#print(current.__str__())

		if current.j == goal_node.j and current.i == goal_node.i:
			print("DONE.")
			goal_node.parent_index = current.parent_index
			goal_node.cost = current.cost
			print("Goal: \t", goal_node)
			return goal_node,closed_set,img,img_arr
		# Remove the item from the open set
		del open_set[c_id]

		# Add it to the closed set
		closed_set[c_id] = current

		neighbors = expand(current,img,space)

		# for move_x, move_y, move_cost in motion:
		# 	temp = Node(current.i + move_x,current.j + move_y,current.cost + move_cost, c_id)
		# 	n_id = calcNodeIndex(temp)
		# 	setColorPixel(img,temp.j,temp.i,red)
		# 	flip = cv2.flip(img, 0)
		# cv2.imshow('image', flip)
		# cv2.waitKey(0)

		for temp in neighbors:
			if temp!= None:
				#print(temp.i,temp.j)
				try:
					#if(img[temp.j][temp.i]!=[0,0,0]):
				#setColorPixel(img,temp.j,temp.i,green)
					img[temp.i][temp.j] = green
					img = cv2.bitwise_and(img, obs)
				
					img[start[0],start[1]] = blue
					img[goal[0],goal[1]] = red
						
				except:
					continue
				n_id = calcNodeIndex(temp)

				if n_id in closed_set:
					continue

				
				if n_id not in open_set:
					open_set[n_id] = temp  # Discover a new node
				else:
					if open_set[n_id].cost >= temp.cost:
				# This path is the best until now. record it!
						open_set[n_id] = temp
		
		if(flag):
			flip = cv2.flip(img, 0)
			img_arr.append(flip)
			cv2.imshow('image', flip)
			cv2.waitKey(1)
		else:
			flip = cv2.flip(img, 0)
			img_arr.append(flip)
	print("Goal not reached.")
	return start_node,closed_set,img,img_arr

def backtrack(goal_node,closed_set,img,img_arr):
	print("Total Cost to Traverse: ",goal_node.cost," units")
	parent_index = goal_node.parent_index
	path = []
	path.append([goal_node.i,goal_node.j])
	while(parent_index!=-1):
		n = closed_set[parent_index]
		#print(n)
		path.append([n.i,n.j])
		parent_index = n.parent_index
	#print(path)

	for point1, point2 in zip(path, path[1:]): 
		path = cv2.line(img, [point1[1],point1[0]], [point2[1],point2[0]], blue, 1) 
		#path = cv2.line(img, points[0].ravel(), points[1].ravel(), (0, 255, 0), thickness=3, lineType=8)
		path = cv2.flip(path, 0)
		img_arr.append(path)
		cv2.imshow("image", path)
	for i in range(50):
		img_arr.append(img_arr[-1])
	cv2.waitKey(0)
	return img_arr


def main():
	# +y,+x 
	#[y,x] [y,x]
	# print("Default Settings:\nRobot dimension = 0\nObstacle clearance distance = 0\nStart: [0,0]\nGoal = [398,248]\nLive Animation = False")
	# default = input("Choose Default? (y/n)")

	# if(default=='y'):
	# 	dimension=0
	# 	clearance=0
	# 	startx = 0
	# 	starty = 0
	# 	goalx = 398
	# 	goaly = 248
	# 	anim = 0
	# else:
	dimension = int(input("Robot dimension: "))
	clearance = int(input("Obstacle clearance distance: "))
	startx = int(input("Start X: "))
	starty = int(input("Start Y: "))
	goalx = int(input("Goal X: "))
	goaly = int(input("Goal Y: "))

	store = input("Video Save? (y/n):")
	if(store=='y'):
		store=1
	else:
		store=0

	# anim = input("Live Animation? (y/n): \nIf 'y': Exploration is saved in output video\nIf 'n': Exploration is NOT saved in output video")
	anim = input("Live Animation? (y/n):")
	if anim=='y':
		anim=1
	else:
		anim=0

	delta = dimension+clearance

	#goal_node,closed_set,img = planning([0,0],[249,399])
#	goal_node,closed_set,img,img_arr = planning([0,0],[5,5],dimension,clearance)
	start_time = timeit.default_timer()	
	goal_node,closed_set,img,img_arr = planning([starty,startx],[goaly,goalx],dimension,clearance,anim)
	elapsed = timeit.default_timer() - start_time
	print("Time taken: ",elapsed," seconds")
	
	start_time = timeit.default_timer()
	img_arr = backtrack(goal_node,closed_set,img,img_arr)
	elapsed = timeit.default_timer() - start_time
	#print("Time taken: ",elapsed," seconds")


	# choose codec according to format needed
	if(store):
		video_name = "Djikstra.avi"
		fourcc = cv2.VideoWriter_fourcc(*'mp4v') 
		out = cv2.VideoWriter(video_name, fourcc, 30, (400, 250))
		#out = cv2.VideoWriter("video.avi", 0, 1, (250,400))
		# for i in range(len(img_arr)):
		# 	# out.write(img_arr[i])
		# 	cv2.imshow("check",img_arr[i])
		# 	cv2.waitKey(1)
		# cv2.destroyAllWindows()
		# fourcc = cv2.VideoWriter_fourcc('M', 'J', 'P', 'G')

		# out = cv2.VideoWriter('video.avi', fourcc, 1.0, img_arr[0].shape)

		for i in img_arr:
			out.write(i)
		out.release()
		print("Video File: ",video_name)
	
	#test cases
	#planning([223,370],[240,370],anim)
	#planning([10,25],[15,25],1)
	#planning([10,250],[10,300],1)
	#planning([100,80],[220,80],0)
	#planning([1,1],[200,398])

if __name__ == "__main__":
	main()