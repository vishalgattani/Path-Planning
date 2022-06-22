from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import sys
import time
import trimesh 
import numpy as np
import os
import inspect


from trimesh.exchange.binvox import voxelize_mesh
from trimesh import voxel as v
from trimesh.voxel import creation
import matplotlib.pyplot as plt
import math
import random 
import pandas as pd
from stl import mesh as stlmesh
from mpl_toolkits import mplot3d
from scipy.spatial import KDTree
from trimesh.ray import ray_pyembree
from pykdtree.kdtree import KDTree as pyKDTree

random.seed(1)

# parameter
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 2.5  # [m] Maximum edge length
minDist = 0.1 # [m] Minimum edge length
maxDist = MAX_EDGE_LEN

pitch = 0.2
num_points = 1500 # 50,100,200,250,400,500
startpoint = [0,0.5,-5.0] # blender y is -z
goalpoint = [0,10.0,0] # blender z is y
dist_from_goal = 1.0
filename = "test.obj"

class Edge:
    """
    Edge class for roadmap
    """
    def __init__(self,nodei,nodej,euclideanDist):
        self.start = [nodei.x,nodei.y,nodei.z]
        self.end = [nodej.x,nodej.y,nodej.z]
        self.edgeLength = euclideanDist

    def __str__(self):
        return str(self.start)+"---"+str(self.edgeLength)+"---"+str(self.end)
    
    def __eq__(self, other):
        if (isinstance(other, Edge)):
            return self.start == other.start and self.end == other.end
        return False
    
    def getEdgeLength(self):
        return self.edgeLength

class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, z, cost, parent_index):
        self.x = x
        self.y = y
        self.z = z
        self.cost = np.round(cost,3)
        self.parent_index = parent_index
        self.numEdges = 0
        self.neighbors = []
        self.id = 0
        self.gscore = self.cost
        self.fscore = math.inf
        self.parent = None

    def __str__(self):
        return str(self.x)+","+str(self.y)+","+str(self.z)#+","+str(self.cost) + "," + str(self.parent_index)
    def __eq__(self, other):
        if (isinstance(other, Node)):
            return self.x == other.x and self.y == other.y and self.z == other.z
        return False
    
    def set_numEdges(self,num):
        self.numEdges = num
    def get_numEdges(self):
        return self.numEdges
    def addNeighbors(self,neighbor):
        self.neighbors.append(neighbor)
    def setNodeIndex(self,idno):
        self.id = idno
    def getNodeIndex(self):
        return self.id
    def calcfscore(self,goal):
        self.fscore = self.cost+c2g(self,goal)
        self.h = c2g(self,goal)
    def getNeighbors(self):
        l = []
        for i in self.neighbors:
            neighborpts = np.array((i.x,i.y,i.z))
            l.append(neighborpts)
        return l
    def setParent(self,parent):
        self.parent = parent

def c2g(current,goal):
    point1 = np.array((current.x,current.y,current.z))
    point2 = np.array((goal.x,goal.y,goal.z))
    return np.linalg.norm(point1 - point2)

def pc2g(point,node):
    point1 = point
    point2 = np.array((node.x,node.y,node.z))
    return np.linalg.norm(point1 - point2)

def as_mesh(scene_or_mesh):
    """
    Convert a possible scene to a mesh.

    If conversion occurs, the returned mesh has only vertex and face data.
    """
    if isinstance(scene_or_mesh, trimesh.Scene):
        if len(scene_or_mesh.geometry) == 0:
            mesh = None  # empty scene
        else:
            # we lose texture information here
            mesh = trimesh.util.concatenate(
                tuple(trimesh.Trimesh(vertices=g.vertices, faces=g.faces)
                    for g in scene_or_mesh.geometry.values()))
    else:
        assert(isinstance(mesh, trimesh.Trimesh))
        mesh = scene_or_mesh
    return mesh

def printVar(var):
    print(type(var))
    print(var)


def createScene(geometry):
    scene = trimesh.Scene([])
    scene.add_geometry(geometry)
#     print(scene)
    return scene

def rotate_objects(scene):
    # create an empty homogenous transformation matrix for rotating the sphere
    matrix = np.eye(4)

    # get the delta of the time. We need this as trimesh does not contain a function for automatic incrementation of movements
    delta_change = time.time()
    # set Y as sin of time. We can change the speed of rotation by multiplying the delta_change with a scalar
    val = round(np.sin(-delta_change*2),3)
    # print(val)
    matrix[0][3] = np.sin(-delta_change*2)
    # set Z as cos of time
    matrix[2][3] = np.cos(-delta_change*2)

    # create a y axis rotation for the angel mesh. We use the trimesh.transofmration.rotation_matrix which requires rotation angle and an axis
    yaxis = [0,1,0]
    Ry = trimesh.transformations.rotation_matrix(delta_change, yaxis)

    # Get the nodes for the sphere and the mesh. You can also directly call them with their created mesh containers or keep which node is which mesh in a dictionary
    # For our purpose we only have two objects so we just select them in the way they are added to the scene
#     node_sphere = s.graph.nodes_geometry[0]
    node_mesh = scene.graph.nodes_geometry[0]
    for i in scene.graph.nodes_geometry:
	    # apply the transform to the node and update the scene
	    scene.graph.update(i, matrix=Ry)

def showScene(geometry):
    scene = trimesh.Scene([])
    scene.add_geometry(geometry)
    return scene.show()

def viewVoxelScene(mesh_to_voxels):
    scene = trimesh.Scene([])
    scene.add_geometry(mesh_to_voxels.as_boxes(colors=(0, 0, 1, 0.3)))
    return scene.show()


def loadObj(file_obj):
    mesh = trimesh.load(file_obj)
    print("Mesh bounds: ",mesh.bounds)  
    print("Min Bound: ",mesh.bounds[0])
    print("Max Bound: ",mesh.bounds[1])
    print("Mesh extent: ",mesh.extents)
    return mesh

def getPCD(mesh,num_points):
    print("Generating Point Cloud by sampling",num_points,"points...")
    points = mesh.bounding_box_oriented.sample_volume(count=num_points)
    return points

def concatenatePointsPCD(points,list_points):
    arr = np.asarray(list_points)
    points = np.concatenate((arr,points))
    return points

def mesh2voxels(mesh,pitch):
    mesh = as_mesh(mesh)
    print("Voxelizing with pitch (side length of cube): ", pitch)
    start_time = time.time()
    mesh_voxels = creation.voxelize(mesh, pitch=pitch, method='subdivide')
    mesh_voxels = v.VoxelGrid(mesh_voxels.encoding.dense,mesh_voxels.transform)
    mesh_voxels.fill(method='holes')
    print("--- %s seconds ---" % (time.time() - start_time))
    print("Finished Voxelizing...")
    return mesh_voxels

def voxels2mesh(mesh_voxels,pitch):
    mesh_voxels_center_points = mesh_voxels.points
    voxelizedMesh = trimesh.voxel.ops.multibox(mesh_voxels_center_points, pitch=pitch, colors=(0,0,255,64))
    return voxelizedMesh

def getInsideOutsidePoints(mesh_voxels,pcd):
    inpoints = []
    outpoints = []
    for i in pcd:
        if mesh_voxels.is_filled(i):
            # print("Filled")
            inpoints.append(i)
        else:
            # print("NOT Filled")
            outpoints.append(i)
    print("Outside voxels\t:",len(outpoints))
    print("Inside voxels\t:",len(inpoints))
    return inpoints,outpoints

def prm(points,mesh_voxels,minDist,maxDist,N_KNN):
    nodes = []
    edges = []
    n_sample = len(points)
    sample_x = points[:,0]
    sample_y = points[:,1]
    sample_z = points[:,2]

    innodes = []
    outnodes = []

    newpts = []
    for i in range(n_sample):
        node = Node(sample_x[i],sample_y[i],sample_z[i],i,0)
        if node:
            if mesh_voxels.is_filled(points[i]):
                innodes.append(node)
            else:
                newpts.append(points[i])
                outnodes.append(node)
                nodes.append(node)

    print("Generating all possible free paths with MAX_EDGE_LEN",MAX_EDGE_LEN,"...")
    start_time = time.time()
    edgeCountMax = 0

    plausible_points = []

    for i in range(len(nodes)):
        if i%100==0:
            print("Parsing through node",i,"/",len(outnodes))
        edgeCount = 0
        nodeiedges = []
        for j in range(i,len(nodes)):
                
            # while(edgeCount<=N_KNN):
                # print(edgeCount)
                diff_x = nodes[i].x - nodes[j].x
                diff_y = nodes[i].y - nodes[j].y
                diff_z = nodes[i].z - nodes[j].z
                euclideanDist = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
                if euclideanDist >= minDist/10 and euclideanDist <= maxDist:
                    
                    ray_origins = np.array([[nodes[i].x,nodes[i].y,nodes[i].z]])
                    ray_directions = np.array([[-diff_x,-diff_y,-diff_z]])
                    locations, index_ray, index_tri = voxelizedMesh.ray.intersects_location(ray_origins=ray_origins,ray_directions=ray_directions)
                    #if points along edge not inside voxelizedMesh:
                    if len(locations)==0:
                        plausible_points.append([nodes[i].x,nodes[i].y,nodes[i].z])
                        plausible_points.append([nodes[j].x,nodes[j].y,nodes[j].z])
                        possibleEdge = Edge(nodes[i],nodes[j],euclideanDist)
                        edges.append(possibleEdge)
                        nodeiedges.append(possibleEdge)
                        nodes[i].addNeighbors(nodes[j])
                        nodes[i].set_numEdges(len(nodeiedges))
                        edgeCount+=1
                        if(len(nodeiedges)>=N_KNN):
                            edgeCountMax = len(nodeiedges)
                            break
                        
    # print("Max Edges from a possible node",edgeCountMax)
    print("--- %s seconds ---" % (time.time() - start_time))
    print("Finished creating",len(edges),"edges...")            
    # print("Total edges possible =",len(edges))
    print("Joining",len(edges),"edges for visualization...")
    edges_list = []
    for edge in edges:
        segments = np.vstack((edge.start,edge.end))
        path = trimesh.load_path(segments, process=False)
        edges_list.append(path)
    return edges,edges_list,newpts


def expand(current,exppts):
    neighborslist = []
    
    for j in exppts:
            
        # while(edgeCount<=N_KNN):
            # print(edgeCount)
            diff_x = current.x - j[0]
            diff_y = current.y - j[1]
            diff_z = current.z - j[2]
            euclideanDist = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
            if euclideanDist >= minDist/1 and euclideanDist <= maxDist:
                ray_origins = np.array([[current.x,current.y,current.z]])
                ray_directions = np.array([[-diff_x,-diff_y,-diff_z]])
                locations, index_ray, index_tri = voxelizedMesh.ray.intersects_location(ray_origins=ray_origins,ray_directions=ray_directions)
                #if points along edge not inside voxelizedMesh:
                if len(locations)==0:
                    neighborslist.append(j)
                    if len(neighborslist)>=N_KNN:
                        break
    return neighborslist


def astar_prm(newpts):


    start_node = Node(newpts[0][0],newpts[0][1],newpts[0][2],0,0)
    goal_node = Node(newpts[1][0],newpts[1][1],newpts[1][2],0,0)
    print("A* Algorithm from",start_node,"to",goal_node)

    start_node.setNodeIndex(0)
    goal_node.setNodeIndex(0)

    open_set, closed_set = dict(), dict()
    open_set[start_node.getNodeIndex()] = start_node


    while 1:
        
        if len(open_set) == 0:
            print("Open set is empty...")
            return None,None
            break
        c_id = min(open_set,key=lambda o: open_set[o].cost + c2g(goal_node,open_set[o]))
        current = open_set[c_id]
        # print(len(open_set),"\t\t",np.round(c2g(goal_node,current),2))

        if current==goal_node or c2g(goal_node,current)<=dist_from_goal:
            print("Found goal!")
            goal_node.parent_index = current.parent_index
            goal_node.cost = current.cost
            goal_node.setParent(current.parent)
            return start_node,goal_node
            break
        del open_set[c_id]

        closed_set[c_id] = current
        neighbors = expand(current,newpts)  

        for neighbor in neighbors:
            node = Node(neighbor[0],neighbor[1],neighbor[2],current.cost + pc2g(neighbor,current), c_id)
            for e in range(len(newpts)):
                if np.array_equiv(newpts[e],neighbor):
                    node.setNodeIndex(e) 
            n_id = node.getNodeIndex()
            node.setParent(current)
            if n_id in closed_set:
                continue
            if n_id not in open_set:
                open_set[n_id] = node  # discovered a new node
            else:
                if open_set[n_id].cost >= node.cost:
                    # This path is the best until now. record it
                    open_set[n_id] = node


def backtrack(start_node,goal_node):
    temp = goal_node
    backtrack_edgelist = []
    distance_to_travel = 0
    while temp!=start_node: 
        inode = temp
        temp = temp.parent
        fnode = temp
        possibleEdge = Edge(inode,fnode,c2g(inode,fnode))
        backtrack_edgelist.append(possibleEdge)
        # print(possibleEdge) 
        distance_to_travel += possibleEdge.getEdgeLength()
    backtrack_paths = []
    for edge in backtrack_edgelist:
        segments = np.vstack((edge.start,edge.end))
        path = trimesh.load_path(segments, process=False)
        backtrack_paths.append(path)
    return backtrack_edgelist,distance_to_travel



print("3D PRM")
mesh = loadObj(filename)
mesh_scene = createScene(mesh)

# mesh_scene.show(callback=rotate_objects)
# sys.exit()

points = getPCD(mesh_scene,num_points)
points = np.genfromtxt('points_data.csv', delimiter=',')

arr = np.asarray([startpoint,goalpoint])
print("Start point \t:",arr[0])
print("Goal point \t:",arr[1])
points = np.concatenate((arr,points))

index = (points / pitch).round().astype(int)
pcd = trimesh.points.PointCloud(points,colors=[[0,0,0,64] for i in points])
initialpoints = trimesh.points.PointCloud(points[0:2],colors=[[0,255,0,255] for i in points[0:2]])


mesh_to_voxels = mesh2voxels(mesh,pitch)

voxelizedMesh = voxels2mesh(mesh_to_voxels,pitch)
inpoints,outpoints = getInsideOutsidePoints(mesh_to_voxels,pcd)
pcd_in = trimesh.points.PointCloud(inpoints,colors=[[255,0,0,255] for i in inpoints])
pcd_out = trimesh.points.PointCloud(outpoints,colors=[[0,0,0,64] for i in outpoints])



edges,edges_list,newpts = prm(points,mesh_to_voxels,minDist,maxDist,N_KNN)


lines = []
for edge in edges:
    segments = np.vstack((edge.start,edge.end))
    path = trimesh.load_path(segments, process=False)
    lines.append(path)

    
startSphere = trimesh.primitives.Sphere(radius=0.2,center=startpoint,subdivisions=2)
goalSphere = trimesh.primitives.Sphere(radius=0.2,center=goalpoint,subdivisions=2)
startSphere.visual.face_colors = np.asarray([0,255,0,255],dtype=np.uint8)
goalSphere.visual.face_colors = np.asarray([255,0,0,255],dtype=np.uint8)

scene = createScene([voxelizedMesh,lines,startSphere,goalSphere])
scene.show(callback=rotate_objects)   


start_node,goal_node = astar_prm(newpts)
if goal_node:
    bt_paths,distance_to_travel = backtrack(start_node,goal_node)
else:
    print("Sample >",num_points,"points maybe...")

print(distance_to_travel)

flag = 0
if goal_node and len(bt_paths)>0:
	flag = 1
	backtrack_paths = []
	for edge in bt_paths:
		segments = np.vstack((edge.start,edge.end))
		path = trimesh.load_path(segments, process=False)
		backtrack_paths.append(path)

	startSphere = trimesh.primitives.Sphere(radius=0.2,center=startpoint,subdivisions=2)
	goalSphere = trimesh.primitives.Sphere(radius=0.2,center=goalpoint,subdivisions=2)
	startSphere.visual.face_colors = np.asarray([0,255,0,255],dtype=np.uint8)
	goalSphere.visual.face_colors = np.asarray([255,0,0,255],dtype=np.uint8)
    

if flag:
	# showScene([voxelizedMesh,backtrack_paths,startSphere,goalSphere])
	scene = createScene([voxelizedMesh,backtrack_paths,startSphere,goalSphere])
	scene.show(callback=rotate_objects)   

	





