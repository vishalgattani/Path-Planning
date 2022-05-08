from __future__ import absolute_import
from __future__ import division
from __future__ import print_function

import time
import trimesh 
import numpy as np
import sys
import os
import inspect
from trimesh.exchange.binvox import voxelize_mesh
from trimesh import voxel as v
from trimesh.voxel import creation
import matplotlib.pyplot as plt
import math

from stl import mesh as stlmesh

from mpl_toolkits import mplot3d



from scipy.spatial import KDTree
from trimesh.ray import ray_pyembree
from pykdtree.kdtree import KDTree as pyKDTree

# parameter
N_KNN = 10  # number of edge from one sampled point
MAX_EDGE_LEN = 50.0  # [m] Maximum edge length
minDist = 0.1
maxDist = MAX_EDGE_LEN


pitch = 0.2
# 50,100,200,250,400,500
num_points = 25   
file_obj = "test.obj"



fig = plt.figure()
ax = plt.axes(projection='3d')




class Edge:
    """
    Edge class for roadmap
    """
    def __init__(self,nodei,nodej,euclideanDist):
        self.start = [nodei.x,nodei.y,nodei.z]
        self.end = [nodej.x,nodej.y,nodej.z]
        self.edgeLength = euclideanDist

    def __str__(self):
        return str(self.start)+"---"+self.edgeLength+"---"+str(self.end)


class Node:
    """
    Node class for dijkstra search
    """

    def __init__(self, x, y, z, cost=0, parent_index=0):
        self.x = x
        self.y = y
        self.z = z
        self.cost = cost
        self.parent_index = parent_index

    def __str__(self):
        return str(self.x)+","+str(self.y)+","+str(self.z)#+","+str(self.cost) + "," + str(self.parent_index)



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


scene = trimesh.Scene([])
point = np.asarray([[0.0,0.0,0.0]]) # right x , out z , up y
origin = trimesh.points.PointCloud(point)
col = np.asarray([0,0,0,255],dtype=np.uint8)
origin.vertices_color = col
scene.add_geometry(origin)




print("Loading ",file_obj,"...")
mesh = trimesh.load(file_obj) # with plane to increase the bounding box
# scene.add_geometry(mesh)

print("Mesh bounds: ",mesh.bounds)  
print("Min Bound: ",mesh.bounds[0])
print("Max Bound: ",mesh.bounds[1])
print("Mesh extent: ",mesh.extents)



print("Point Cloud of ",num_points," Sampled Points")
points = mesh.bounding_box_oriented.sample_volume(count=num_points)
index = (points / pitch).round().astype(int)
pcd = trimesh.points.PointCloud(points,colors=[[0,0,0,64] for i in points])



createPCD = True
if createPCD:
    scene.add_geometry(pcd)


voxelize = True
if voxelize:
    mesh = as_mesh(mesh)
    print("Voxelizing with pitch (side length of cube): ", pitch)
    start_time = time.time()
    mesh_voxels = creation.voxelize(mesh, pitch=pitch, method='subdivide')
    mesh_voxels = v.VoxelGrid(mesh_voxels.encoding.dense,mesh_voxels.transform)
    # mesh_voxels.fill(method='holes')
    print("--- %s seconds ---" % (time.time() - start_time))
    print("Finished Voxelizing...")
    scene.add_geometry(mesh_voxels.as_boxes(colors=(0, 0, 1, 0.3)))

    # print(mesh_voxels.points.shape)
    # print("1 Filled", mesh_voxels.filled_count)
    mesh_voxels.fill(method='holes')
    # print(mesh_voxels.points.shape)
    # print("2 Filled", mesh_voxels.filled_count)


print("Creating Ray Object...")
ray_inter = ray_pyembree.RayMeshIntersector(mesh)
print(ray_inter)
# rtrace = ray_inter.intersects_location(starts, ends, multiple_hits=False)

if voxelize:
    inpoints = []
    outpoints = []
    for i in pcd:
        if mesh_voxels.is_filled(i):
            # print("Filled")
            inpoints.append(i)
        else:
            # print("NOT Filled")
            outpoints.append(i)
    if createPCD:
        pcd1 = trimesh.points.PointCloud(inpoints,colors=[[255,0,0,255] for i in inpoints])
        scene.add_geometry(pcd1)
        pcd2 = trimesh.points.PointCloud(outpoints,colors=[[0,255,0,255] for i in outpoints])
        scene.add_geometry(pcd2)

    print("Outside voxels\t:",len(outpoints))
    print("Inside voxels\t:",len(inpoints))




nodes = []
# nodes.append(startnode)
# nodes.append(goalnode)

edges = []
road_map = []
n_sample = len(points)
sample_x = points[:,0]
sample_y = points[:,1]
sample_z = points[:,2]

innodes = []
outnodes = []

for i in range(n_sample):
    node = Node(sample_x[i],sample_y[i],sample_z[i])
    if node:
        if mesh_voxels.is_filled(points[i]):
            innodes.append(node)
        else:
            outnodes.append(node)
            nodes.append(node)



print("Generating all possible free paths with MAX_EDGE_LEN ",MAX_EDGE_LEN,"...")
for i in range(len(nodes)):
    for j in range(i,len(nodes)):
        diff_x = nodes[i].x - nodes[j].x
        diff_y = nodes[i].y - nodes[j].y
        diff_z = nodes[i].z - nodes[j].z
        euclideanDist = math.sqrt(diff_x**2 + diff_y**2 + diff_z**2)
        if euclideanDist >= minDist and euclideanDist <= maxDist:
            ray_origins = np.array([[nodes[i].x,nodes[i].y,nodes[i].z]])
            ray_directions = np.array([[-diff_x,-diff_y,-diff_z]])
            locations, index_ray, index_tri = mesh.ray.intersects_location(ray_origins=ray_origins,ray_directions=ray_directions)
            if len(locations)==0:
                # print(nodes[i],"  ",nodes[j],"\tN")
                possibleEdge = Edge(nodes[i],nodes[j],euclideanDist)
                edges.append(possibleEdge)    

            
print("Total edges possible = ",len(edges))

print("Joining",len(edges),"edges for visualization...")
for edge in edges:
    edge.start
    segments = np.vstack((edge.start,edge.end))
    path = trimesh.load_path(segments, process=False)
    # col = np.asarray([0,255,0,128],dtype=np.uint8)
    # path.vertices_color = col
    # printVar(path)
    trimesh.visual.color.ColorVisuals(mesh=path, vertex_colors=[255,0,0,255])
    scene.add_geometry(path)
    ax.plot3D([edge.start[0],edge.end[0]], [edge.start[1],edge.end[1]], [edge.start[2],edge.end[2]], 'red')

your_mesh = stlmesh.Mesh.from_file('test.stl')
ax.add_collection3d(mplot3d.art3d.Poly3DCollection(your_mesh.vectors))
# Auto scale to the mesh size
# scale = your_mesh.points.flatten(-1)
# ax.auto_scale_xyz(scale, scale, scale)
ax.view_init(-90, 90)

ax.set_xlabel('x')
ax.set_ylabel('y')
ax.set_zlabel('z')
plt.show()



# # rotate the axes and update
# for angle in range(0, 360):
#     ax.view_init(30, angle)
#     plt.draw()
#     plt.pause(.001)


scene.show()

sys.exit()
