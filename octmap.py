import open3d as o3d
import numpy as np
from numpy.random import random_integers as rnd

class Octree:
    def __init__(self, data, resolution):
        self.data = data
        self.resolution = resolution
        self.tree = self.gen_octree(data)

    def gen_octree(self, data):
        pass




def maze(width=81, height=51, complexity=.75, density =.75):
    # Only odd shapes
    shape = ((height//2)*2+1, (width//2)*2+1)
    # Adjust complexity and density relative to maze size
    complexity = int(complexity*(5*(shape[0]+shape[1])))
    density    = int(density*(shape[0]//2*shape[1]//2))
    # Build actual maze
    Z = np.zeros(shape, dtype=float)
    # Fill borders
    Z[0,:] = Z[-1,:] = 1
    Z[:,0] = Z[:,-1] = 1
    # Make isles
    for i in range(density):
        x, y = rnd(0,shape[1]//2)*2, rnd(0,shape[0]//2)*2
        Z[y,x] = 1
        for j in range(complexity):
            neighbours = []
            if x > 1:           neighbours.append( (y,x-2) )
            if x < shape[1]-2:  neighbours.append( (y,x+2) )
            if y > 1:           neighbours.append( (y-2,x) )
            if y < shape[0]-2:  neighbours.append( (y+2,x) )
            if len(neighbours):
                y_,x_ = neighbours[rnd(0,len(neighbours)-1)]
                if Z[y_,x_] == 0:
                    Z[y_,x_] = 1
                    Z[y_+(y-y_)//2, x_+(x-x_)//2] = 1
                    x, y = x_, y_
    return Z

def gen_pointcloud_from_map(np_maze):
    depth = 1

    np_map = []
    for i in range(len(np_maze)):
        for j in range(len(np_maze[0])):
            if np_maze[i][j] == 1:
                # np_map.append([i, j, 0])
                for h in range(depth):
                    np_map.append([i, j, h/10])

    for i in range(len(np_maze)):
        for j in range(len(np_maze[0])):
            if np_maze[i][j] == 1:
                for num in range(10):
                    if i != 0 and np_maze[i-1][j] == 1:
                        for h in range(depth):
                            np_map.append([i-num/10, j, h/10])
                    if i != len(np_maze)-1 and np_maze[i+1][j] == 1:
                        for h in range(depth):
                            np_map.append([i+num/10, j, h/10])
                    if j != 0 and np_maze[i][j-1] == 1:
                        for h in range(depth):
                            np_map.append([i, j-num/10, h/10])
                    if j != len(np_maze[0])-1 and np_maze[i][j+1] == 1:
                        for h in range(depth):
                            np_map.append([i, j+num/10, h/10])
    return np_map


def gen_pointcloud(width=20, height=20, depth=5):
    np_maze = maze(width, height)
    print(np_maze)

    depth = 1

    np_map = []
    for i in range(len(np_maze)):
        for j in range(len(np_maze[0])):
            if np_maze[i][j] == 1:
                # np_map.append([i, j, 0])
                for h in range(depth):
                    np_map.append([i, j, h/10])

    for i in range(len(np_maze)):
        for j in range(len(np_maze[0])):
            if np_maze[i][j] == 1:
                for num in range(10):
                    if i != 0 and np_maze[i-1][j] == 1:
                        for h in range(depth):
                            np_map.append([i-num/10, j, h/10])
                    if i != len(np_maze)-1 and np_maze[i+1][j] == 1:
                        for h in range(depth):
                            np_map.append([i+num/10, j, h/10])
                    if j != 0 and np_maze[i][j-1] == 1:
                        for h in range(depth):
                            np_map.append([i, j-num/10, h/10])
                    if j != len(np_maze[0])-1 and np_maze[i][j+1] == 1:
                        for h in range(depth):
                            np_map.append([i, j+num/10, h/10])
    return np_map

def gen_octreee_from_points(np_map):
    pcd =  o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_map)
    # o3d.visualization.draw_geometries([pcd])

    octree = o3d.geometry.Octree(max_depth=5, origin= np.ndarray(shape=(3,1), buffer=np.array([0,10,0], dtype=float)), size=25) #(max_depth=7, origin=[[1,1,0]], size=1)
    # print(1, octree.get_axis_aligned_bounding_box(), octree.get_center(), octree.is_empty())
    # print(2, octree.size, octree.to_voxel_grid(), octree.origin)
    # print(3, octree.get_min_bound(), type(octree.get_min_bound()))
    octree.convert_from_point_cloud(pcd) #(pcd, size_expand=0.001)
    return octree

def gen_octree(width=20, height=20, depth=5):

    np_map = gen_pointcloud(width, height, depth)

    # print(np_map)

    pcd =  o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_map)
    # o3d.visualization.draw_geometries([pcd])

    octree = o3d.geometry.Octree(max_depth=5, origin= np.ndarray(shape=(3,1), buffer=np.array([0,10,0], dtype=float)), size=25) #(max_depth=7, origin=[[1,1,0]], size=1)
    print(1, octree.get_axis_aligned_bounding_box(), octree.get_center(), octree.is_empty())
    print(2, octree.size, octree.to_voxel_grid(), octree.origin)
    print(3, octree.get_min_bound(), type(octree.get_min_bound()))
    octree.convert_from_point_cloud(pcd) #(pcd, size_expand=0.001)

    # o3d.visualization.draw_geometries([octree])
    return octree

# octree = gen_octree()

# print(1, octree.get_axis_aligned_bounding_box(), octree.get_center(), octree.is_empty())
# print(2, octree.size, octree.to_voxel_grid(), octree.origin)
# print(3, octree.get_min_bound(), type(octree.get_min_bound()))
# for i in range(5):
#     temp_octree_data = octree.locate_leaf_node(np.ndarray(shape=(3,1), buffer=np.array([i,i,i], dtype=float))) # [octcolor, octNodeInfo]
#     print(i, type(temp_octree_data[0]), temp_octree_data[0])
#     print(i, type(temp_octree_data[1]), temp_octree_data[1])

# o3d.visualization.draw_geometries([octree])