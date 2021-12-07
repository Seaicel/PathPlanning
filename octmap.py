import open3d as o3d
import numpy as np
from numpy.random import random_integers as rnd

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

def gen_octree(width=20, height=20, depth=5):

    np_maze = maze(width, height)
    print(np_maze)

    depth = 5

    np_map = []
    for i in range(len(np_maze)):
        for j in range(len(np_maze[0])):
            if np_maze[i][j] == 1:
                # np_map.append([i, j, 0])
                for h in range(depth):
                    np_map.append([i, j, h/2])

    for i in range(len(np_maze)):
        for j in range(len(np_maze[0])):
            if np_maze[i][j] == 1:
                if i != 0 and np_maze[i-1][j] == 1:
                    for h in range(depth):
                        np_map.append([i-0.5, j, h/2])
                if i != len(np_maze)-1 and np_maze[i+1][j] == 1:
                    for h in range(depth):
                        np_map.append([i+0.5, j, h/2])
                if j != 0 and np_maze[i][j-1] == 1:
                    for h in range(depth):
                        np_map.append([i, j-0.5, h/2])
                if j != len(np_maze[0])-1 and np_maze[i][j+1] == 1:
                    for h in range(depth):
                        np_map.append([i, j+0.5, h/2])


    print(np_map)

    pcd =  o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(np_map)

    # o3d.visualization.draw_geometries([pcd])

    octree = o3d.geometry.Octree(max_depth=5) #(max_depth=7, origin=[[1,1,0]], size=1)
    octree.convert_from_point_cloud(pcd) #(pcd, size_expand=0.001)

    # o3d.visualization.draw_geometries([octree])
    return octree

octree = gen_octree()
o3d.visualization.draw_geometries([octree])