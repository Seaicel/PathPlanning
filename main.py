import octmap
from surface import Surface
import numpy as np
import DstarLite as ds

octree = octmap.gen_octree()

surface = Surface(octree, np.array((1,1,0)), 0.1, 1)

surface_grid, height_grid = surface.get_surface()
print(surface_grid)
print(height_grid)

surface_grid[surface_grid==1] = np.inf
path, sensed_map, updated_points = ds.Main(surface_grid, height_grid, 19, 19, 1, 1)

print(path)

for point in path:
    surface_grid[tuple(point)] = 2

surface_grid[surface_grid > 10] = 1
print(surface_grid)