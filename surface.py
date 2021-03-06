import numpy as np
import open3d as o3d
import heapq
import octmap
'''
    compute surface and height grids
    input: octmap, 3D pos (center of robot), radius of robot, unit width
'''

class Surface:
    def __init__(self, octree, start, r, w):
        self.octree = octree
        self.start = start
        self.r = r
        self.w = w
        self.motion = np.array([[-1,0,0], [1,0,0], [0,1,0], [0,-1,0], [-1,-1,0], [-1,1,0], [1,-1,0], [1,1,0]])
        min_bound = self.octree.get_min_bound()
        max_bound = self.octree.get_max_bound()
        self.bounding_box = np.array([[x for x in min_bound], [x for x in max_bound]])
        self.x_num, self.y_num, _ = np.ceil((self.bounding_box[1] - self.bounding_box[0]) / w).astype(int)
        self.mark_offset = np.array([[1,0,0], [-1,0,0], [0,1,0], [0,-1,0], [0,0,1]])
        # print("bounding box", self.bounding_box, self.x_num, self.y_num)

    def pos_valid(self, pos):
        jud = True
        for i in range(len(self.mark_offset)):
            oct_data = self.octree.locate_leaf_node(np.ndarray(shape=(3,1), buffer=np.array(pos + self.mark_offset[i] * self.r, dtype=float)))
            if oct_data[0] is not None:  # locate in obstacles
                jud = False
        return jud


    def in_view(self, pos):
        return pos[0] > 0 and pos[1] > 0 and pos[0] < self.y_num and pos[1] < self.x_num


    def coord2pos(self, coord):
        return self.bounding_box[0] + coord * self.w


    def coord_valid(self, coord):
        pos = self.coord2pos(coord)
        return self.pos_valid(pos)

    def pos2coord(self, pos):
        return (pos - self.bounding_box[0]) / self.w

    def coord_reachable(self, coord, next_coord):
        # whether next_coord is reachable from coord (sample point in the path coord->next_coord)
        for i in range(1, 10):
            pos = self.coord2pos((10-i)/10 * coord + i / 10 * next_coord)
            oct_data = self.octree.locate_leaf_node(np.ndarray(shape=(3,1), buffer=np.array(pos, dtype=float)))
            # print(i, pos, coord, next_coord, oct_data)
            if oct_data[0] is not None:
                return False
        return True



    def get_surface(self):
        surface = np.full((self.y_num, self.x_num), 2, dtype=float)
        start_coord = (self.start - self.bounding_box[0]) / self.w
        height_grid = np.full((self.y_num, self.x_num), self.start[2], dtype=float)
        s = []       # dfs search
        s.append(start_coord)
        while s:
            top = s.pop()
            # print(top, self.coord_valid(top), self.coord2pos(top))
            if self.coord_valid(top):
                surface[int(top[0]), int(top[1])] = 0
                for direction in self.motion:
                    next_coord = (top + direction)
                    if self.coord_reachable(top, next_coord) and self.in_view(next_coord) and surface[int(next_coord[0]), int(next_coord[1])] == 2:
                        s.append(next_coord)
            else:
                surface[int(top[0]), int(top[1])] = 1
        surface[surface == 2] = 1

        return surface, height_grid


def main():
    octree = octmap.gen_octree()
    surface, grid = Surface(octree, [0, 0, -2], 20, 1).get_surface()
    print(surface)
    print(grid)


if __name__ == '__main__':
    main()
