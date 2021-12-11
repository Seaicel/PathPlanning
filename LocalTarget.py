import math

'''
This file includes functions to calculate local target
'''


class CutEdge:
    def __init__(self):
        self.nodes = []
        self.border1 = ()
        self.border2 = ()


def cal_local_target(global_map, x_start, y_start, height_grid):
    cut_edges = gen_cut_edges(global_map)
    if cut_edges:
        candidate = []
        node_p = {}
        k1 = 1
        k2 = 1
        for edge in cut_edges:
            sorted(edge.nodes, key=lambda x: abs(x_start - x[0]) + abs(y_start - x[1]))
            candidate.append(edge.nodes[0])
            theta = math.atan((edge.border1[1] - y_start) / (edge.border1[0] - x_start)) - math.atan(
                (edge.border2[1] - y_start) / (edge.border2[0] - x_start))
            if theta > math.pi:
                theta -= 2 * math.pi
            if theta < -math.pi:
                theta += 2 * math.pi
            theta = abs(theta / math.pi * 180.0)
            node_p[edge.nodes[0]] = k1 * (1 / (abs(x_start - edge.nodes[0][0])
                                               + abs(y_start - edge.nodes[0][1]))) + k2 * theta
        sorted(candidate, key=lambda x: node_p[x], reverse=True)
        s_goal = candidate[0]
    else:
        s_goal = (x_start, y_start)  # boundary为空，需要回溯，直到boundary不为空
    return s_goal


def gen_cut_edges(global_map):
    """
    生成cut edges
    :param global_map: 局部地图，暂时假设0为空地，1为墙，2为未知
    :return: cut edges，用坐标点表示
    """
    boundary = []
    for i in range(len(global_map)):
        for j in range(len(global_map[0])):
            if global_map[i][j] == 0:
                if is_boundary(global_map, i, j):
                    boundary.append((i, j))

    cut_edges = []
    while len(boundary):
        cut_edge = CutEdge()
        cut_edge.nodes = [boundary[0]]
        find_adjacent(boundary[0], boundary, cut_edge)
        if not cut_edge.border2:
            cut_edge.border2 = cut_edge.nodes[0]
        cut_edges.append(cut_edge)

    return cut_edges


def is_boundary(inp_map, i, j):
    """
    通过八连通，判断一个点周围的八个点有没有未知的，如果有，表示这个点是一个boundary
    :return: true or false
    """
    l_bound = max(j - 1, 0)
    r_bound = min(j + 1, len(inp_map[0]) - 1)
    up_bound = max(i - 1, 0)
    down_bound = min(i + 1, len(inp_map) - 1)
    return ((inp_map[i][l_bound] == 2) or (inp_map[i][r_bound] == 2) or
            (inp_map[up_bound][j] == 2) or (inp_map[down_bound][j] == 2) or
            (inp_map[up_bound][l_bound] == 2) or (inp_map[up_bound][r_bound] == 2) or
            (inp_map[down_bound][l_bound] == 2) or (inp_map[down_bound][r_bound] == 2))


def find_adjacent(cur_node, boundary, cut_edge):
    """
    从一个boundary点出发，构造一条cut edge
    """
    boundary.remove(cur_node)
    motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
               (1, 0), (1, -1), (0, -1), (-1, -1)]
    flag = False  # 用于判断是否是最后一次递归，如果是，这个点是cut edge 最边缘的一点，在计算视野角度时使用
    for u in motions:
        adj_node = (cur_node[0] + u[0], cur_node[1] + u[1])
        if adj_node in boundary:
            flag = True
            cut_edge.nodes.append(adj_node)
            find_adjacent(adj_node, boundary, cut_edge)
    if flag is False:
        if cut_edge.border1:
            cut_edge.border1 = cur_node
        elif cut_edge.border2:
            cut_edge.border2 = cur_node
