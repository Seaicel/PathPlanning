import os
import sys
import math
import heapq
from node import Node


class CutEdge:
    def __init__(self):
        self.nodes = []
        self.border1 = ()
        self.border2 = ()


class AStar:

    def __init__(self):
        self.openset = []
        self.closeset = []
        self.parent = dict()
        self.g = dict()  # cost to come
        self.s_start = Node([0, 0], 0, lambda x: 0, 0, None)
        self.s_goal = Node([0, 0], 0, lambda x: 0, 0, None)
        self.map = []
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]

    def update_map(self, cur_node, inp_map):
        """ 输入：当前地址(node)，八叉图([node]))
            选择离当前位置最近(曼哈顿距离)的边界点作为目的地
        """
        self.map = inp_map
        self.s_start = cur_node
        self.openset = []
        self.closeset = []
        cut_edges = self.gen_cut_edges()
        if cut_edges:
            candidate = []
            node_p = {}
            k1 = 1
            k2 = 1
            for edge in cut_edges:
                sorted(edge.nodes, key=lambda x: abs(cur_node[0] - x[0]) + abs(cur_node[1] - x[1]))
                candidate.append(edge.nodes[0])
                theta = math.atan((edge.border1[1] - cur_node[1])/(edge.border1[0] - cur_node[0])) - math.atan((edge.border2[1] - cur_node[1])/(edge.border2[0] - cur_node[0]))
                if theta > math.pi:
                    theta -= 2 * math.pi
                if theta < -math.pi:
                    theta += 2 * math.pi
                theta = abs(theta / math.pi * 180.0)
                node_p[edge.nodes[0]] = k1 * (1 / (abs(cur_node[0] - edge.nodes[0][0])
                                                   + abs(cur_node[1] - edge.nodes[0][1]))) + k2 * theta
            sorted(candidate, key=lambda x: node_p[x], reverse=True)
            self.s_goal = candidate[0]
        else:
            self.s_goal = cur_node  # boundary为空，需要回溯，直到boundary不为空

    def gen_cut_edges(self, inp_map):
        """
        生成cut edges
        :param inp_map: 局部地图，暂时假设0为空地，1为墙，2为未知
        :return: cut edges，用坐标点表示
        """
        boundary = []
        for i in range(len(inp_map)):
            for j in range(len(inp_map[0])):
                if inp_map[i][j] == 0:
                    if self.is_boundary(inp_map, i, j):
                        boundary.append((i, j))

        cut_edges = []
        while len(boundary):
            cut_edge = CutEdge()
            cut_edge.nodes = [boundary[0]]
            self.find_adjacent(boundary[0], boundary, cut_edge)
            if not cut_edge.border2:
                cut_edge.border2 = cut_edge.nodes[0]
            cut_edges.append(cut_edge)

        return cut_edges

    def is_boundary(self, inp_map, i, j):
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

    def find_adjacent(self, cur_node, boundary, cut_edge):
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
                self.find_adjacent(adj_node, boundary, cut_edge)
        if flag is False:
            if cut_edge.border1:
                cut_edge.border1 = cur_node
            elif cut_edge.border2:
                cut_edge.border2 = cur_node

    def searching(self):
        """ A*算法实现
            返回：路径，closeset
        """
        heapq.heappush(self.openset,
                       (self.f_value(self.s_start), self.s_start))

        while self.openset:
            _, s = heapq.heappop(self.openset)
            self.closeset.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in self.get_neighbor(s):
                new_cost = s.g + self.cost(s, s_n)

                if new_cost < s_n.g:  # conditions for updating Cost
                    s_n.g = new_cost
                    s_n.parent = s
                    heapq.heappush(self.openset, (self.f_value(s_n), s_n))

        return self.extract_path(self.parent), self.closeset

    def get_neighbor(self, s):
        return [Node((s[0] + u[0], s[1] + u[1]), 0, lambda x: 0, 0, None, []) for u in self.motions]

    def cost(self, s_start, cur_node):
        """ 计算从起始位置start到当前位置pos的cost
        """

        if self.is_collision(cur_node):
            return math.inf

        return abs(cur_node.pos[0] - s_start.pos[0]) + abs(cur_node.pos[1] - s_start.pos[1])

    def is_collision(self, n_pos):
        """ 判断节点是否会发生碰撞
        """
        return n_pos.status == 1

    def f_value(self, n_pos):
        """ f = g + h. (g: Cost to come, h: heuristic value)
        """
        return n_pos.g + self.heuristic(n_pos)

    def heuristic(self, s):
        """
        Calculate heuristic.
        :param s: current node (state)
        :return: heuristic function value
        """

        goal = self.s_goal  # goal node

        return abs(goal[0] - s[0]) + abs(goal[1] - s[1])

    def extract_path(self, parent):
        """
        Extract the path based on the parent set.
        :return: The planning path
        """

        path = [self.s_goal]
        s = self.s_goal

        while True:
            s = parent[s]
            path.append(s)

            if s == self.s_start:
                break

        return list(path)


def main():
    astar = AStar()


if __name__ == '__main__':
    main()
