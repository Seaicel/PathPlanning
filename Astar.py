import os
import sys
import math
import heapq
from node import Node


class AStar:

    def __init__(self):
        self.openset = []
        self.closeset = []
        self.parent = dict()
        self.g = dict()  # cost to come
        self.s_start = Node([0, 0], 0, lambda x: 0, 0, None, [])
        self.s_goal = Node([0, 0], 0, lambda x: 0, 0, None, [])
        self.map = []
        self.motions = [(-1, 0), (-1, 1), (0, 1), (1, 1),
                        (1, 0), (1, -1), (0, -1), (-1, -1)]

    def update_map(self, cur_node, inp_map, boundary):
        """ 输入：当前地址(node)，八叉图([node])，视野内的未知边界([node])
            选择离当前位置最近(曼哈顿距离)的边界点作为目的地
        """
        self.map = inp_map
        self.s_start = cur_node
        self.openset = []
        self.closeset = []
        if boundary:
            sorted(boundary, key=lambda x: abs(self.s_start.pos[0] - x.pos[0]) + abs(self.s_start.pos[1] - x.pos[1]))
            self.s_goal = boundary[0]
        else:
            self.s_goal = cur_node  # boundary为空，需要回溯，直到boundary不为空

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
