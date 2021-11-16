import os
import sys
import math
import heapq
from node import node

class AStar:

    def __init__(self):
        self.openset = []
        self.closeset = []
        self.parent = dict()
        self.g = dict()  # cost to come
        self.s_start = node([0,0], 0, lambda x: 0, 0, None, [])
        self.s_goal = node([0,0], 0, lambda x: 0, 0, None, [])
        self.map = []


    def update_map(self, cur_node, inp_map, boundary):
        """ 输入：当前地址(node)，八叉图([node])，视野内的未知边界([node])
            选择离当前位置最近(曼哈顿距离)的边界点作为目的地
        """
        self.map = inp_map
        self.s_start = cur_node
        self.openset = []
        self.closeset = []
        if boundary:
            sorted(boundary, key = lambda x: abs(s_start.pos[0] - x.pos[0]) + abs(s_start.pos[1] - x.pos[1]))
            self.s_goal = boundary[0]
        else:
            self.s_goal = now_pos   # 当前视野内全部探索完毕
        

    def searching(self):
        """ A*算法实现
            返回：路径，closeset
        """
        heapq.heappush(self.openset,
                       (self.s_start.get_value(), self.s_start))

        while self.openset:
            _, s = heapq.heappop(self.openset)
            self.closeset.append(s)

            if s == self.s_goal:  # stop condition
                break

            for s_n in s.neighbors:
                new_cost = s.g + self.cost(s, s_n)

                if new_cost < s_n.g:  # conditions for updating Cost
                    s_n.g = new_cost
                    s_n.parent = s
                    heapq.heappush(self.openset, (s_n.get_value(), s_n))

        return self.extract_path(self.parent), self.closeset

    def get_neighbor(self, s):
        return s.get_neighbor()

    def cost(self, s_start, cur_node):
        """ 计算从起始位置start到当前位置pos的cost
        """

        if self.is_collision(cur_node):
            return math.inf

        return abs(cur_node.pos[0] - s_start.pos[0]) + abs(cur_node.pos[1] - s_start.pos[1])

    def is_collision(self, n_pos):
        """ 判断节点是否会发生碰撞
        """
        return n_pos.get_status() === 1

    def f_value(self, n_pos):
        """ f = g + h. (g: Cost to come, h: heuristic value)
        """
        return n_pos.get_value()

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