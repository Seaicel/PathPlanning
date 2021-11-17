#!/usr/bin/env python

import numpy as np
import math


class Node:
    # initializing nodes
    def __init__(self, pos, g, h, status, parent):
        self.pos = pos
        self.g = g
        self.h = h  # heuristic function
        self.status = status  # 0: 空地，1：墙
        self.parent = parent

