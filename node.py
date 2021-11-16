#!/usr/bin/env python

import numpy as np
import math
class node():
	global pos=[]
	global g=math.inf
	global h            # heuristic function
	global status=0     # 0: 空地，1：墙
	global parent
    global neighbors = []
	# initializing nodes
	def __init__(self, pos, g, h, status, parent, neighbors):
		self.pos = pos
		self.g = g
		self.h = h
		self.status = status
		self.parent = parent
        self.neighbors = neighbors
