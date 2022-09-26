import itertools
import heapq

import numpy as np

class MinheapPQ:
    def __init__(self):
        self.pq = [] # lis of the entries arranged in a heap
        self.nodes = set()
        self.entry_finder = {} # mapping of the item entries
        self.counter = itertools.count() # unique sequence count
        self.REMOVED = '<removed-item>'
    
    def put(self, item, priority):
        if item in self.entry_finder:
            self.check_remove(item)
        count = next(self.counter)
        entry = [priority, count, item]
        self.entry_finder[item] = entry
        heapq.heappush(self.pq, entry)
        self.nodes.add(item)

    def check_remove(self, item):
        if item not in self.entry_finder:
            return
        entry = self.entry_finder.pop(item)
        entry[-1] = self.REMOVED
        self.nodes.remove(item)

    def get(self):
        while self.pq:
            priority, count, item = heapq.heappop(self.pq)
            if item is not self.REMOVED:
                del self.entry_finder[item]
                self.nodes.remove(item)
                return item
        raise KeyError('pop from an empty priority queue')

    def top_key(self):
        return self.pq[0][0]
        
    def enumerate(self):
        return self.pq

    def allnodes(self):
        return self.nodes


def check_in_2d_array(test,array):
    return any(np.array_equal(x, test) for x in array)

def find_index_in_2d_array(test,array):
    idx = 0
    for x in array:
        if np.array_equal(x, test):
            return idx
        else:
            idx += 1
    return False

def getDist(pos1, pos2):
    return np.sqrt(sum([(pos1[0] - pos2[0]) ** 2, (pos1[1] - pos2[1]) ** 2, (pos1[2] - pos2[2]) ** 2]))

def calcCost(initparams, pos1, pos2):
    pos_diff = (pos1[0] - pos2[0], pos1[1] - pos2[1], pos1[2] - pos2[2])
    # print(pos1, pos2)
    if pos1[2] == pos2[2]:
        cost = np.sqrt(pos_diff[0] ** 2 + pos_diff[1] ** 2)
        cost /= initparams.robot_speed
    else:
        if initparams.Alldirec[(pos_diff[0], pos_diff[1], pos_diff[2])] > 1: # staris
            if pos_diff[2] > 0: # stair down
                cost = (np.sqrt(pos_diff[0] ** 2 + pos_diff[1] ** 2 + pos_diff[2]**2))
                cost /= initparams.robot_speed
            else: # stair up
                cost = (np.sqrt(pos_diff[0] ** 2 + pos_diff[1] ** 2 + pos_diff[2]**2))
                cost /= initparams.robot_speed
        else: # elevators
            cost = (np.sqrt(pos_diff[0] ** 2 + pos_diff[1] ** 2 + pos_diff[2]**2))
            cost /= initparams.elv_speed
            
            addi = (abs(initparams.curr_elv_loc - initparams.start[2]) / initparams.elv_speed)
            
            cost += addi
            
    return cost

def heuristic_fun(initparams, k, t=None):
    if t is None:
        t = initparams.goal
    return max([abs(t[0] - k[0]), abs(t[1] - k[1]), abs(t[2] - k[2])])

def isCollide(initparams, x, child, roads, dist=None):
    if dist==None:
        dist = getDist(x, child)
    movement = np.array([child[0] - x[0], child[1] - x[1], child[2] - x[2]])
    mv_size = np.sqrt(movement[0]**2 + movement[1]**2 + movement[2]**2)
    if mv_size == 1:
        return False, dist
    elif movement[2] == 0: # movement on xy plane
        diag_points = [[x[0], x[1], x[2]], [x[0], x[1], x[2]]]
        is_extra_added = False
        for idx, mv_el in enumerate(movement):
            if abs(mv_el) > 0:
                if not is_extra_added:
                    diag_points[0][idx] += mv_el
                    is_extra_added = True
                else:
                    diag_points[1][idx] += mv_el
        for dp in diag_points:
            if not check_in_2d_array(dp, initparams.roads):
                # print("cost", x, child, dp)
                return True, dist
        return False, dist
    else:
        return False, dist

def children(initparams, x, settings = 0):
    allchild = []
    allcost = []
    resolution = initparams.resolution
    for direc in initparams.Alldirec:
        child = tuple(map(np.add, x, np.multiply(direc, resolution)))
        if check_in_2d_array(child, initparams.roads):
            allchild.append(child)
            allcost.append((child,initparams.Alldirec[direc]*resolution))
    if settings == 0:
        return allchild
    if settings == 1:
        return allcost

def cost(initparams, i, j, dist=None, settings='Euclidean'):
    if initparams.settings == 'NonCollisionChecking':
        if dist==None:
            dist = getDist(i,j)
        collide = False
    else:
        collide, dist = isCollide(initparams, i, j, dist)
    if settings == 'Euclidean':
        if collide:
            return np.inf
        else:
            return calcCost(initparams, i, j)