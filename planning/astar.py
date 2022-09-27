import utils

import numpy as np

import random

class Weighted_A_star(object):
    def __init__(self, s_start, s_goal, roads, resolution=1.0, rand_range=1, curr_elv_loc=1, elv_speed=1, robot_speed=1, z_voxel_resol=1.0):
        self.Alldirec = {(1, 0, 0): 1, (0, 1, 0): 1, (0, 0, 1): 1, \
                        (-1, 0, 0): 1, (0, -1, 0): 1, (0, 0, -1): 1, \
                        (1, 1, 0): np.sqrt(2), (0, 1, 1): np.sqrt(2), \
                        (-1, -1, 0): np.sqrt(2), (0, -1, -1): np.sqrt(2), \
                        (1, -1, 0): np.sqrt(2), (-1, 1, 0): np.sqrt(2),\
                        (0, 1, -1): np.sqrt(2), (0, -1, 1): np.sqrt(2),\
                        (1, 1, 1): np.sqrt(3), (1, -1, 1): np.sqrt(2),\
                        (-1, 1, 1): np.sqrt(3), (-1, -1, 1): np.sqrt(2),\
                        (1, 1, -1): np.sqrt(3), (1, -1, -1): np.sqrt(3),\
                        (-1, 1, -1): np.sqrt(3), (-1, -1, -1): np.sqrt(3)}
        self.settings = 'CollisionChecking' # 'NonCollisionChecking' or 'CollisionChecking'                
        self.start, self.goal = tuple(s_start), tuple(s_goal)
        self.g = {self.start:0,self.goal:np.inf}
        self.Parent = {}
        self.CLOSED = set()
        self.V = []
        self.done = False
        self.Path = []
        self.ind = 0
        self.x0, self.xt = self.start, self.goal
        self.OPEN = utils.MinheapPQ()  # store [point,priority]
        self.OPEN.put(self.x0, self.g[self.x0] + utils.heuristic_fun(self,self.x0))  # item, priority = g + h
        self.lastpoint = self.x0
        self.roads = roads
        self.resolution = resolution
        self.rand_range = rand_range
        self.rand_num = random.choice([i for i in range(self.rand_range)])
        self.curr_elv_loc = curr_elv_loc
        self.elv_speed = elv_speed
        self.robot_speed = robot_speed
        self.z_voxel_resol = z_voxel_resol

    def run(self, N=None):
        xt = self.xt
        xi = self.x0
        self.rand_num = random.choice([i for i in range(self.rand_range)])
        while self.OPEN:  # while xt not reached and open is not empty
            xi = self.OPEN.get()
            if xi not in self.CLOSED:
                self.V.append(np.array(xi))
            self.CLOSED.add(xi)  # add the point in CLOSED set
            if utils.getDist(xi,xt) < self.resolution:
                break
            for xj in utils.children(self,xi):
                if xj not in self.g:
                    self.g[xj] = np.inf
                else:
                    pass
                a = self.g[xi] + utils.cost(self, xi, xj)
                if a < self.g[xj]:
                    self.g[xj] = a
                    self.Parent[xj] = xi
                    # assign or update the priority in the open
                    self.OPEN.put(xj, a + 1 * utils.heuristic_fun(self, xj))
            if self.ind % 100 == 0: print('number node expanded = ' + str(len(self.V)))
            self.ind += 1

        self.lastpoint = xi
        # if the path finding is finished
        if self.lastpoint in self.CLOSED:
            self.done = True
            self.Path = self.path()
            return True

        return False

    def path(self):
        path = []
        x = self.lastpoint
        start = self.x0
        while x != start:
            path.append([x, self.Parent[x]])
            x = self.Parent[x]
        return path