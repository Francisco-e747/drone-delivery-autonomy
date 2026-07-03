import numpy as np
import random
import math
import time

class Node:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.parent = None
        self.cost = 0.0

class RRTStar:
    def __init__(self, start, goal, obstacles, bounds, max_iter=1000,
                 step_size=5.0, search_radius=15.0, entropy_weight=0.3):
        self.start = Node(*start)
        self.goal = Node(*goal)
        self.obstacles = obstacles  # list of (x, y, radius)
        self.bounds = bounds        # (xmin, xmax, ymin, ymax)
        self.max_iter = max_iter
        self.step_size = step_size
        self.search_radius = search_radius
        self.entropy_weight = entropy_weight
        self.nodes = [self.start]
        self.past_paths = []        # list of past paths for entropy

    def distance(self, n1, n2):
        return math.sqrt((n1.x - n2.x)**2 + (n1.y - n2.y)**2)

    def sample(self):
        if random.random() < 0.1:
            return Node(self.goal.x, self.goal.y)
        x = random.uniform(self.bounds[0], self.bounds[1])
        y = random.uniform(self.bounds[2], self.bounds[3])
        return Node(x, y)

    def nearest(self, node):
        return min(self.nodes, key=lambda n: self.distance(n, node))

    def steer(self, from_node, to_node):
        d = self.distance(from_node, to_node)
        if d < self.step_size:
            return Node(to_node.x, to_node.y)
        theta = math.atan2(to_node.y - from_node.y, to_node.x - from_node.x)
        new = Node(from_node.x + self.step_size * math.cos(theta),
                   from_node.y + self.step_size * math.sin(theta))
        return new

    def collision_free(self, n1, n2):
        for (ox, oy, r) in self.obstacles:
            for t in np.linspace(0, 1, 10):
                x = n1.x + t * (n2.x - n1.x)
                y = n1.y + t * (n2.y - n1.y)
                if math.sqrt((x - ox)**2 + (y - oy)**2) < r:
                    return False
        return True

    def path_entropy_cost(self, node):
        if not self.past_paths:
            return 0.0
        cost = 0.0
        for path in self.past_paths:
            for px, py in path:
                d = math.sqrt((node.x - px)**2 + (node.y - py)**2)
                if d < self.step_size * 2:
                    cost += math.exp(-d / self.step_size) * 10.0
        return cost / max(len(self.past_paths), 1)

    def plan(self):
        t0 = time.time()
        for _ in range(self.max_iter):
            rand = self.sample()
            nearest = self.nearest(rand)
            new = self.steer(nearest, rand)

            if not self.collision_free(nearest, new):
                continue

            # find nearby nodes
            near_nodes = [n for n in self.nodes
                         if self.distance(n, new) < self.search_radius]

            # choose best parent
            best_parent = nearest
            best_cost = nearest.cost + self.distance(nearest, new)
            for near in near_nodes:
                if self.collision_free(near, new):
                    c = near.cost + self.distance(near, new)
                    c += self.entropy_weight * self.path_entropy_cost(new)
                    if c < best_cost:
                        best_cost = c
                        best_parent = near

            new.parent = best_parent
            new.cost = best_cost
            self.nodes.append(new)

            # rewire
            for near in near_nodes:
                if self.collision_free(new, near):
                    new_cost = new.cost + self.distance(new, near)
                    if new_cost < near.cost:
                        near.parent = new
                        near.cost = new_cost

            # check goal
            if self.distance(new, self.goal) < self.step_size:
                self.goal.parent = new
                self.goal.cost = new.cost + self.distance(new, self.goal)
                path = self.extract_path()
                elapsed = (time.time() - t0) * 1000
                return path, elapsed

        return None, (time.time() - t0) * 1000

    def extract_path(self):
        path = []
        node = self.goal
        while node is not None:
            path.append((node.x, node.y))
            node = node.parent
        return list(reversed(path))

    def add_past_path(self, path):
        self.past_paths.append(path)
        if len(self.past_paths) > 10:
            self.past_paths.pop(0)
